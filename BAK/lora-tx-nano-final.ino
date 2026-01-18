// Arduino Nano Transmitter — Pulse Oximeter + LoRa Alert System
// Combines MAX30102 pulse oximeter with RA-02 (SX1278) 433 MHz LoRa
//
// Wiring: RA-02 Module -> Arduino Nano
// -----------------------------------------
// VCC   -> 3.3V from EXTERNAL regulator (Nano's 3.3V is too weak!)
// GND   -> GND (common ground with Nano and external supply)
// NSS   -> D10 (SS)
// MOSI  -> D11 (hardware SPI)
// MISO  -> D12 (hardware SPI)
// SCK   -> D13 (hardware SPI)
// RST   -> D9
// DIO0  -> D2
//
// Wiring: MAX30102 -> Arduino Nano
// -----------------------------------------
// VCC   -> 3.3V or 5V (module has regulator)
// GND   -> GND
// SDA   -> A4
// SCL   -> A5
//
// Wiring: OLED SSD1306 (I2C) -> Arduino Nano
// -----------------------------------------
// VCC   -> 3.3V or 5V
// GND   -> GND
// SDA   -> A4
// SCL   -> A5
//
// Wiring: LM35 Temperature Sensor
// -----------------------------------------
// VCC   -> 5V
// GND   -> GND
// OUT   -> A0
//
// Wiring: Buzzer
// -----------------------------------------
// +     -> D3 (PWM pin)
// -     -> GND
//
// IMPORTANT: Arduino Nano's onboard 3.3V regulator cannot provide
//            enough current for LoRa transmission (~120mA peak).
//            Use an external AMS1117-3.3V or similar regulator!

#include <SPI.h>
#include <LoRa.h>
#include "ssd1306h.h"
#include "MAX30102.h"
#include "Pulse.h"
#include <avr/pgmspace.h>

// TaskScheduler configuration - MUST be before include
#define _TASK_MICRO_RES          // Microsecond resolution for precise timing
#define _TASK_PRIORITY           // Enable layered priority scheduling
#include <TaskScheduler.h>

// ============ PIN DEFINITIONS ============
#define LED LED_BUILTIN
#define BUZZER 3
#define LM35_PIN A0

// LoRa pins
#define LORA_SS    10
#define LORA_RST   9
#define LORA_DIO0  2

// ============ CONSTANTS ============
// LoRa
const long LORA_FREQ = 433E6;

// Pulse oximeter thresholds
#define FINGER_THRESHOLD 5000
#define MAX_BPM 200
#define SENSOR_INTERVAL 4000      // 4ms = 250Hz
#define DISPLAY_INTERVAL 50000    // 50ms
#define BEAT_FLASH_DURATION 30    // ms
#define TEMP_READ_INTERVAL 1000000 // 1s

// Alert thresholds
#define ALERT_BPM_THRESHOLD 100
#define ALERT_TEMP_THRESHOLD 40
#define ALERT_DURATION_MS 3000   // 30 seconds sustained condition

// ============ OBJECTS ============
SSD1306 oled;
MAX30102 sensor;
Pulse pulseIR;
Pulse pulseRed;
MAFilter bpm;

// ============ HEART BITMAP ============
static const uint8_t heart_bits[] PROGMEM = {
  0x00, 0x00, 0x38, 0x38, 0x7c, 0x7c, 0xfe, 0xfe, 0xfe, 0xff,
  0xfe, 0xff, 0xfc, 0x7f, 0xf8, 0x3f, 0xf0, 0x1f, 0xe0, 0x0f,
  0xc0, 0x07, 0x80, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00
};

// ============ ECG WAVEFORM ============
const uint8_t MAXWAVE = 128;
const uint8_t WAVE_BASELINE = 52;
const uint8_t WAVE_TOP = 20;
const uint8_t WAVE_BOTTOM = 60;

const int8_t ECG_PATTERN[] PROGMEM = {
  0, 2, 4, 2, 0, -3, 20, -8, 0, 3, 8, 4, 0
};
const uint8_t ECG_PATTERN_LEN = 13;

class Waveform {
public:
  Waveform() {
    wavep = 0;
    ecgIndex = 0;
    inBeat = false;
    for (int i = 0; i < MAXWAVE; i++) {
      waveform[i] = 0;
    }
  }

  void record(bool beatDetected) {
    if (beatDetected && !inBeat) {
      inBeat = true;
      ecgIndex = 0;
    }
    if (inBeat) {
      waveform[wavep] = pgm_read_byte(&ECG_PATTERN[ecgIndex]);
      ecgIndex++;
      if (ecgIndex >= ECG_PATTERN_LEN) {
        inBeat = false;
        ecgIndex = 0;
      }
    } else {
      waveform[wavep] = 0;
    }
    wavep = (wavep + 1) % MAXWAVE;
  }

  void draw(uint8_t X) {
    for (int i = 32; i < MAXWAVE; i += 32) {
      for (uint8_t y = WAVE_TOP; y <= WAVE_BOTTOM; y += 6) {
        oled.drawPixel(X + i, y);
      }
    }
    uint8_t index = wavep;
    for (int i = 0; i < MAXWAVE - 1; i++) {
      int y1 = WAVE_BASELINE - waveform[index];
      index = (index + 1) % MAXWAVE;
      int y2 = WAVE_BASELINE - waveform[index];
      if (y1 < WAVE_TOP) y1 = WAVE_TOP;
      if (y1 > WAVE_BOTTOM) y1 = WAVE_BOTTOM;
      if (y2 < WAVE_TOP) y2 = WAVE_TOP;
      if (y2 > WAVE_BOTTOM) y2 = WAVE_BOTTOM;
      oled.drawPixel(X + i, y1);
      if (y2 > y1) {
        oled.drawVLine(X + i, y1, y2 - y1 + 1);
      } else if (y1 > y2) {
        oled.drawVLine(X + i, y2, y1 - y2 + 1);
      }
    }
    int yLast = WAVE_BASELINE - waveform[(wavep + MAXWAVE - 1) % MAXWAVE];
    if (yLast < WAVE_TOP) yLast = WAVE_TOP;
    if (yLast > WAVE_BOTTOM) yLast = WAVE_BOTTOM;
    oled.drawPixel(X + MAXWAVE - 1, yLast);
  }

private:
  int8_t waveform[MAXWAVE];
  uint8_t wavep = 0;
  uint8_t ecgIndex = 0;
  bool inBeat = false;
} wave;

// ============ STATE VARIABLES ============
int beatAvg = 0;
int SPO2 = 99;
int temperature = 0;
const bool filter_for_graph = false;
const bool draw_Red = false;

volatile bool fingerPresent = false;
volatile unsigned long lastBeatTime = 0;

// Alert state tracking (non-blocking)
unsigned long bpmAlertStartTime = 0;
unsigned long tempAlertStartTime = 0;
bool bpmAlertActive = false;
bool tempAlertActive = false;
bool alertSent = false;  // Prevent repeated sends

// ============ SCHEDULERS ============
Scheduler sensorScheduler;
Scheduler displayScheduler;

// Forward declarations
void sensorCallback();
void displayCallback();
void tempCallback();
void buzzerOffCallback();
void alertCheckCallback();

// Tasks
Task taskSensor(SENSOR_INTERVAL, TASK_FOREVER, &sensorCallback, &sensorScheduler, true);
Task taskBuzzerOff(BEAT_FLASH_DURATION * 1000, TASK_ONCE, &buzzerOffCallback, &sensorScheduler, false);
Task taskDisplay(DISPLAY_INTERVAL, TASK_FOREVER, &displayCallback, &displayScheduler, true);
Task taskTemp(TEMP_READ_INTERVAL, TASK_FOREVER, &tempCallback, &displayScheduler, true);
Task taskAlertCheck(500000, TASK_FOREVER, &alertCheckCallback, &displayScheduler, true);  // Check every 500ms

// ============ HELPER FUNCTIONS ============
void print_digit(int x, int y, long val, char c = ' ', uint8_t field = 3, const int BIG = 2) {
  uint8_t ff = field;
  bool first = true;
  do {
    char ch;
    if (first || val != 0) {
      ch = val % 10 + '0';
      first = false;
    } else {
      ch = c;
    }
    oled.drawChar(x + BIG * (ff - 1) * 6, y, ch, BIG);
    val = val / 10;
    --ff;
  } while (ff > 0);
}

// Temperature smoothing
#define TEMP_SAMPLES 8
int tempBuffer[TEMP_SAMPLES];
uint8_t tempIndex = 0;
bool tempBufferFilled = false;

int readTemperatureFiltered() {
  int rawTemp = analogRead(LM35_PIN);
  int tempC = (rawTemp * 500L) / 1024;
  tempBuffer[tempIndex] = tempC;
  tempIndex = (tempIndex + 1) % TEMP_SAMPLES;
  if (tempIndex == 0) tempBufferFilled = true;
  int samples = tempBufferFilled ? TEMP_SAMPLES : tempIndex;
  if (samples == 0) return tempC;
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += tempBuffer[i];
  }
  return sum / samples;
}

void draw_oled(int msg) {
  oled.firstPage();
  do {
    switch (msg) {
      case 0:
        oled.drawStr(16, 24, F("DEVICE ERROR"), 2);
        break;
      case 1:
        oled.drawStr(28, 20, F("PLACE"), 2);
        oled.drawStr(22, 40, F("FINGER"), 2);
        break;
      case 2:
        oled.drawXBMP(0, 0, 16, 16, heart_bits);
        print_digit(18, 0, beatAvg, ' ', 3, 2);
        oled.drawStr(56, 0, F("O2"), 1);
        print_digit(72, 0, SPO2, ' ', 3, 2);
        oled.drawChar(108, 0, '%', 2);
        oled.drawStr(56, 10, F("T"), 1);
        print_digit(64, 10, temperature, ' ', 2, 1);
        oled.drawChar(76, 10, 'C');
        // Alert indicator
        if (bpmAlertActive || tempAlertActive) {
          oled.drawStr(100, 10, F("!"), 1);
        }
        for (int i = 0; i < 128; i += 2) {
          oled.drawPixel(i, 13);
        }
        wave.draw(0);
        break;
      case 3:
        oled.drawStr(20, 20, F("PULSE OX"), 2);
        oled.drawStr(28, 42, F("MONITOR"), 2);
        break;
    }
  } while (oled.nextPage());
}

// ============ SETUP ============
void setup() {
  pinMode(LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  Serial.begin(9600);
  delay(1000);

  Serial.println(F("Pulse Ox + LoRa TX"));
  Serial.println(F("------------------"));

  // Initialize OLED
  oled.init();
  oled.fill(0x00);
  draw_oled(3);

  // Initialize LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println(F("LoRa init failed!"));
    oled.fill(0x00);
    draw_oled(0);
    while (1);
  }

  // Configure LoRa - must match receiver
  LoRa.setTxPower(20);
  LoRa.setSpreadingFactor(10);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);
  LoRa.enableCrc();
  LoRa.setSyncWord(0x12);

  Serial.println(F("LoRa OK at 433 MHz"));

  delay(2000);

  // Initialize MAX30102
  if (!sensor.begin()) {
    Serial.println(F("MAX30102 failed!"));
    draw_oled(0);
    while (1);
  }
  sensor.setup();
  Serial.println(F("MAX30102 OK"));

  // Setup schedulers
  displayScheduler.setHighPriorityScheduler(&sensorScheduler);
  displayScheduler.enableAll(true);
  displayScheduler.startNow(true);

  Serial.println(F("System ready"));
  Serial.println();
}

// ============ MAIN LOOP ============
void loop() {
  displayScheduler.execute();
}

// ============ SENSOR TASK (HIGH PRIORITY) ============
void sensorCallback() {
  sensor.check();
  if (!sensor.available()) return;

  uint32_t irValue = sensor.getIR();
  uint32_t redValue = sensor.getRed();
  sensor.nextSample();

  fingerPresent = (irValue >= FINGER_THRESHOLD);
  if (!fingerPresent) return;

  int16_t IR_signal, Red_signal;
  bool beatRed, beatIR;

  if (!filter_for_graph) {
    IR_signal = pulseIR.dc_filter(irValue);
    Red_signal = pulseRed.dc_filter(redValue);
    beatRed = pulseRed.isBeat(pulseRed.ma_filter(Red_signal));
    beatIR = pulseIR.isBeat(pulseIR.ma_filter(IR_signal));
  } else {
    IR_signal = pulseIR.ma_filter(pulseIR.dc_filter(irValue));
    Red_signal = pulseRed.ma_filter(pulseRed.dc_filter(redValue));
    beatRed = pulseRed.isBeat(Red_signal);
    beatIR = pulseIR.isBeat(IR_signal);
  }

  bool beatDetected = draw_Red ? beatRed : beatIR;
  wave.record(beatDetected);

  if (beatDetected) {
    unsigned long now = millis();
    unsigned long timeSinceLastBeat = now - lastBeatTime;

    if (lastBeatTime > 0 && timeSinceLastBeat > 0) {
      long btpm = 60000 / timeSinceLastBeat;
      if (btpm > 0 && btpm < MAX_BPM) {
        beatAvg = bpm.filter((int16_t)btpm);
      }
    }
    lastBeatTime = now;

    digitalWrite(LED, HIGH);
    digitalWrite(BUZZER, HIGH);
    taskBuzzerOff.restartDelayed(BEAT_FLASH_DURATION * 1000);

    // Compute SpO2
    long redAC = pulseRed.avgAC();
    long redDC = pulseRed.avgDC();
    long irAC = pulseIR.avgAC();
    long irDC = pulseIR.avgDC();

    if (redDC > 0 && irAC > 0 && irDC > 0 && redAC >= 0) {
      long R_x100 = ((redAC * 100) / redDC) * irDC / irAC;
      int spo2_calc = 104 - (17 * R_x100) / 100;
      if (spo2_calc > 100) spo2_calc = 100;
      if (spo2_calc < 0) spo2_calc = 0;
      SPO2 = spo2_calc;
    }
  }
}

// ============ DISPLAY TASK ============
void displayCallback() {
  int displayMode = fingerPresent ? 2 : 1;

  oled.firstPage();
  do {
    switch (displayMode) {
      case 1:
        oled.drawStr(28, 20, F("PLACE"), 2);
        oled.drawStr(22, 40, F("FINGER"), 2);
        break;
      case 2:
        oled.drawXBMP(0, 0, 16, 16, heart_bits);
        print_digit(18, 0, beatAvg, ' ', 3, 2);
        oled.drawStr(56, 0, F("O2"), 1);
        print_digit(72, 0, SPO2, ' ', 3, 2);
        oled.drawChar(108, 0, '%', 2);
        oled.drawStr(56, 10, F("T"), 1);
        print_digit(64, 10, temperature, ' ', 2, 1);
        oled.drawChar(76, 10, 'C');
        if (bpmAlertActive || tempAlertActive) {
          oled.drawStr(100, 10, F("!"), 1);
        }
        for (int i = 0; i < 128; i += 2) {
          oled.drawPixel(i, 13);
        }
        wave.draw(0);
        break;
    }
    sensorScheduler.execute();
  } while (oled.nextPage());
}

// ============ TEMPERATURE TASK ============
void tempCallback() {
  temperature = readTemperatureFiltered();
}

// ============ BUZZER OFF TASK ============
void buzzerOffCallback() {
  digitalWrite(LED, LOW);
  digitalWrite(BUZZER, LOW);
}

// ============ ALERT CHECK TASK (NON-BLOCKING) ============
void alertCheckCallback() {
  // Once alert is sent, LATCH it - do not auto-clear
  // User must manually reset the device to clear the alert

  if (alertSent) {
    // Alert already sent and latched, do nothing
    return;
  }

  unsigned long now = millis();

  // Check BPM threshold
  if (beatAvg >= ALERT_BPM_THRESHOLD && fingerPresent) {
    if (!bpmAlertActive) {
      bpmAlertStartTime = now;
      bpmAlertActive = true;
    }
  } else {
    bpmAlertActive = false;
    bpmAlertStartTime = 0;
  }

  // Check temperature threshold
  if (temperature >= ALERT_TEMP_THRESHOLD) {
    if (!tempAlertActive) {
      tempAlertStartTime = now;
      tempAlertActive = true;
    }
  } else {
    tempAlertActive = false;
    tempAlertStartTime = 0;
  }

  // Check if either condition has been sustained for required duration
  bool bpmSustained = bpmAlertActive && (now - bpmAlertStartTime >= ALERT_DURATION_MS);
  bool tempSustained = tempAlertActive && (now - tempAlertStartTime >= ALERT_DURATION_MS);

  if (bpmSustained || tempSustained) {
    // Send alert via LoRa and LATCH
    sendAlert(bpmSustained, tempSustained);
    alertSent = true;

    Serial.println(F("ALERT LATCHED - Reset device to clear"));
    // Alert stays active until device reset
  }
}

// ============ SEND ALERT (NON-BLOCKING) ============
void sendAlert(bool bpmAlert, bool tempAlert) {
  Serial.print(F("ALERT! Sending: "));

  LoRa.beginPacket();
  LoRa.print(F("ALERT:"));
  if (bpmAlert) {
    LoRa.print(F("BPM="));
    LoRa.print(beatAvg);
    Serial.print(F("BPM="));
    Serial.print(beatAvg);
  }
  if (bpmAlert && tempAlert) {
    LoRa.print(F(","));
    Serial.print(F(","));
  }
  if (tempAlert) {
    LoRa.print(F("TEMP="));
    LoRa.print(temperature);
    Serial.print(F("TEMP="));
    Serial.print(temperature);
  }
  LoRa.endPacket();  // Non-async for reliability

  Serial.println(F(" ... sent!"));
}
