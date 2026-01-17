#include "ssd1306h.h"
#include "MAX30102.h"
#include "Pulse.h"
#include <avr/pgmspace.h>

// TaskScheduler configuration - MUST be before include
#define _TASK_MICRO_RES          // Microsecond resolution for precise timing
#define _TASK_PRIORITY           // Enable layered priority scheduling
#include <TaskScheduler.h>

SSD1306 oled;
MAX30102 sensor;
Pulse pulseIR;
Pulse pulseRed;
MAFilter bpm;

#define LED LED_BUILTIN
#define BUZZER 3  // Buzzer pin (use a PWM pin for tone)
#define LM35_PIN A0  // LM35 temperature sensor analog pin

// Threshold and timing constants
#define FINGER_THRESHOLD 5000   // IR value below this = no finger detected
#define MAX_BPM 200             // Maximum valid heart rate
#define SENSOR_INTERVAL 4000    // Sensor read interval (4ms = 250Hz, matches MAX30102 sample rate)
#define DISPLAY_INTERVAL 50000  // Display update interval (50ms in microseconds)
#define BEAT_FLASH_DURATION 30  // LED/buzzer flash duration (ms)
#define TEMP_READ_INTERVAL 1000000 // Temperature read interval (1s in microseconds)

static const uint8_t heart_bits[] PROGMEM = { 0x00, 0x00, 0x38, 0x38, 0x7c, 0x7c, 0xfe, 0xfe, 0xfe, 0xff, 
                                        0xfe, 0xff, 0xfc, 0x7f, 0xf8, 0x3f, 0xf0, 0x1f, 0xe0, 0x0f,
                                        0xc0, 0x07, 0x80, 0x03, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 
                                        0x00, 0x00 };

// SpO2 calculation uses the standard formula: SpO2 = 104 - 17 * R
// where R = (RedAC/RedDC) / (IRAC/IRDC)
// This is the Beer-Lambert law approximation used in pulse oximetry


void print_digit(int x, int y, long val, char c=' ', uint8_t field = 3,const int BIG = 2)
    {
    uint8_t ff = field;
    bool first = true;  // Track if we're on the rightmost digit
    do {
        char ch;
        if (first || val != 0) {
            ch = val % 10 + '0';  // Always show at least the last digit
            first = false;
        } else {
            ch = c;  // Padding character for leading positions
        }
        oled.drawChar( x+BIG*(ff-1)*6, y, ch, BIG);
        val = val/10;
        --ff;
    } while (ff>0);
}


/*
 *   ECG-Style Waveform Display
 *   Converts heartbeat detection into PQRST-like ECG pattern
 *   Graph area: rows 16-63, baseline at row 52
 */
const uint8_t MAXWAVE = 128;      // Full display width
const uint8_t WAVE_BASELINE = 52; // Fixed baseline row
const uint8_t WAVE_TOP = 20;      // Top boundary of waveform area
const uint8_t WAVE_BOTTOM = 60;   // Bottom boundary of waveform area

// ECG PQRST pattern template (13 samples)
// Values are offsets from baseline (positive = up, negative = down)
const int8_t ECG_PATTERN[] PROGMEM = {
  0,    // baseline
  2,    // P wave start
  4,    // P wave peak
  2,    // P wave end
  0,    // PR segment
  -3,   // Q wave (small dip)
  20,   // R wave peak (tall spike up)
  -8,   // S wave (dip below baseline)
  0,    // ST segment
  3,    // T wave start
  8,    // T wave peak
  4,    // T wave end
  0     // back to baseline
};
const uint8_t ECG_PATTERN_LEN = 13;

class Waveform {
  public:
    Waveform(void) {
      wavep = 0;
      ecgIndex = 0;
      inBeat = false;
      // Initialize all to baseline (0 = flat line)
      for (int i = 0; i < MAXWAVE; i++) {
        waveform[i] = 0;
      }
    }

    // Call this every sample - pass true when a beat is detected
    void record(bool beatDetected) {
      // If beat detected and not already drawing a beat, start ECG pattern
      if (beatDetected && !inBeat) {
        inBeat = true;
        ecgIndex = 0;
      }

      // If currently drawing ECG pattern
      if (inBeat) {
        waveform[wavep] = pgm_read_byte(&ECG_PATTERN[ecgIndex]);
        ecgIndex++;
        if (ecgIndex >= ECG_PATTERN_LEN) {
          inBeat = false;  // Pattern complete
          ecgIndex = 0;
        }
      } else {
        // Flat baseline when no beat
        waveform[wavep] = 0;
      }

      wavep = (wavep + 1) % MAXWAVE;
    }

    void draw(uint8_t X) {
      // Draw subtle vertical grid lines (hospital monitor style)
      for (int i = 32; i < MAXWAVE; i += 32) {
        for (uint8_t y = WAVE_TOP; y <= WAVE_BOTTOM; y += 6) {
          oled.drawPixel(X + i, y);
        }
      }

      // Draw waveform - ECG style with connected lines
      // Oldest data on left (x=0), newest on right (x=127)
      uint8_t index = wavep;  // Start from oldest
      for (int i = 0; i < MAXWAVE - 1; i++) {
        // Calculate Y positions: baseline minus offset (positive offset = up)
        int y1 = WAVE_BASELINE - waveform[index];
        index = (index + 1) % MAXWAVE;
        int y2 = WAVE_BASELINE - waveform[index];

        // Clamp to display bounds
        if (y1 < WAVE_TOP) y1 = WAVE_TOP;
        if (y1 > WAVE_BOTTOM) y1 = WAVE_BOTTOM;
        if (y2 < WAVE_TOP) y2 = WAVE_TOP;
        if (y2 > WAVE_BOTTOM) y2 = WAVE_BOTTOM;

        // Draw the point
        oled.drawPixel(X + i, y1);

        // Connect to next point with vertical line for sharp ECG transitions
        if (y2 > y1) {
          oled.drawVLine(X + i, y1, y2 - y1 + 1);
        } else if (y1 > y2) {
          oled.drawVLine(X + i, y2, y1 - y2 + 1);
        }
      }
      // Draw last point
      int yLast = WAVE_BASELINE - waveform[(wavep + MAXWAVE - 1) % MAXWAVE];
      if (yLast < WAVE_TOP) yLast = WAVE_TOP;
      if (yLast > WAVE_BOTTOM) yLast = WAVE_BOTTOM;
      oled.drawPixel(X + MAXWAVE - 1, yLast);
    }

private:
    int8_t waveform[MAXWAVE];  // Signed: offset from baseline (+ = up, - = down)
    uint8_t wavep = 0;
    uint8_t ecgIndex = 0;      // Current position in ECG pattern
    bool inBeat = false;       // Currently drawing a beat pattern

} wave;

int  beatAvg = 0;
int  SPO2 = 99;        // Default to 99% until first reading
int  temperature = 0;  // Temperature in Celsius (integer)
const bool filter_for_graph = false;  // Set true to filter before graphing
const bool draw_Red = false;          // Set true to display Red signal instead of IR

// Beat detection state (shared between tasks)
volatile bool fingerPresent = false;
volatile unsigned long lastBeatTime = 0;

// Priority-based TaskSchedulers (highest priority runs first on every pass)
// Sensor scheduler runs on EVERY pass - never misses a beat
Scheduler sensorScheduler;   // HIGHEST priority - sensor + buzzer
Scheduler displayScheduler;  // LOW priority - display + temp (can be delayed)

// Forward declarations for task callbacks
void sensorCallback();
void displayCallback();
void tempCallback();
void buzzerOffCallback();

// HIGHEST PRIORITY: Sensor task (runs every 4ms = 250Hz)
// This task runs on every scheduler pass, even during display updates
Task taskSensor(SENSOR_INTERVAL, TASK_FOREVER, &sensorCallback, &sensorScheduler, true);

// HIGHEST PRIORITY: Buzzer off task (one-shot, triggered by beat)
Task taskBuzzerOff(BEAT_FLASH_DURATION * 1000, TASK_ONCE, &buzzerOffCallback, &sensorScheduler, false);

// LOW PRIORITY: Display update (every 50ms = 20Hz)
// Display is slow but sensor still runs during display update
Task taskDisplay(DISPLAY_INTERVAL, TASK_FOREVER, &displayCallback, &displayScheduler, true);

// LOW PRIORITY: Temperature reading (every 1 second)
Task taskTemp(TEMP_READ_INTERVAL, TASK_FOREVER, &tempCallback, &displayScheduler, true);

// Temperature smoothing filter (8-sample moving average)
#define TEMP_SAMPLES 8
int tempBuffer[TEMP_SAMPLES];
uint8_t tempIndex = 0;
bool tempBufferFilled = false;

int readTemperatureFiltered() {
    // Read LM35: 10mV per degree C
    int rawTemp = analogRead(LM35_PIN);
    int tempC = (rawTemp * 500L) / 1024;

    // Store in circular buffer
    tempBuffer[tempIndex] = tempC;
    tempIndex = (tempIndex + 1) % TEMP_SAMPLES;
    if (tempIndex == 0) tempBufferFilled = true;

    // Calculate average
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
    do{
    switch(msg){
        case 0:  // Device error - centered on 128x64
                 oled.drawStr(16,24,F("DEVICE ERROR"),2);
                 break;
        case 1:  // Place finger - centered on 128x64
                 oled.drawStr(28,20,F("PLACE"),2);
                 oled.drawStr(22,40,F("FINGER"),2);
                 break;
        case 2:  // Main display - data bar on top, waveform below
                 // Top row: Heart + BPM | O2 + SpO2% | Temp
                 oled.drawXBMP(0,0,16,16,heart_bits);   // Heart icon at left
                 print_digit(18,0,beatAvg,' ',3,2);     // BPM value (large)

                 oled.drawStr(56,0,F("O2"),1);          // SpO2 label
                 print_digit(72,0,SPO2,' ',3,2);        // SpO2 value (large)
                 oled.drawChar(108,0,'%',2);            // Percent sign

                 oled.drawStr(56,10,F("T"),1);          // Temp label (below O2)
                 print_digit(64,10,temperature,' ',2,1); // Temp value
                 oled.drawChar(76,10,'C');              // Celsius

                 // Separator line between data and waveform
                 for (int i=0; i<128; i+=2) {
                   oled.drawPixel(i, 13);
                 }

                 // Waveform fills bottom area (rows 16-63, baseline at 50)
                 wave.draw(0);
                 break;
        case 3:  // Startup message - centered on 128x64
                 oled.drawStr(20,20,F("PULSE OX"),2);
                 oled.drawStr(28,42,F("MONITOR"),2);
                 break;
        }
    } while (oled.nextPage());
}

void setup(void) {
  pinMode(LED, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  oled.init();
  oled.fill(0x00);
  draw_oled(3);
  delay(3000);
  if (!sensor.begin())  {
    draw_oled(0);
    while (1);
  }
  sensor.setup();

  // Link priority schedulers: displayScheduler -> sensorScheduler (highest)
  // sensorScheduler runs on EVERY pass of displayScheduler
  displayScheduler.setHighPriorityScheduler(&sensorScheduler);

  // Enable all tasks in both schedulers
  displayScheduler.enableAll(true);  // true = recursive, enables sensorScheduler tasks too

  // Sync timing
  displayScheduler.startNow(true);   // true = recursive
}

void loop()  {
    // Execute the base scheduler - it will automatically run high priority scheduler first
    displayScheduler.execute();
}

/*
 * SENSOR TASK - High priority, runs every 4ms (250Hz)
 * Reads MAX30102, detects beats, triggers immediate buzzer/LED response
 */
void sensorCallback() {
    sensor.check();
    if (!sensor.available()) return;

    uint32_t irValue = sensor.getIR();
    uint32_t redValue = sensor.getRed();
    sensor.nextSample();

    // Check finger presence
    fingerPresent = (irValue >= FINGER_THRESHOLD);
    if (!fingerPresent) return;

    // Signal processing
    int16_t IR_signal, Red_signal;
    bool beatRed, beatIR;

    if (!filter_for_graph) {
       IR_signal =  pulseIR.dc_filter(irValue);
       Red_signal = pulseRed.dc_filter(redValue);
       beatRed = pulseRed.isBeat(pulseRed.ma_filter(Red_signal));
       beatIR =  pulseIR.isBeat(pulseIR.ma_filter(IR_signal));
    } else {
       IR_signal =  pulseIR.ma_filter(pulseIR.dc_filter(irValue));
       Red_signal = pulseRed.ma_filter(pulseRed.dc_filter(redValue));
       beatRed = pulseRed.isBeat(Red_signal);
       beatIR =  pulseIR.isBeat(IR_signal);
    }

    // Check for heartbeat
    bool beatDetected = draw_Red ? beatRed : beatIR;

    // Record ECG-style waveform (triggers PQRST pattern on beat detection)
    wave.record(beatDetected);

    if (beatDetected) {
        unsigned long now = millis();
        unsigned long timeSinceLastBeat = now - lastBeatTime;

        // Avoid division by zero and ignore first beat (no valid interval yet)
        if (lastBeatTime > 0 && timeSinceLastBeat > 0) {
            long btpm = 60000 / timeSinceLastBeat;
            if (btpm > 0 && btpm < MAX_BPM) {
                beatAvg = bpm.filter((int16_t)btpm);
            }
        }
        lastBeatTime = now;

        // IMMEDIATE buzzer and LED response - no delay!
        digitalWrite(LED, HIGH);
        digitalWrite(BUZZER, HIGH);

        // Schedule buzzer off task (one-shot, will turn off after BEAT_FLASH_DURATION)
        taskBuzzerOff.restartDelayed(BEAT_FLASH_DURATION * 1000);  // microseconds

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

/*
 * DISPLAY TASK - Low priority, runs every 50ms (20Hz)
 * Uses incremental page-by-page drawing with sensor yields between pages
 */
void displayCallback() {
    int displayMode = fingerPresent ? 2 : 1;

    oled.firstPage();
    do {
        // Draw current page content
        switch(displayMode) {
            case 1:  // Place finger message
                oled.drawStr(28,20,F("PLACE"),2);
                oled.drawStr(22,40,F("FINGER"),2);
                break;
            case 2:  // Main display with waveform
                oled.drawXBMP(0,0,16,16,heart_bits);
                print_digit(18,0,beatAvg,' ',3,2);
                oled.drawStr(56,0,F("O2"),1);
                print_digit(72,0,SPO2,' ',3,2);
                oled.drawChar(108,0,'%',2);
                oled.drawStr(56,10,F("T"),1);
                print_digit(64,10,temperature,' ',2,1);
                oled.drawChar(76,10,'C');
                for (int i=0; i<128; i+=2) {
                    oled.drawPixel(i, 13);
                }
                wave.draw(0);
                break;
        }

        // CRITICAL: Run sensor scheduler between each OLED page write
        // This ensures we don't miss any beats during slow I2C transfers
        sensorScheduler.execute();

    } while (oled.nextPage());
}

/*
 * TEMPERATURE TASK - Low priority, runs every 1 second
 * Reads LM35 temperature sensor
 */
void tempCallback() {
    temperature = readTemperatureFiltered();
}

/*
 * BUZZER OFF TASK - One-shot, triggered by beat detection
 * Turns off LED and buzzer after BEAT_FLASH_DURATION
 */
void buzzerOffCallback() {
    digitalWrite(LED, LOW);
    digitalWrite(BUZZER, LOW);
}