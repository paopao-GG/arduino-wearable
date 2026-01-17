#include "ssd1306h.h"
#include "MAX30102.h"
#include "Pulse.h"
#include <avr/pgmspace.h>

// Routines to clear and set bits 
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


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
#define DISPLAY_INTERVAL 50     // Display update interval (ms)
#define BEAT_FLASH_DURATION 30  // LED/buzzer flash duration (ms)
#define TEMP_READ_INTERVAL 1000 // Temperature read interval (ms)

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
    do { 
        char ch = (val!=0) ? val%10+'0': c;
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

int  beatAvg;
int  SPO2;
int  temperature;  // Temperature in Celsius (integer)
const bool filter_for_graph = false;  // Set true to filter before graphing
const bool draw_Red = false;          // Set true to display Red signal instead of IR
bool buzzer_on = false;

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
}

long lastBeat = 0;    //Time of the last beat
long displaytime = 0; //Time of the last display update
long tempReadTime = 0; //Time of the last temperature update
bool led_on = false;


void loop()  {
    sensor.check();
    long now = millis();   //start time of this cycle
    if (!sensor.available()) return;
    uint32_t irValue = sensor.getIR(); 
    uint32_t redValue = sensor.getRed();
    sensor.nextSample();
    if (irValue < FINGER_THRESHOLD) {
        draw_oled(1);  // finger not down message
        delay(200);
    } else {
        // remove DC element
        int16_t IR_signal, Red_signal;
        bool beatRed, beatIR;
        if (!filter_for_graph) {
           IR_signal =  pulseIR.dc_filter(irValue) ;
           Red_signal = pulseRed.dc_filter(redValue);
           beatRed = pulseRed.isBeat(pulseRed.ma_filter(Red_signal));
           beatIR =  pulseIR.isBeat(pulseIR.ma_filter(IR_signal));        
        } else {
           IR_signal =  pulseIR.ma_filter(pulseIR.dc_filter(irValue)) ;
           Red_signal = pulseRed.ma_filter(pulseRed.dc_filter(redValue));
           beatRed = pulseRed.isBeat(Red_signal);
           beatIR =  pulseIR.isBeat(IR_signal);
        }
        // Check for heartbeat
        bool beatDetected = draw_Red ? beatRed : beatIR;

        // Record ECG-style waveform (triggers PQRST pattern on beat detection)
        wave.record(beatDetected);

        if (beatDetected) {
            long btpm = 60000/(now - lastBeat);
            if (btpm > 0 && btpm < MAX_BPM) beatAvg = bpm.filter((int16_t)btpm);
            lastBeat = now;
            digitalWrite(LED, HIGH);
            digitalWrite(BUZZER, HIGH);  // Active buzzer ON
            buzzer_on = true;
            led_on = true;
            // Compute SpO2 using R ratio: R = (RedAC/RedDC) / (IRAC/IRDC)
            // Rearranged to avoid floating point: R*100 = (RedAC * IRDC * 100) / (RedDC * IRAC)
            long redAC = pulseRed.avgAC();
            long redDC = pulseRed.avgDC();
            long irAC = pulseIR.avgAC();
            long irDC = pulseIR.avgDC();

            // Validate all values to prevent division by zero
            if (redDC > 0 && irAC > 0 && irDC > 0 && redAC >= 0) {
              // Calculate R * 100 to preserve precision
              // Reordered to prevent overflow: (redAC * 100 / redDC) * irDC / irAC
              long R_x100 = ((redAC * 100) / redDC) * irDC / irAC;
              // SpO2 = 104 - 17 * R  (standard linear approximation)
              // Clamp to valid range 0-100
              int spo2_calc = 104 - (17 * R_x100) / 100;
              if (spo2_calc > 100) spo2_calc = 100;
              if (spo2_calc < 0) spo2_calc = 0;
              SPO2 = spo2_calc;
            }
        }
        // update temperature periodically
        if (now - tempReadTime > TEMP_READ_INTERVAL) {
            tempReadTime = now;
            temperature = readTemperatureFiltered();
        }
        // update display periodically if finger down
        if (now - displaytime > DISPLAY_INTERVAL) {
            displaytime = now;
            draw_oled(2);
        }
    }
    // flash led and buzzer briefly
    if (led_on && (now - lastBeat) > BEAT_FLASH_DURATION) {
        digitalWrite(LED, LOW);
        led_on = false;
    }
    if (buzzer_on && (now - lastBeat) > BEAT_FLASH_DURATION) {
        digitalWrite(BUZZER, LOW);
        buzzer_on = false;
    }
}