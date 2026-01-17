# Pulse Oximeter Project - Memory Bank

## Project Overview
A wearable pulse oximeter device using Arduino that measures:
- **Heart Rate (BPM)** - via MAX30102 PPG sensor
- **Blood Oxygen (SpO2%)** - calculated using Beer-Lambert law approximation
- **Temperature** - via LM35 analog sensor

Display: SSD1306 128x64 OLED with ECG-style waveform visualization

## Hardware Components
| Component | Pin | Description |
|-----------|-----|-------------|
| MAX30102 | I2C (A4/A5) | Pulse oximeter sensor (PPG, not ECG) |
| SSD1306 OLED | I2C | 128x64 pixel display |
| LM35 | A0 | Temperature sensor (10mV/°C) |
| Buzzer | D3 | Active buzzer for heartbeat feedback |
| LED | LED_BUILTIN | Visual heartbeat indicator |

## Current Code Status

### Main File
`test_pulse_oled_buzzerr.ino`

### Key Constants
```cpp
#define FINGER_THRESHOLD 5000   // IR value below = no finger
#define MAX_BPM 200             // Maximum valid heart rate
#define DISPLAY_INTERVAL 50     // Display update (ms)
#define BEAT_FLASH_DURATION 30  // LED/buzzer flash (ms)
#define TEMP_READ_INTERVAL 1000 // Temperature read (ms)
```

### Display Layout (128x64 OLED)
```
Row 0-12:  Data bar (BPM, SpO2%, Temperature)
Row 13:    Dotted separator line
Row 14-63: ECG-style waveform (baseline at row 52)
```

### ECG-Style Waveform Implementation
The waveform converts PPG beat detection into an ECG-like PQRST pattern:

```cpp
const int8_t ECG_PATTERN[] PROGMEM = {
  0,    // baseline
  2,    // P wave start
  4,    // P wave peak (4px up)
  2,    // P wave end
  0,    // PR segment
  -3,   // Q wave (3px down)
  20,   // R wave peak (20px up - tall spike)
  -8,   // S wave (8px down)
  0,    // ST segment
  3,    // T wave start
  8,    // T wave peak (8px up)
  4,    // T wave end
  0     // back to baseline
};
```

- **Baseline**: Row 52 (fixed, no auto-scaling)
- **Waveform area**: Rows 20-60
- **Display width**: 128 pixels (full width)
- **Scrolling**: Right-to-left (newest data on right)

### SpO2 Calculation
Using Beer-Lambert law approximation:
```cpp
// R = (RedAC/RedDC) / (IRAC/IRDC)
// SpO2 = 104 - 17 * R
// Reordered to prevent overflow:
long R_x100 = ((redAC * 100) / redDC) * irDC / irAC;
int spo2_calc = 104 - (17 * R_x100) / 100;
```

### Temperature Reading
- 8-sample moving average filter
- LM35 formula: `tempC = (rawADC * 500) / 1024`

## Libraries Required
Located at: `C:\Users\satui\OneDrive\Documents\Arduino\libraries`

| Library | Description |
|---------|-------------|
| ssd1306h | OLED display driver |
| MAX30102 | Pulse oximeter sensor driver |
| Pulse | Signal processing (DC filter, MA filter, beat detection) |
| TaskScheduler | Cooperative multitasking for timing synchronization |

## Important Technical Notes

### MAX30102 is PPG, NOT ECG
- **PPG (Photoplethysmography)**: Optical measurement of blood volume changes
- **ECG (Electrocardiogram)**: Electrical measurement of heart activity
- The display shows a *styled* ECG-like pattern triggered by PPG beat detection
- This is for visual aesthetics only - not actual ECG data

### For Real ECG
Would need different sensor:
- AD8232 ECG module
- MAX86150 (combined PPG + ECG)
- ADS1292R

## TaskScheduler Optimization (Latest Update)

### Problem Solved
- Graph was laggy/delayed
- Buzzer was not synced with heartbeat
- Display updates were blocking sensor reading

### Solution: Cooperative Multitasking with TaskScheduler

Using `TaskScheduler` library with microsecond resolution (`_TASK_MICRO_RES`):

```cpp
#define _TASK_MICRO_RES  // Enable microsecond timing
#include <TaskScheduler.h>

Scheduler runner;

// Task definitions (interval in microseconds)
Task taskSensor(4000, TASK_FOREVER, &sensorCallback);      // 4ms = 250Hz
Task taskDisplay(50000, TASK_FOREVER, &displayCallback);   // 50ms = 20Hz
Task taskTemp(1000000, TASK_FOREVER, &tempCallback);       // 1 second
Task taskBuzzerOff(30000, TASK_ONCE, &buzzerOffCallback);  // 30ms one-shot
```

### Task Architecture

| Task | Interval | Priority | Purpose |
|------|----------|----------|---------|
| `taskSensor` | 4ms (250Hz) | High | Read MAX30102, detect beats, trigger buzzer |
| `taskDisplay` | 50ms (20Hz) | Medium | Update OLED display |
| `taskTemp` | 1000ms (1Hz) | Low | Read LM35 temperature |
| `taskBuzzerOff` | 30ms (one-shot) | - | Turn off LED/buzzer after beat |

### Key Improvements

1. **Immediate Buzzer Response**: When beat detected, buzzer turns on IMMEDIATELY in sensor callback, not waiting for next loop iteration

2. **Non-blocking Display**: Display updates happen independently of sensor reading - no more 50ms blocking delays

3. **Precise Timing**: Microsecond resolution ensures sensor reads at exact 250Hz to match MAX30102 sample rate

4. **One-shot Buzzer Off**: Instead of polling `millis()` every loop, a one-shot task handles buzzer timeout

### Loop Structure Change

**Before (blocking):**
```cpp
void loop() {
    readSensor();      // may block
    if (time_check) updateDisplay();  // blocks ~50ms
    if (time_check) buzzerOff();      // polling
}
```

**After (cooperative):**
```cpp
void loop() {
    runner.execute();  // TaskScheduler handles everything
}
```

## Fixes Applied During Development

1. **Integer overflow in SpO2 calculation** - Reordered operations
2. **Division by zero risk** - Added validation checks
3. **Display optimization** - Expanded from 32px to full 64px height
4. **Magic numbers** - Replaced with named constants
5. **Waveform style** - Converted raw PPG to ECG-style PQRST pattern
6. **Timing synchronization** - TaskScheduler for non-blocking cooperative multitasking

## Display Screens

| Screen | Trigger | Content |
|--------|---------|---------|
| 0 | Sensor init fail | "DEVICE ERROR" |
| 1 | No finger | "PLACE FINGER" |
| 2 | Normal operation | BPM, SpO2%, Temp + waveform |
| 3 | Startup | "PULSE OX MONITOR" |

## Potential Future Improvements
- [ ] Add low SpO2 alarm (< 90%)
- [ ] Add abnormal heart rate alerts
- [ ] Battery level indicator
- [ ] Data logging to SD card
- [ ] Bluetooth connectivity
- [ ] Adjust ECG pattern heights for visual preference

---
*Last updated: 2026-01-17*
