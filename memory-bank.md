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
- [x] Add abnormal heart rate alerts - ✓ Implemented via LoRa
- [ ] Add low SpO2 alarm (< 90%)
- [ ] Battery level indicator
- [ ] Data logging to SD card
- [ ] Bluetooth connectivity
- [ ] Adjust ECG pattern heights for visual preference

---

# LoRa Alert System - Extended Project

## Project Overview
Extended the pulse oximeter with LoRa wireless alert transmission system.

**Architecture:**
- **Transmitter**: Arduino Nano + MAX30102 + OLED + LoRa RA-02
- **Receiver**: Arduino UNO + LoRa RA-02 + Alert LED

## Hardware Components - LoRa System

### Transmitter (Arduino Nano)
| Component | Pin | Description |
|-----------|-----|-------------|
| LoRa RA-02 VCC | External 3.3V | **CRITICAL**: Nano's 3.3V cannot supply ~120mA TX current |
| LoRa NSS | D10 | SPI Chip Select |
| LoRa MOSI/MISO/SCK | D11/D12/D13 | Hardware SPI (fixed pins) |
| LoRa RST | D9 | Reset |
| LoRa DIO0 | D2 | Interrupt |
| MAX30102 + OLED | I2C (A4/A5) | Shared I2C bus |
| LM35 | A0 | Temperature sensor |
| Buzzer | D3 | Heartbeat feedback |

### Receiver (Arduino UNO)
| Component | Pin | Description |
|-----------|-----|-------------|
| LoRa RA-02 VCC | 3.3V | UNO's 3.3V is sufficient for RX-only (~50mA) |
| LoRa NSS | D10 | SPI Chip Select |
| LoRa MOSI/MISO/SCK | D11/D12/D13 | Hardware SPI (fixed pins) |
| LoRa RST | D9 | Reset |
| LoRa DIO0 | D2 | Interrupt |
| Alert LED | D8 | Alert indicator (220-330Ω resistor) |

## LoRa Configuration (433 MHz)
Both transmitter and receiver use matching settings:

```cpp
LoRa.setTxPower(20);             // 20 dBm max power
LoRa.setSpreadingFactor(10);     // SF10 - better noise immunity
LoRa.setSignalBandwidth(125E3);  // 125 kHz
LoRa.setCodingRate4(8);          // 4/8 - max error correction
LoRa.enableCrc();                // Enable CRC checking
LoRa.setSyncWord(0x12);          // Private sync word (filter noise)
```

**Why SF10?** Better noise immunity than SF7, suitable for medical alerts where reliability > speed.

## Alert Logic (Transmitter)

### Alert Thresholds
```cpp
#define ALERT_BPM_THRESHOLD 100   // Testing value (production: 120)
#define ALERT_TEMP_THRESHOLD 40   // 40°C
#define ALERT_DURATION_MS 3000    // Testing value (production: 30000)
```

### State Machine
```
Normal vitals → No transmission
    ↓
BPM >= 100 for 3s → Send "ALERT:BPM=xxx" (ONE packet)
    ↓ or
Temp >= 40°C for 3s → Send "ALERT:TEMP=xx" (ONE packet)
    ↓ or both
BPM + Temp alert → Send "ALERT:BPM=xxx,TEMP=xx" (ONE packet)
    ↓
Alert stays active → No repeated sends (event-driven, not periodic)
    ↓
Both conditions normal → Send "CLEAR" (ONE packet)
```

### Non-Blocking Alert Check
Uses TaskScheduler with 500ms check interval:
```cpp
Task taskAlertCheck(500000, TASK_FOREVER, &alertCheckCallback, &displayScheduler, true);
```

**Key behavior:**
- `alertSent` flag prevents repeated transmissions
- Sustained threshold required (not instant trigger)
- CLEAR sent only when **both** BPM and temp return to normal
- Conservative design for medical safety

## Receiver Behavior

### LED States
| LED State | Trigger | Meaning |
|-----------|---------|---------|
| OFF | Normal | No alerts |
| ON (solid) | `ALERT:` received | High BPM and/or high temp |
| OFF | `CLEAR` received | All vitals normal |

### Message Format
Receiver parses three message types:
1. `ALERT:BPM=xxx` - High heart rate only
2. `ALERT:TEMP=xx` - High temperature only
3. `ALERT:BPM=xxx,TEMP=xx` - Both conditions
4. `CLEAR` - All clear

## File Locations

### Final Implementation
- **Transmitter**: `lora-tx-nano-final/lora-tx-nano-final.ino`
- **Receiver**: `lora-rx-uno-final/lora-rx-uno-final.ino`

### Test Files (basic LoRa communication)
- `lora-tx-nano/lora-tx-nano.ino` - Simple transmitter test
- `lora-rx-uno/lora-rx-uno.ino` - Simple receiver test

## TaskScheduler Architecture (Transmitter)

### Priority Schedulers
```cpp
Scheduler sensorScheduler;   // HIGH priority - critical timing
Scheduler displayScheduler;  // LOW priority - can be delayed

displayScheduler.setHighPriorityScheduler(&sensorScheduler);
```

**Why two schedulers?**
- Sensor runs every 4ms (250Hz) - MUST NOT be delayed
- Display updates are slow (~50ms) but sensor still runs during OLED I2C transfers
- This prevents missing heartbeats during display refresh

### Task Breakdown (Transmitter)
| Task | Interval | Scheduler | Purpose |
|------|----------|-----------|---------|
| `taskSensor` | 4ms | sensorScheduler | Read MAX30102, detect beats, compute BPM/SpO2 |
| `taskBuzzerOff` | 30ms (one-shot) | sensorScheduler | Turn off LED/buzzer |
| `taskDisplay` | 50ms | displayScheduler | Update OLED (yields to sensor between pages) |
| `taskTemp` | 1s | displayScheduler | Read LM35 temperature |
| `taskAlertCheck` | 500ms | displayScheduler | Check alert thresholds, send LoRa |

## Libraries Required (LoRa System)

### Transmitter
- `LoRa` by Sandeep Mistry (v0.8.0)
- `TaskScheduler`
- `ssd1306h`, `MAX30102`, `Pulse` (from original project)

### Receiver
- `LoRa` by Sandeep Mistry (v0.8.0)

## Common Pitfalls & Solutions

### ❌ Problem: Transmitter not sending
**Cause**: `while (!Serial);` blocks forever if Serial Monitor not open
**Fix**: Changed to `delay(1000);`

### ❌ Problem: Garbled data received
**Symptoms**: SNR = -16 dB (signal below noise), RSSI +27 dBm (impossible)
**Causes**: No antenna, modules too close, frequency mismatch
**Fix**:
- Added 17.3cm wire antenna (433MHz λ/4)
- Increased SF from 7 to 10
- Increased coding rate to 4/8
- Added sync word 0x12

### ❌ Problem: Nano 3.3V can't power LoRa
**Cause**: LoRa TX needs ~120mA peak, Nano's 3.3V regulator supplies ~50mA
**Fix**: Use external AMS1117-3.3V regulator

## BPM Detection Accuracy

The BPM calculation is standard but accuracy depends on:
1. **Finger placement stability** - movement causes false beats
2. **FINGER_THRESHOLD (5000)** - may need tuning
3. **Pulse library's beat detection algorithm** - uses MA filter

```cpp
// Beat-to-beat interval calculation
long btpm = 60000 / timeSinceLastBeat;
beatAvg = bpm.filter((int16_t)btpm);  // MAFilter smooths fluctuations
```

## GitHub Repository
Pushed to: https://github.com/paopao-GG/arduino-wearable.git

## Power Consumption Notes
- **Transmitter (event-driven)**:
  - Idle: Sensor (250Hz) + OLED refresh = ~30mA continuous
  - Alert TX: +120mA burst (~100ms) per transmission
  - Very low RF traffic (1-2 packets per alert event)

- **Receiver (listen-only)**:
  - LoRa RX mode: ~15mA continuous
  - No power-hungry peripherals

---
*Last updated: 2026-01-18*
