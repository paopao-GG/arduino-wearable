# Arduino 2 - Keypad OTP Relay Controller Implementation Plan

## Overview
Modify `uno-keypad-sms.ino` to add 3x4 keypad with PIN verification system featuring random OTP generation when alert is received.

## Requirements Analysis

### Hardware
- Arduino UNO 2 (existing)
- 5V Single Channel Relay Module (existing)
- 3x4 Matrix Keypad (NEW)
- Signal input from Arduino 1 on D8 (existing)

### Functional Requirements
1. **Normal Mode (No Alert)**: Default PIN = "1234" to trigger relay
2. **Alert Mode**: When signal received from Arduino 1:
   - Generate random 4-digit OTP
   - Display OTP on Serial Monitor
   - Require OTP entry to trigger relay
   - Latch relay ON once correct PIN entered
3. **Non-blocking operation using TaskScheduler**

## Pin Allocation

### Current Pins
- D7: Relay control (OUTPUT)
- D8: Signal from Arduino 1 (INPUT_PULLUP)

### New Pins for 3x4 Keypad
3x4 keypad requires 7 pins (4 rows + 3 columns):
- **Row Pins (4)**: D2, D3, D4, D5
- **Column Pins (3)**: D6, A0, A1

Keypad layout:
```
[1] [2] [3]
[4] [5] [6]
[7] [8] [9]
[*] [0] [#]
```

## Libraries Required

1. **Keypad Library** by Christopher Andrews (Chris--A)
   - Repository: https://github.com/Chris--A/Keypad
   - Supports 3x4 and 4x4 matrix keypads
   - Version 3.0+ supports multiple keypresses
   - Uses internal pullup resistors (no external resistors needed)

2. **TaskScheduler** by Anatoli Arkhipenko
   - Already in use in transmitter code
   - Cooperative multitasking for non-blocking operations
   - Repository: https://github.com/arkhipenko/TaskScheduler

## State Machine Design

### States
1. **IDLE**: Waiting for input or signal
2. **NORMAL_MODE**: No alert, waiting for default PIN "1234"
3. **ALERT_MODE**: Alert received, waiting for OTP
4. **PIN_ENTRY**: User entering PIN
5. **RELAY_ACTIVE**: Relay latched ON

### State Transitions
```
IDLE/NORMAL_MODE + No signal detected → NORMAL_MODE
NORMAL_MODE + Correct PIN (1234) → RELAY_ACTIVE

IDLE/NORMAL_MODE + Signal detected → ALERT_MODE (generate OTP)
ALERT_MODE + Correct OTP → RELAY_ACTIVE

RELAY_ACTIVE → stays latched (reset required to exit)
```

## Implementation Structure

### Global Variables
```cpp
// Keypad configuration
const byte ROWS = 4;
const byte COLS = 3;
char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
byte rowPins[ROWS] = {2, 3, 4, 5};
byte colPins[COLS] = {6, A0, A1};

// PIN management
String currentPIN = "";
String defaultPIN = "1234";
String generatedOTP = "";

// State tracking
enum SystemState {NORMAL_MODE, ALERT_MODE, RELAY_ACTIVE};
SystemState currentState = NORMAL_MODE;
bool alertSignalReceived = false;
```

### TaskScheduler Tasks
```cpp
Scheduler taskScheduler;

// Tasks
Task taskCheckSignal(50, TASK_FOREVER, &checkSignalCallback);     // Check D8 every 50ms
Task taskReadKeypad(100, TASK_FOREVER, &readKeypadCallback);      // Read keypad every 100ms
Task taskPINTimeout(30000, TASK_ONCE, &pinTimeoutCallback);       // 30s timeout for PIN entry
```

### Core Functions

#### 1. `setup()`
- Initialize pins (D7 relay, D8 input)
- Initialize Keypad object
- Initialize Serial (9600 baud)
- Setup TaskScheduler with tasks
- Display welcome message and instructions

#### 2. `checkSignalCallback()` (Task)
- Read D8 signal state
- If HIGH detected and not in ALERT_MODE:
  - Transition to ALERT_MODE
  - Generate 4-digit random OTP
  - Print OTP to Serial Monitor
  - Enable PIN timeout task
  - Clear any partial PIN entry

#### 3. `readKeypadCallback()` (Task)
- Check for keypad press using `keypad.getKey()`
- If key pressed:
  - If numeric (0-9): append to `currentPIN`
  - If '#': submit PIN for verification
  - If '*': clear current PIN entry
  - Display current entry state on Serial

#### 4. `generateOTP()`
- Use `random(0, 10000)` to generate 4-digit number
- Format as 4-digit string with leading zeros
- Store in `generatedOTP`
- Return OTP string

#### 5. `verifyPIN(String enteredPIN)`
- Compare entered PIN against:
  - `defaultPIN` if in NORMAL_MODE
  - `generatedOTP` if in ALERT_MODE
- If correct:
  - Activate relay (setRelayState(true))
  - Transition to RELAY_ACTIVE
  - Print success message
  - Disable PIN timeout task
- If incorrect:
  - Print error message
  - Clear `currentPIN`
  - Allow retry

#### 6. `pinTimeoutCallback()` (Task)
- Only runs in ALERT_MODE
- If timeout occurs without correct PIN:
  - Print timeout warning
  - Optionally: sound buzzer, send alert, etc.
  - Stay in ALERT_MODE (user must still enter correct OTP)

#### 7. `setRelayState(bool state)`
- Existing function (no changes needed)
- Accounts for ACTIVE_HIGH/ACTIVE_LOW relay configuration

## Serial Monitor Output Design

### On Startup
```
Arduino 2 - Keypad OTP Relay Controller
---------------------------------------
Keypad initialized (3x4 matrix)
Default PIN: 1234 (NORMAL MODE)

Waiting for alert signal or PIN entry...
```

### When Alert Signal Received
```
*** ALERT SIGNAL RECEIVED! ***
OTP GENERATED: 7392
Enter OTP to activate relay
Timeout: 30 seconds
********************************
```

### During PIN Entry
```
PIN Entry: 7___
PIN Entry: 73__
PIN Entry: 739_
PIN Entry: 7392
Verifying...
```

### On Correct PIN
```
✓ PIN CORRECT!
Relay: ON (LATCHED)
System locked until reset
```

### On Incorrect PIN
```
✗ INCORRECT PIN!
Try again...
```

### On Timeout
```
!!! PIN ENTRY TIMEOUT !!!
OTP still required: 7392
```

## Code Flow Diagram

```
setup()
  ├─ Initialize hardware
  ├─ Seed random number generator (randomSeed(analogRead(A5)))
  ├─ Start TaskScheduler
  └─ Enter loop()

loop()
  └─ taskScheduler.execute()
       ├─ taskCheckSignal.execute() every 50ms
       │    └─ If HIGH detected → generateOTP() → ALERT_MODE
       │
       └─ taskReadKeypad.execute() every 100ms
            ├─ Read key
            ├─ Process input (* = clear, # = submit, 0-9 = append)
            └─ If # pressed → verifyPIN()
                 └─ If correct → setRelayState(true) → RELAY_ACTIVE
```

## Safety and Edge Cases

### 1. Debouncing
- Keypad library handles debouncing internally
- No additional debouncing needed

### 2. Multiple Alert Signals
- Once in ALERT_MODE, ignore subsequent signals
- OTP remains valid until correct entry or reset

### 3. PIN Length Validation
- Only allow 4-digit PINs
- Auto-reject if > 4 digits entered before '#'

### 4. Random Seed
- Use `randomSeed(analogRead(A5))` on floating analog pin
- Ensures different OTP each time

### 5. Memory Management
- Use F() macro for Serial strings to save RAM
- String operations are minimal (4-char PINs only)

### 6. Relay Latching
- Once relay activates, it stays ON
- No automatic turn-off
- Requires device reset to clear

## Testing Plan

### Test 1: Normal Mode Operation
1. Power on Arduino 2 (no signal from Arduino 1)
2. Enter default PIN "1234" on keypad
3. Press '#' to submit
4. Verify relay turns ON
5. Verify latching behavior

### Test 2: Alert Mode OTP Generation
1. Power on Arduino 2
2. Send HIGH signal from Arduino 1 (D8)
3. Verify OTP appears on Serial Monitor
4. Verify OTP is 4 digits
5. Verify system is in ALERT_MODE

### Test 3: OTP Verification
1. Trigger alert mode
2. Note OTP from Serial Monitor
3. Enter OTP on keypad
4. Press '#' to submit
5. Verify relay turns ON

### Test 4: Incorrect PIN Handling
1. Enter wrong PIN in NORMAL_MODE
2. Verify error message
3. Verify relay stays OFF
4. Verify can retry

### Test 5: PIN Timeout (Alert Mode)
1. Trigger alert mode
2. Wait 30 seconds without entering PIN
3. Verify timeout message
4. Verify OTP still required

### Test 6: State Persistence
1. Activate relay (either mode)
2. Send additional signals from Arduino 1
3. Verify relay stays ON
4. Verify system ignores new inputs

## Wiring Diagram

```
Arduino UNO 2 Connections:
------------------------

Signal Input (from Arduino 1):
D8 (INPUT_PULLUP) ← Arduino 1 D8 (OUTPUT)
GND               ← Arduino 1 GND

Relay Module:
VCC → 5V
GND → GND
IN  → D7

3x4 Keypad:
Row 1 → D2
Row 2 → D3
Row 3 → D4
Row 4 → D5
Col 1 → D6
Col 2 → A0
Col 3 → A1
```

## Dependencies Installation

Install via Arduino Library Manager:
1. Keypad by Christopher Andrews
2. TaskScheduler by Anatoli Arkhipenko (if not already installed)

## Estimated Code Size
- Sketch size: ~8-10 KB
- Global variables: ~500 bytes
- Well within Arduino UNO limits (32 KB flash, 2 KB RAM)

## Implementation Notes

1. **Non-blocking**: All operations use TaskScheduler - no `delay()` calls
2. **Serial output**: Verbose for debugging, can be reduced for production
3. **OTP security**: OTP is printed to Serial (acceptable for this use case)
4. **Extensibility**: Easy to add LCD display, SMS notifications, etc.

---

## Questions Before Implementation

1. **Keypad model**: Do you have a specific 3x4 keypad model? (affects pin mapping)
2. **PIN timeout behavior**: Should timeout disable the system or just warn?
3. **Buzzer feedback**: Should we add buzzer beeps for key presses?
4. **LCD display**: Do you want OTP shown on an LCD instead of Serial?
5. **Maximum retry attempts**: Should there be a lockout after X wrong attempts?

---

## References
- [Keypad Library Documentation](https://docs.arduino.cc/libraries/keypad/)
- [Keypad GitHub Repository](https://github.com/Chris--A/Keypad)
- [TaskScheduler Library](https://github.com/arkhipenko/TaskScheduler)
- [TaskScheduler Wiki](https://github.com/arkhipenko/TaskScheduler/wiki)
- [Arduino Keypad Tutorial](https://lastminuteengineers.com/arduino-keypad-tutorial/)

---

**Status**: Plan ready for review
**Next Step**: Await user approval to proceed with implementation
