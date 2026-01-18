// Arduino 2 - Keypad OTP Relay Controller
// Receives alert signal from Arduino 1 and requires PIN/OTP entry to activate relay
//
// Wiring: Signal Input from Arduino 1
// -----------------------------------------
// D8 (INPUT_PULLUP) -> Arduino 1 D8 (OUTPUT)
// GND               -> Arduino 1 GND (COMMON GROUND REQUIRED!)
//
// Wiring: 5V Single Channel Relay Module
// -----------------------------------------
// VCC  -> 5V
// GND  -> GND
// IN   -> D7 (relay signal pin)
//
// Wiring: 4x4 Matrix Keypad
// -----------------------------------------
// Row 1 -> D2
// Row 2 -> D3
// Row 3 -> D4
// Row 4 -> D5
// Col 1 -> D6
// Col 2 -> A1
// Col 3 -> A2
// Col 4 -> A3
//
// NOTE: Most relay modules are ACTIVE LOW:
//       - Write LOW to turn relay ON
//       - Write HIGH to turn relay OFF
//       Check your relay module datasheet!

#include <Keypad.h>

// ============ PIN DEFINITIONS ============
#define SIGNAL_INPUT_PIN  8   // Signal from Arduino 1
#define RELAY_PIN         7   // Relay control pin

// ============ RELAY CONFIGURATION ============
// Set to true if your relay is ACTIVE HIGH (LOW = OFF, HIGH = ON)
// Set to false if your relay is ACTIVE LOW (LOW = ON, HIGH = OFF) - most common
#define RELAY_ACTIVE_HIGH false

// ============ KEYPAD CONFIGURATION ============
const byte ROWS = 4;
const byte COLS = 4;

char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};

byte rowPins[ROWS] = {2, 3, 4, 5};
byte colPins[COLS] = {6, A1, A2, A3};

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// Set keypad scan interval (default is 10ms, increase for reliability)
void setupKeypad() {
  keypad.setDebounceTime(50);  // 50ms debounce
  keypad.setHoldTime(1000);     // 1s hold time
}

// ============ CONSTANTS ============
const String DEFAULT_PIN = "1234";
const unsigned long PIN_TIMEOUT_US = 30000000;  // 30 seconds in microseconds

// ============ STATE MACHINE ============
enum SystemState {
  NORMAL_MODE,    // No alert, default PIN works
  ALERT_MODE,     // Alert received, OTP required
  RELAY_ACTIVE    // Relay latched ON
};

SystemState currentState = NORMAL_MODE;

// ============ STATE VARIABLES ============
String currentPIN = "";
String generatedOTP = "";
bool relayState = false;
bool signalPreviouslyHigh = false;
unsigned long alertStartTime = 0;
bool alertTimeoutActive = false;

// ============ SETUP ============
void setup() {
  // Configure pins
  pinMode(SIGNAL_INPUT_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);

  // Initialize relay to OFF state
  setRelayState(false);

  // Setup serial for debugging
  Serial.begin(9600);
  delay(100);

  // Configure keypad
  setupKeypad();

  // Seed random number generator with better entropy
  randomSeed(analogRead(A4) ^ micros());

  // Display startup message
  Serial.println(F("Arduino 2 - Keypad OTP Relay Controller"));
  Serial.println(F("----------------------------------------"));
  Serial.println(F("Keypad initialized (4x4 matrix)"));
  Serial.println(F("Default PIN: 1234 (NORMAL MODE)"));
  Serial.println();
  Serial.println(F("Waiting for alert signal or PIN entry..."));
  Serial.println(F("Enter PIN and press '#' to submit"));
  Serial.println(F("Press '*' to clear entry"));
  Serial.println();

  // Check if signal is already HIGH on boot
  if (digitalRead(SIGNAL_INPUT_PIN) == HIGH) {
    generateAndDisplayOTP();
    currentState = ALERT_MODE;
    alertStartTime = millis();
    alertTimeoutActive = true;
  }
}

// ============ MAIN LOOP ============
void loop() {
  // Check for alert signal
  checkSignal();

  // Read keypad
  readKeypad();

  // Check for timeout
  checkTimeout();
}

// ============ CHECK SIGNAL ============
void checkSignal() {
  // Only respond to signal in NORMAL_MODE
  if (currentState == RELAY_ACTIVE) {
    return;
  }

  int signalState = digitalRead(SIGNAL_INPUT_PIN);

  // Detect rising edge (LOW to HIGH transition)
  if (signalState == HIGH && !signalPreviouslyHigh && currentState == NORMAL_MODE) {
    // Alert detected - transition to ALERT_MODE
    generateAndDisplayOTP();
    currentState = ALERT_MODE;
    alertStartTime = millis();
    alertTimeoutActive = true;
    currentPIN = "";  // Clear any partial entry

    Serial.println();
    Serial.println(F("*** ALERT SIGNAL RECEIVED! ***"));
    Serial.print(F("OTP GENERATED: "));
    Serial.println(generatedOTP);
    Serial.println(F("Enter OTP to activate relay"));
    Serial.println(F("Timeout: 30 seconds"));
    Serial.println(F("********************************"));
    Serial.println();
  }

  signalPreviouslyHigh = (signalState == HIGH);
}

// ============ READ KEYPAD ============
void readKeypad() {
  // Don't read keypad if relay is already active
  if (currentState == RELAY_ACTIVE) {
    return;
  }

  char key = keypad.getKey();

  if (key) {
    // Display keypress feedback
    Serial.print(F("Key pressed: "));
    Serial.println(key);

    if (key >= '0' && key <= '9') {
      // Numeric key - append to PIN
      if (currentPIN.length() < 4) {
        currentPIN += key;

        // Display masked PIN entry
        Serial.print(F("PIN Entry: "));
        for (int i = 0; i < currentPIN.length(); i++) {
          Serial.print(currentPIN[i]);
        }
        for (int i = currentPIN.length(); i < 4; i++) {
          Serial.print('_');
        }
        Serial.println();
      } else {
        Serial.println(F("! Max 4 digits - press '#' to submit or '*' to clear"));
      }
    }
    else if (key == '#') {
      // Submit PIN for verification
      if (currentPIN.length() == 4) {
        verifyPIN(currentPIN);
      } else {
        Serial.println(F("! PIN must be 4 digits"));
        Serial.print(F("Current entry: "));
        Serial.print(currentPIN.length());
        Serial.println(F(" digits"));
      }
    }
    else if (key == '*') {
      // Clear PIN entry
      currentPIN = "";
      Serial.println(F("PIN cleared - enter new PIN"));
    }
  }
}

// ============ CHECK TIMEOUT ============
void checkTimeout() {
  // Only check timeout in ALERT_MODE
  if (currentState == ALERT_MODE && alertTimeoutActive) {
    if (millis() - alertStartTime >= 30000) {  // 30 seconds
      alertTimeoutActive = false;

      Serial.println();
      Serial.println(F("!!! PIN ENTRY TIMEOUT !!!"));
      Serial.print(F("OTP still required: "));
      Serial.println(generatedOTP);
      Serial.println(F("System locked - enter OTP or reset"));
      Serial.println();
    }
  }
}

// ============ GENERATE OTP ============
void generateAndDisplayOTP() {
  // Generate random 4-digit OTP (0000-9999)
  long otpNumber = random(0, 10000);

  // Format with leading zeros
  generatedOTP = "";
  if (otpNumber < 10) {
    generatedOTP = "000";
  } else if (otpNumber < 100) {
    generatedOTP = "00";
  } else if (otpNumber < 1000) {
    generatedOTP = "0";
  }
  generatedOTP += String(otpNumber);
}

// ============ VERIFY PIN ============
void verifyPIN(String enteredPIN) {
  bool pinCorrect = false;
  String expectedPIN = "";

  // Determine expected PIN based on current state
  if (currentState == NORMAL_MODE) {
    expectedPIN = DEFAULT_PIN;
  } else if (currentState == ALERT_MODE) {
    expectedPIN = generatedOTP;
  }

  // Compare PINs
  if (enteredPIN.equals(expectedPIN)) {
    pinCorrect = true;
  }

  Serial.println(F("Verifying..."));
  delay(500);  // Brief delay for user feedback

  if (pinCorrect) {
    // Correct PIN - activate relay
    setRelayState(true);
    currentState = RELAY_ACTIVE;
    alertTimeoutActive = false;  // Stop timeout timer

    Serial.println();
    Serial.println(F("================================"));
    Serial.println(F("✓ PIN CORRECT!"));
    Serial.println(F("Relay: ON (LATCHED)"));
    Serial.println(F("System locked until reset"));
    Serial.println(F("================================"));
    Serial.println();
  } else {
    // Incorrect PIN
    Serial.println();
    Serial.println(F("✗ INCORRECT PIN!"));
    Serial.println(F("Try again..."));
    Serial.println();
  }

  // Clear current PIN entry
  currentPIN = "";
}

// ============ SET RELAY STATE ============
// Set relay state accounting for ACTIVE_HIGH or ACTIVE_LOW configuration
void setRelayState(bool state) {
  relayState = state;

  if (RELAY_ACTIVE_HIGH) {
    // Active HIGH: true = HIGH (ON), false = LOW (OFF)
    digitalWrite(RELAY_PIN, state ? HIGH : LOW);
  } else {
    // Active LOW: true = LOW (ON), false = HIGH (OFF)
    digitalWrite(RELAY_PIN, state ? LOW : HIGH);
  }
}
