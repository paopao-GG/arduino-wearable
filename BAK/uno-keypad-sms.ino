// Arduino 2 - Relay Controller
// Receives alert signal from Arduino 1 (LoRa receiver) and activates relay
//
// Wiring: Signal Input from Arduino 1
// -----------------------------------------
// D8 (INPUT) -> Arduino 1 D8 (OUTPUT)
// GND        -> Arduino 1 GND (COMMON GROUND REQUIRED!)
//
// Wiring: 5V Single Channel Relay Module
// -----------------------------------------
// VCC  -> 5V
// GND  -> GND
// IN   -> D7 (relay signal pin)
//
// NOTE: Most relay modules are ACTIVE LOW:
//       - Write LOW to turn relay ON
//       - Write HIGH to turn relay OFF
//       Check your relay module datasheet!

#define SIGNAL_INPUT_PIN  8   // Signal from Arduino 1
#define RELAY_PIN         7   // Relay control pin

// Relay type configuration
// Set to true if your relay is ACTIVE HIGH (LOW = OFF, HIGH = ON)
// Set to false if your relay is ACTIVE LOW (LOW = ON, HIGH = OFF) - most common
#define RELAY_ACTIVE_HIGH false

// State variables
bool alertActive = false;
bool relayState = false;

void setup() {
  // Configure pins
  pinMode(SIGNAL_INPUT_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);

  // Initialize relay to OFF state
  setRelayState(false);

  // Setup serial for debugging
  Serial.begin(9600);
  delay(100);

  Serial.println(F("Arduino 2 - Relay Controller"));
  Serial.println(F("----------------------------"));
  Serial.println(F("Waiting for alert signal..."));
  Serial.println();
}

void loop() {
  // Read signal from Arduino 1 (non-blocking)
  int signalState = digitalRead(SIGNAL_INPUT_PIN);

  // Check if alert signal received (latch ON, never turn OFF)
  if (signalState == HIGH && !alertActive) {
    // Alert detected - turn ON relay and KEEP IT ON
    alertActive = true;
    setRelayState(true);

    Serial.println();
    Serial.println(F("*** ALERT SIGNAL RECEIVED! ***"));
    Serial.println(F("Relay: ON (LATCHED)"));
    Serial.println(F("Relay will stay ON until reset"));
    Serial.println(F("******************************"));
    Serial.println();
  }

  // Note: Relay stays ON even if CLEAR is received
  // To turn OFF: reset Arduino or power cycle

  // Small delay to avoid excessive polling
  delay(50);
}

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
