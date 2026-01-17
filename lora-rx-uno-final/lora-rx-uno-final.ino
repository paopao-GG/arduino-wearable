// Arduino UNO Receiver — LoRa Alert System with LED Indicator
// Receives alerts from pulse oximeter transmitter via RA-02 (SX1278) 433 MHz
//
// Wiring: RA-02 Module -> Arduino UNO
// -----------------------------------------
// VCC   -> 3.3V (NOT 5V! Module is 3.3V only)
// GND   -> GND
// NSS   -> D10 (SS)
// MOSI  -> D11 (hardware SPI)
// MISO  -> D12 (hardware SPI)
// SCK   -> D13 (hardware SPI)
// RST   -> D9
// DIO0  -> D2
//
// Wiring: Alert LED
// -----------------------------------------
// LED+  -> D8 (through 220-330 ohm resistor)
// LED-  -> GND
//
// NOTE: SX1278 is NOT 5V tolerant on VCC!
//       SPI pins can tolerate 5V, but for safety consider level shifters.
//       Arduino UNO's 3.3V pin can supply ~50mA, which is fine for RX-only.

#include <SPI.h>
#include <LoRa.h>

// ============ PIN DEFINITIONS ============
#define LORA_SS    10
#define LORA_RST   9
#define LORA_DIO0  2

#define ALERT_LED  8   // Alert indicator LED

// ============ CONSTANTS ============
const long LORA_FREQ = 433E6;

// ============ STATE VARIABLES ============
bool alertActive = false;

// Received data storage
int receivedBPM = 0;
int receivedTemp = 0;
String alertType = "";

// ============ SETUP ============
void setup() {
  pinMode(ALERT_LED, OUTPUT);
  digitalWrite(ALERT_LED, LOW);

  Serial.begin(9600);
  delay(1000);

  Serial.println(F("LoRa Alert Receiver"));
  Serial.println(F("-------------------"));

  // Configure LoRa pins
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  // Initialize LoRa
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println(F("ERROR: LoRa init failed!"));
    Serial.println(F("Check: wiring, antenna, power"));
    // Blink LED rapidly to indicate error
    while (1) {
      digitalWrite(ALERT_LED, HIGH);
      delay(100);
      digitalWrite(ALERT_LED, LOW);
      delay(100);
    }
  }

  // Configure radio parameters - MUST match transmitter!
  LoRa.setSpreadingFactor(10);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);
  LoRa.enableCrc();
  LoRa.setSyncWord(0x12);

  Serial.println(F("LoRa RX OK at 433 MHz"));
  Serial.println(F("Waiting for alerts..."));
  Serial.println();

}

// ============ MAIN LOOP (NON-BLOCKING) ============
void loop() {
  // Check for incoming LoRa packets (non-blocking)
  checkLoRa();
}

// ============ CHECK LORA PACKETS (NON-BLOCKING) ============
void checkLoRa() {
  int packetSize = LoRa.parsePacket();

  if (packetSize) {
    // Read the incoming packet
    String incoming = "";
    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }

    // Get signal quality
    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();

    // Process the message
    processMessage(incoming);

    // Print debug info
    Serial.print(F("RX: "));
    Serial.print(incoming);
    Serial.print(F(" | RSSI: "));
    Serial.print(rssi);
    Serial.print(F(" dBm | SNR: "));
    Serial.print(snr, 1);
    Serial.println(F(" dB"));
  }
}

// ============ PROCESS RECEIVED MESSAGE ============
void processMessage(String msg) {
  msg.trim();

  if (msg.startsWith("ALERT:")) {
    // Parse alert message
    alertActive = true;
    alertType = msg.substring(6);  // Remove "ALERT:" prefix

    // Extract BPM if present
    int bpmIdx = alertType.indexOf("BPM=");
    if (bpmIdx >= 0) {
      int endIdx = alertType.indexOf(',', bpmIdx);
      if (endIdx < 0) endIdx = alertType.length();
      receivedBPM = alertType.substring(bpmIdx + 4, endIdx).toInt();
    }

    // Extract TEMP if present
    int tempIdx = alertType.indexOf("TEMP=");
    if (tempIdx >= 0) {
      int endIdx = alertType.indexOf(',', tempIdx);
      if (endIdx < 0) endIdx = alertType.length();
      receivedTemp = alertType.substring(tempIdx + 5, endIdx).toInt();
    }

    // Turn ON LED when alert received
    digitalWrite(ALERT_LED, HIGH);

    Serial.println();
    Serial.println(F("*** ALERT RECEIVED! ***"));
    if (receivedBPM > 0) {
      Serial.print(F("  High BPM: "));
      Serial.println(receivedBPM);
    }
    if (receivedTemp > 0) {
      Serial.print(F("  High Temp: "));
      Serial.print(receivedTemp);
      Serial.println(F(" C"));
    }
    Serial.println(F("LED ON"));
    Serial.println(F("***********************"));
    Serial.println();

  } else if (msg.equals("CLEAR")) {
    // Alert cleared
    alertActive = false;
    receivedBPM = 0;
    receivedTemp = 0;
    alertType = "";

    // Turn OFF LED when clear received
    digitalWrite(ALERT_LED, LOW);

    Serial.println();
    Serial.println(F("=== ALERT CLEARED ==="));
    Serial.println(F("LED OFF"));
    Serial.println();
  }
}

