// Arduino UNO Receiver — RA-02 (SX1278) 433 MHz
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
// NOTE: SX1278 is NOT 5V tolerant on VCC!
//       SPI pins can tolerate 5V, but for safety consider level shifters.
//       Arduino UNO's 3.3V pin can supply ~50mA, which may be marginal
//       for LoRa TX. For RX-only, it should be fine.

#include <SPI.h>
#include <LoRa.h>

// Arduino UNO SPI pins are fixed (D11=MOSI, D12=MISO, D13=SCK)
// Only need to define control pins
#define LORA_SS    10   // NSS/CS pin
#define LORA_RST   9    // Reset pin
#define LORA_DIO0  2    // DIO0 for interrupt

// Frequency for RA-02 module (433 MHz version)
// Change to 868E6 or 915E6 if you have different module
const long LORA_FREQ = 433E6;

void setup() {
  Serial.begin(9600);
  delay(1000);  // Give time for serial to initialize

  Serial.println("Arduino UNO LoRa Receiver");
  Serial.println("-------------------------");

  // Configure LoRa control pins
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  // Initialize LoRa at specified frequency
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("ERROR: LoRa init failed!");
    Serial.println("Check: wiring, antenna, power supply");
    while (1);  // Halt
  }

  // Configure radio parameters - MUST match transmitter settings!
  LoRa.setSpreadingFactor(10);     // SF10 - must match TX (was SF7)
  LoRa.setSignalBandwidth(125E3);  // 125 kHz bandwidth
  LoRa.setCodingRate4(8);          // 4/8 coding rate - must match TX
  LoRa.enableCrc();                // Enable CRC checking
  LoRa.setSyncWord(0x12);          // Private sync word - must match TX

  Serial.println("LoRa RX initialized at 433 MHz");
  Serial.println("Waiting for packets...");
  Serial.println();
}

void loop() {
  // Check for incoming packet
  int packetSize = LoRa.parsePacket();

  if (packetSize) {
    // Packet received - read the data
    String incoming = "";
    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }

    // Get signal quality metrics
    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();

    // Print received data
    Serial.print("Received: ");
    Serial.print(incoming);
    Serial.print(" | RSSI: ");
    Serial.print(rssi);
    Serial.print(" dBm | SNR: ");
    Serial.print(snr, 1);
    Serial.println(" dB");
  }
}
