// Arduino Nano Transmitter — RA-02 (SX1278) 433 MHz
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
// IMPORTANT: Arduino Nano's onboard 3.3V regulator cannot provide
//            enough current for LoRa transmission (~120mA peak).
//            Use an external AMS1117-3.3V or similar regulator!
//
// NOTE: SX1278 is NOT 5V tolerant on VCC!
//       SPI pins can tolerate 5V, but for safety consider level shifters.

#include <SPI.h>
#include <LoRa.h>

// Arduino Nano SPI pins are fixed (D11=MOSI, D12=MISO, D13=SCK)
// Only need to define control pins
#define LORA_SS    10   // NSS/CS pin
#define LORA_RST   9    // Reset pin
#define LORA_DIO0  2    // DIO0 for interrupt

// Frequency for RA-02 module (433 MHz version)
// Change to 868E6 or 915E6 if you have different module
const long LORA_FREQ = 433E6;

// Transmission interval (milliseconds)
const unsigned long TX_INTERVAL = 2000;  // Send every 2 seconds

unsigned long lastTxTime = 0;
uint32_t packetCounter = 0;

void setup() {
  Serial.begin(9600);
  delay(1000);  // Give time for serial to initialize (don't block if no monitor)

  Serial.println("Arduino Nano LoRa Transmitter");
  Serial.println("-----------------------------");

  // Configure LoRa control pins
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  // Initialize LoRa at specified frequency
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("ERROR: LoRa init failed!");
    Serial.println("Check: wiring, antenna, power supply");
    while (1);  // Halt
  }

  // Configure radio parameters - MUST match receiver settings!
  LoRa.setTxPower(20);             // TX power max (20 dBm) for better range
  LoRa.setSpreadingFactor(10);     // SF10 - much better noise immunity (was SF7)
  LoRa.setSignalBandwidth(125E3);  // 125 kHz bandwidth
  LoRa.setCodingRate4(8);          // 4/8 coding rate - max error correction
  LoRa.enableCrc();                // Enable CRC for error detection
  LoRa.setSyncWord(0x12);          // Private sync word to filter noise

  Serial.println("LoRa TX initialized at 433 MHz");
  Serial.println("Starting transmission...");
  Serial.println();
}

void loop() {
  unsigned long currentTime = millis();

  // Send packet at regular intervals
  if (currentTime - lastTxTime >= TX_INTERVAL) {
    lastTxTime = currentTime;

    // Create payload
    String payload = "Hello #" + String(packetCounter++);

    Serial.print("Sending: ");
    Serial.print(payload);
    Serial.print(" ... ");

    // Transmit the packet
    LoRa.beginPacket();
    LoRa.print(payload);
    LoRa.endPacket();  // Blocking until transmission complete

    Serial.println("done!");
  }
}
