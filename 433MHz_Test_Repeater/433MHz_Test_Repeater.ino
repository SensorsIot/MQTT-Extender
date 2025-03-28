/*
 * TTGO LoRa32 Repeater Sketch with Debug Messages
 *
 * This sketch configures the TTGO LoRa32 board as a LoRa repeater.
 * It listens for incoming LoRa packets, displays the RSSI and SNR values on an OLED,
 * and sends an acknowledgment packet containing these signal metrics.
 * Debug messages are printed to the Serial monitor to help trace the execution.
 */

#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <U8g2lib.h>

// Pin definitions for the LoRa module
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_CS 18
#define LORA_RST 23
#define LORA_DIO0 26

// Create a SPI interface using VSPI for the LoRa module
SPIClass spiLora(VSPI);

// Initialize the LoRa module instance using RadioLib.
// Constructor parameters: Chip Select, DIO0, Reset, unused pin (-1), and SPI interface.
SX1276 lora = new Module(LORA_CS, LORA_DIO0, LORA_RST, -1, spiLora);

// Initialize the OLED display (128x64 pixels) using hardware I2C
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

void setup() {
  // Start Serial communication for debugging at 115200 baud
  Serial.begin(115200);
  while (!Serial)
    ;  // Wait for Serial port to be ready

  Serial.println("Initializing SPI interface for LoRa...");
  // Initialize SPI for LoRa communication with defined SCK, MISO, and MOSI pins
  spiLora.begin(LORA_SCK, LORA_MISO, LORA_MOSI);

  Serial.println("Setting up LoRa parameters...");

  Serial.println("Initializing OLED display...");
  // Initialize the OLED display and show an initialization message
  display.begin();
  display.clearBuffer();
  display.setFont(u8g2_font_ncenB08_tr);
  display.drawStr(0, 10, "Repeater Init");
  display.sendBuffer();

  Serial.println("Initializing LoRa module...");
  // spreading factor (9), coding rate (7), and sync word (0x12).
  int state = lora.begin(434.0, 125.0, 7, 7, 0x12);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("LoRa init failed with error code: ");
    Serial.println(state);
    while (true)
      ;  // Halt execution if LoRa fails to initialize
  }

  // Set the LoRa module's transmission power to 10 dBm
  lora.setOutputPower(10);
  Serial.println("LoRa module initialized successfully. Entering main loop...");
}

void loop() {
  uint8_t buf[10];  // Buffer to store received data (up to 10 bytes)
  size_t len = sizeof(buf);

  // Attempt to receive a packet over LoRa
  int state = lora.receive(buf, len);

  // Check if a packet was received successfully
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Packet received!");

    // Retrieve the received signal strength (RSSI) and signal-to-noise ratio (SNR)
    float rssi = lora.getRSSI();
    float snr = lora.getSNR();
    Serial.print("RSSI: ");
    Serial.println(rssi);
    Serial.print("SNR: ");
    Serial.println(snr);

    // Update the OLED display with RSSI and SNR values
    display.clearBuffer();
    display.setCursor(0, 15);
    display.print("Remote RSSI: ");
    display.println(rssi);
    display.setCursor(0, 35);
    display.print("Remote SNR: ");
    display.println(snr);
    display.sendBuffer();

    // Prepare an acknowledgment packet containing the RSSI and SNR values.
    // Casting float values to int8_t for a compact 2-byte payload.
    uint8_t ack[2];
    ack[0] = (int8_t)rssi;
    ack[1] = (int8_t)snr;

    // Transmit the acknowledgment packet back to the sender
    Serial.println("Transmitting acknowledgment...");
    lora.transmit(ack, 2);
    Serial.println("Acknowledgment transmitted.");
  } else {
    // To avoid flooding the Serial monitor, only log when no packet is received at intervals.
    static unsigned long lastDebugTime = 0;
    if (millis() - lastDebugTime > 5000) {
      Serial.print("Listening for packets... (state code: ");
      Serial.print(state);
      Serial.println(")");
      lastDebugTime = millis();
    }
  }
}
