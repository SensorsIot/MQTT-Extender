#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <U8g2lib.h>

// -----------------------------
// Pin definitions for LoRa module
// -----------------------------
#define LORA_SCK   5
#define LORA_MISO  19
#define LORA_MOSI  27
#define LORA_CS    18
#define LORA_RST   23
#define LORA_DIO0  26

// -----------------------------
// Create an SPI instance for LoRa
// -----------------------------
SPIClass spiLora(VSPI);

// -----------------------------
// Initialize the LoRa module instance using RadioLib.
// The constructor takes the Chip Select, DIO0, Reset pins,
// a dummy parameter (-1), and the SPI instance.
// -----------------------------
SX1276 lora = new Module(LORA_CS, LORA_DIO0, LORA_RST, -1, spiLora);

// -----------------------------
// Initialize the OLED display using U8g2 library.
// -----------------------------
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(115200);
  Serial.println("System initializing...");

  // Initialize SPI for LoRa module with defined pins
  spiLora.begin(LORA_SCK, LORA_MISO, LORA_MOSI);
  Serial.println("SPI interface initialized.");

  // Initialize the OLED display
  display.begin();
  display.clearBuffer();
  display.setFont(u8g2_font_ncenB08_tr);
  display.drawStr(0, 10, "Transmitter Init");
  display.sendBuffer();
  Serial.println("OLED display initialized.");

  // Initialize the LoRa module with parameters:
  // Frequency: 434.0 MHz, Bandwidth: 125.0 kHz, Spreading Factor: 9,
  // Coding Rate: 7, Sync Word: 0x12.
  int state = lora.begin(434.0, 125.0, 7, 7, 0x12);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print("LoRa init failed with error code: ");
    Serial.println(state);

    // Display error message on OLED
    display.clearBuffer();
    display.setCursor(0, 10);
    display.print("LoRa init failed: ");
    display.println(state);
    display.sendBuffer();

    while (true); // Halt execution if initialization fails
  }
  Serial.println("LoRa module initialized successfully.");

  // Set LoRa module's output power to 10 dBm
  lora.setOutputPower(10);
  Serial.println("LoRa output power set to 10 dBm.");
}

void loop() {
  // Prepare a 1-byte payload to be sent
  uint8_t payload = 42;
  Serial.println("Sending message...");

  // Transmit the payload over LoRa
  int state = lora.transmit((uint8_t *)&payload, 1);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("Message sent successfully.");
    Serial.println("Waiting for reply...");

    // Buffer to store the received response (expecting at least 2 bytes)
    uint8_t buf[10];
    size_t len = sizeof(buf);
    int rcvState = lora.receive(buf, len);

    if (rcvState == RADIOLIB_ERR_NONE) {
      // Retrieve local LoRa signal metrics
      float localRSSI = lora.getRSSI();
      float localSNR = lora.getSNR();

      // Assume that the first two bytes of the received buffer contain
      // the remote device's RSSI and SNR, respectively
      int8_t remoteRSSI = buf[0];
      int8_t remoteSNR = buf[1];

      // Display local and remote signal metrics on the OLED display
      display.clearBuffer();
      display.setCursor(0, 10);
      display.print("Local RSSI: ");
      display.println(localRSSI);
      display.setCursor(0, 25);
      display.print("Local SNR: ");
      display.println(localSNR);
      display.setCursor(0, 40);
      display.print("Remote RSSI: ");
      display.println(remoteRSSI);
      display.setCursor(0, 55);
      display.print("Remote SNR: ");
      display.println(remoteSNR);
      display.sendBuffer();

      // Print the signal metrics to the Serial Monitor for debugging
      Serial.print("Local RSSI: ");
      Serial.println(localRSSI);
      Serial.print("Local SNR: ");
      Serial.println(localSNR);
      Serial.print("Remote RSSI: ");
      Serial.println(remoteRSSI);
      Serial.print("Remote SNR: ");
      Serial.println(remoteSNR);
    } else {
      Serial.print("Receive failed with error code: ");
      Serial.println(rcvState);
    }
  } else {
    Serial.print("Transmit failed with error code: ");
    Serial.println(state);
  }

  // Wait 3 seconds before the next transmission
  delay(3000);
}
