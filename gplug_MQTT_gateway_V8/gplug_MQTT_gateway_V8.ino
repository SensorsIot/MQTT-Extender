#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <MBUS_data_structure.h>  // Contains the updated SensorDataPacked structure
#include <secrets.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ----- Three-Level Debugging Macros -----
// Set the debug level at compile time (1 = simple, 2 = intermediate, 3 = extended)
#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL 3
#endif

#if DEBUG_LEVEL >= 1
#define DEBUG_SIMPLE(x) Serial.println(x)
#define DEBUG_SIMPLEF(fmt, ...) Serial.printf(fmt, __VA_ARGS__)
#else
#define DEBUG_SIMPLE(x)
#define DEBUG_SIMPLEF(fmt, ...)
#endif

#if DEBUG_LEVEL >= 2
#define DEBUG_INTERMEDIATE(x) Serial.println(x)
#define DEBUG_INTERMEDIATEF(fmt, ...) Serial.printf(fmt, __VA_ARGS__)
#else
#define DEBUG_INTERMEDIATE(x)
#define DEBUG_INTERMEDIATEF(fmt, ...)
#endif

#if DEBUG_LEVEL >= 3
#define DEBUG_EXTENDED(x) Serial.println(x)
#define DEBUG_EXTENDEDF(fmt, ...) Serial.printf(fmt, __VA_ARGS__)
#else
#define DEBUG_EXTENDED(x)
#define DEBUG_EXTENDEDF(fmt, ...)
#endif

// --------- WiFi and MQTT Settings ---------
#define MQTT_PORT         1883
#define MQTT_TOPIC_VALUES "MBUS/values"
#define MQTT_TOPIC_SIGNAL "MBUS/signal"
#define MQTT_TOPIC_LOG    "MBUS/log"

// Create WiFi and MQTT clients.
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// --------- LoRa Settings (TTGO LoRa32 V1.6.1, 433MHz) ---------
#define LORA_SCK   5
#define LORA_MISO  19
#define LORA_MOSI  27
#define LORA_CS    18
#define LORA_RST   23
#define LORA_DIO0  26

// Macro to get only the file name from __FILE__
#define __SHORT_FILE__ ((strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)))

// --------- ACK and NACK Settings ---------
enum {
  ACK_BYTE = 0x55,
  NACK_BYTE = 0xAA
};

// Expected size for SensorDataPacked (includes 16-bit CRC field)
#define EXPECTED_MSG_SIZE (sizeof(SensorDataPacked))

// --------- OLED Display Settings ---------
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --------- Buffer for Received LoRa Packet ---------
#define RX_BUFFER_SIZE 128
volatile bool packetReceived = false;
volatile uint8_t rxBuffer[RX_BUFFER_SIZE];
volatile int rxLength = 0;

// --------- Timing Settings ---------
#define PACKET_TIMEOUT         120000UL   // 2 minutes
unsigned long lastPacketTime = 0;
unsigned long lastMQTTClientConnect = 0;    // Timestamp of last MQTT activity
#define MQTT_CONNECT_TIMEOUT   60000UL    // 60 seconds
#define MAX_RETRIES            5

// --------- OLED Display Functions ---------
void displayMBUSGateway() {
  display.setCursor(25, 55);
  display.println("MBUS-Gateway");
  display.display();
}

void displaySensorData(const SensorDataPacked& data) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Pi:" + String(data.Pi, 2) + " Po:" + String(data.Po, 2));
  display.setCursor(0, 10);
  display.println("i1:" + String(data.Pi1, 1) + " i2:" + String(data.Pi2, 1) + " i3:" + String(data.Pi3, 1));
  display.setCursor(0, 20);
  display.println("o1:" + String(data.Po1, 1) + " o2:" + String(data.Po2, 1) + " o3:" + String(data.Po3, 1));
  displayMBUSGateway();
}

void displaySignalQuality(int rssi, float snr) {
  display.fillRect(0, 30, SCREEN_WIDTH, 10, SSD1306_BLACK);
  display.setCursor(0, 30);
  display.print("RSSI: ");
  display.print(rssi);
  display.print(" SNR: ");
  display.print(snr);
  displayMBUSGateway();
}

void displayAckStatus(const String& statusMessage) {
  display.fillRect(0, 40, SCREEN_WIDTH, 10, SSD1306_BLACK);
  display.setCursor(0, 40);
  display.println(statusMessage);
  displayMBUSGateway();
}

void displayError(const String& message) {
  display.fillRect(0, 40, SCREEN_WIDTH, 10, SSD1306_BLACK);
  display.setCursor(0, 40);
  display.println(message);
  displayMBUSGateway();
}

// --------- CRC16 Calculation (CCITT, polynomial 0x1021) ---------
uint16_t crc16(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;  // Initial value
  for (size_t i = 0; i < length; i++) {
    crc ^= ((uint16_t)data[i] << 8);
    for (int j = 0; j < 8; j++) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc = (crc << 1);
    }
  }
  return crc;
}

// --------- System and MQTT Functions ---------
void rebootSystem(const String& reason) {
  DEBUG_SIMPLE("Critical error: " + reason);
  mqttLog("Critical error: " + reason);
  displayError(reason);
  delay(3000);
  ESP.restart();
}

void reconnectMQTT() {
  int retryCount = 0;
  while (!mqttClient.connected() && retryCount < MAX_RETRIES) {
    DEBUG_INTERMEDIATEF("Attempting MQTT connection... (Attempt %d of %d)\n", retryCount + 1, MAX_RETRIES);

    String clientId = "ESP32Receiver-";
    clientId += String(random(0xffff), HEX);

    if (mqttClient.connect(clientId.c_str())) {
      DEBUG_SIMPLE("MQTT connected");
      lastMQTTClientConnect = millis();
      return;
    } else {
      DEBUG_INTERMEDIATEF("MQTT connection failed, rc=%d\n", mqttClient.state());
      mqttLog("MQTT connection failed with state: " + String(mqttClient.state()));
      retryCount++;
      if (retryCount < MAX_RETRIES) {
        delay(5000);  // Reduced delay for faster retry
      } else {
        DEBUG_SIMPLE("Max retry attempts reached, rebooting...");
        mqttLog("MQTT connection failed after max retries, rebooting system.");
        rebootSystem("MQTT connection failed after max retries.");
      }
    }
  }
}

void mqttLog(const String& message) {
  DEBUG_EXTENDED("MQTT Log (Error): " + message);
  if (!mqttClient.publish(MQTT_TOPIC_LOG, message.c_str())) {
    DEBUG_INTERMEDIATE("MQTT log publish failed for " + String(MQTT_TOPIC_LOG));
  }
}

void checkPacketTimeout() {
  if (millis() - lastPacketTime > PACKET_TIMEOUT) {
    rebootSystem("Packet timeout");
  }
}

// --------- LoRa Interrupt Handler ---------
void IRAM_ATTR onReceive(int packetSize) {
  if (packetSize == 0) return;
  rxLength = 0;
  while (LoRa.available() && rxLength < RX_BUFFER_SIZE) {
    rxBuffer[rxLength++] = LoRa.read();
  }
  packetReceived = true;
  lastPacketTime = millis();
  DEBUG_EXTENDED("LoRa packet received, size: " + String(packetSize));
}

// --------- LoRa ACK/NACK Transmission ---------
bool sendLoRaAck() {
  LoRa.beginPacket();
  LoRa.write(ACK_BYTE);
  int result = LoRa.endPacket();
  if (result == 0) {
    DEBUG_INTERMEDIATE("Failed to send ACK (LoRa.endPacket returned 0)");
    mqttLog("Failed to send ACK (LoRa.endPacket returned 0)");
    return false;
  }
  LoRa.receive();
  DEBUG_EXTENDED("ACK sent to transmitter");
  return true;
}

bool sendLoRaNack() {
  LoRa.beginPacket();
  LoRa.write(NACK_BYTE);
  int result = LoRa.endPacket();
  if (result == 0) {
    DEBUG_INTERMEDIATE("Failed to send NACK (LoRa.endPacket returned 0)");
    mqttLog("Failed to send NACK (LoRa.endPacket returned 0)");
    return false;
  }
  LoRa.receive();
  DEBUG_EXTENDED("NACK sent to transmitter");
  return true;
}

// --------- Hardware Initialization ---------
bool initializeHardware() {
  // Initialize OLED display.
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    DEBUG_SIMPLE("SSD1306 allocation failed");
    return false;
  }
  display.clearDisplay();
  display.display();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("OLED init OK!");
  display.display();
  DEBUG_SIMPLE("OLED initialized");

  // Set WiFi to Station mode.
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    DEBUG_SIMPLE(".");
  }
  DEBUG_SIMPLE("\nWiFi connected, IP: " + WiFi.localIP().toString());

  // Set up MQTT client.
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  DEBUG_SIMPLE("MQTT client configured");

  // Initialize LoRa module.
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    DEBUG_SIMPLE("Starting LoRa failed!");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("LoRa init fail!");
    display.display();
    return false;
  }
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setTxPower(20);
  DEBUG_SIMPLE("LoRa initialized");
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("LoRa init OK!");
  display.display();

  return true;
}

// --------- Setup and Loop ---------
void setup() {
  Serial.begin(115200);
  while (!Serial)
    ;  // Wait for Serial to be ready
  DEBUG_SIMPLE("\n-------------------------------------------");
  DEBUG_SIMPLEF("File: %s", __SHORT_FILE__);  // Fixed line
  DEBUG_SIMPLE("\n-------------------------------------------");
  DEBUG_SIMPLE("\nMBUS LoRa to MQTT Receiver starting...");
  lastMQTTClientConnect = millis();
  if (!initializeHardware()) {
    while (true) {
      delay(1000);
    }
  }

  // Setup OTA updates.
  ArduinoOTA.onStart([]() {
    String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
    DEBUG_SIMPLE("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    DEBUG_SIMPLE("\nOTA Update End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    DEBUG_EXTENDEDF("OTA Progress: %u%%\n", (progress * 100) / total);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    DEBUG_SIMPLEF("OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) DEBUG_SIMPLE("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) DEBUG_SIMPLE("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) DEBUG_SIMPLE("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) DEBUG_SIMPLE("Receive Failed");
    else if (error == OTA_END_ERROR) DEBUG_SIMPLE("End Failed");
  });
  ArduinoOTA.setHostname("MBUS_Gateway");
  ArduinoOTA.begin();
  DEBUG_SIMPLE("OTA initialized");

  reconnectMQTT();  // Connect to the MQTT broker.
  LoRa.onReceive(onReceive);
  LoRa.receive();
  lastPacketTime = millis();
  DEBUG_SIMPLE("LoRa receiver started");
}

void loop() {
  ArduinoOTA.handle();

  if (WiFi.status() != WL_CONNECTED) {
    rebootSystem("WiFi lost");
  }

  if (!mqttClient.connected()) {
    reconnectMQTT();
  }

  mqttClient.loop();
  checkPacketTimeout();

  // Reboot if no MQTT activity occurs within the timeout period.
  if (millis() - lastMQTTClientConnect > MQTT_CONNECT_TIMEOUT) {
    rebootSystem("No MQTT client connected within 60 seconds");
  }

  // Process received LoRa packet.
  if (packetReceived) {
    noInterrupts();
    int packetSize = rxLength;
    uint8_t packetBuffer[RX_BUFFER_SIZE];
    memcpy(packetBuffer, (const void*)rxBuffer, rxLength);
    packetReceived = false;
    interrupts();

    if (packetSize == EXPECTED_MSG_SIZE) {
      SensorDataPacked sensorData;
      memcpy(&sensorData, packetBuffer, EXPECTED_MSG_SIZE);

      // Compute CRC16 over the received data (excluding the CRC field).
      uint16_t computedCrc = crc16((uint8_t*)&sensorData, sizeof(sensorData) - sizeof(sensorData.parity));
      if (computedCrc != sensorData.parity) {
        DEBUG_INTERMEDIATE("CRC error detected. Sending NACK and discarding packet.");
        mqttLog("CRC error detected. Sending NACK and discarding packet.");
        sendLoRaNack();
        return;
      } else {
        if (!sendLoRaAck()) {
          DEBUG_INTERMEDIATE("Failed to send ACK.");
          mqttLog("Failed to send ACK.");
        }
      }

      // Build JSON payload from sensor data.
      StaticJsonDocument<512> doc;
      doc["Pi"] = sensorData.Pi;
      doc["Po"] = sensorData.Po;
      doc["Pi1"] = sensorData.Pi1;
      doc["Pi2"] = sensorData.Pi2;
      doc["Pi3"] = sensorData.Pi3;
      doc["Po1"] = sensorData.Po1;
      doc["Po2"] = sensorData.Po2;
      doc["Po3"] = sensorData.Po3;
      doc["I1"] = sensorData.I1;
      doc["I2"] = sensorData.I2;
      doc["I3"] = sensorData.I3;
      doc["Ei"] = sensorData.Ei;
      doc["Eo"] = sensorData.Eo;
      doc["Ei1"] = sensorData.Ei1;
      doc["Ei2"] = sensorData.Ei2;
      doc["Eo1"] = sensorData.Eo1;
      doc["Eo2"] = sensorData.Eo2;
      doc["Q5"] = sensorData.Q5;
      doc["Q6"] = sensorData.Q6;
      doc["Q7"] = sensorData.Q7;
      doc["Q8"] = sensorData.Q8;

      String jsonPayload;
      serializeJson(doc, jsonPayload);
      DEBUG_EXTENDED("Decoded LoRa binary message:");
      DEBUG_EXTENDED(jsonPayload);

      if (!mqttClient.publish(MQTT_TOPIC_VALUES, jsonPayload.c_str())) {
        DEBUG_INTERMEDIATE("MQTT publish failed for MBUS/values.");
        mqttLog("MQTT publish failed for MBUS/values.");
      } else {
        lastMQTTClientConnect = millis();  // Reset MQTT activity timestamp.
      }

      // Publish signal quality.
      int rssi = LoRa.packetRssi();
      float snr = LoRa.packetSnr();
      DEBUG_EXTENDEDF("Packet RSSI: %d dB, SNR: %.2f dB\n", rssi, snr);
      String signalPayload = "{ \"rssi\": " + String(rssi) + ", \"snr\": " + String(snr) + " }";
      if (!mqttClient.publish(MQTT_TOPIC_SIGNAL, signalPayload.c_str())) {
        DEBUG_INTERMEDIATE("MQTT publish failed for MBUS/signal.");
        mqttLog("MQTT publish failed for MBUS/signal.");
      } else {
        lastMQTTClientConnect = millis();
      }

      // Display sensor data and signal quality.
      displaySensorData(sensorData);
      displaySignalQuality(rssi, snr);
      displayAckStatus("ACK OK");

    } else {
      DEBUG_INTERMEDIATEF("Received packet size (%d) does not match expected size (%d).\n", packetSize, EXPECTED_MSG_SIZE);
      mqttLog("Received packet size (" + String(packetSize) + ") does not match expected (" + String(EXPECTED_MSG_SIZE) + ")");
    }
  }
}
