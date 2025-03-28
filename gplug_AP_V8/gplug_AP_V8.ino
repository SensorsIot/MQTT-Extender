#include <WiFi.h>
#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>
#include "MBUS_data_structure.h"  // Contains SensorMessage and SensorMessagePacked

// ----- Timeout Definitions -----
// MQTT Message Timeout: If no MQTT message is received for 60000 ms (1 minute), reboot.
#define MQTT_MSG_TIMEOUT 60000
// ACK Watchdog Timeout: If no ACK is received for 60000 ms (1 minute), reboot.
#define ACK_WD_TIMEOUT 60000
// ACK Wait Timeout for each sendWithAck trial (used in sendWithAck below).
#define ACK_TIMEOUT 2000  // 2000 ms as before
// Maximum retransmission attempts for LoRa message.
#define MAX_RETRIES 3

// ----- Three-Level Debugging Macros -----
// Set the debug level at compile time (1 = simple, 2 = intermediate, 3 = extended)
#ifndef DEBUG_LEVEL
#define DEBUG_LEVEL 1
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

// ----- LoRa Settings -----
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_CS 18
#define LORA_RST 23
#define LORA_DIO0 26
#define LORA_DIO1 33

// Macro to get only the file name from __FILE__
#define __SHORT_FILE__ ((strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)))

// ----- OLED Display Settings -----
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 64  // OLED display height, in pixels
#define OLED_RESET -1     // Reset pin (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ----- WiFi & MQTT Server Settings -----
const char *ssid = "ESP32_AP";
const char *password = "esp32password";
WiFiServer mqttServer(1883);  // Minimal MQTT broker on port 1883
const size_t MAX_PACKET_SIZE = 1024;
#define MQTT_CONNECT_TIMEOUT 30000  // 30-second timeout if no client connects

// ----- ACK Settings for LoRa Transmission -----
// (ACK_TIMEOUT and MAX_RETRIES already defined above)
// Expected ACK byte from the gateway:
#define ACK_BYTE 0x55

// ----- Global Variables for Watchdog -----
// Record the last time a valid MQTT message was received.
unsigned long lastMQTTMessageTime = 0;
// Record the last time an ACK was received.
unsigned long lastAckTime = 0;

// ----- Global Variable for MQTT Client Connection -----
// (Used in Task1_MQTT to reboot if no client connects)
unsigned long lastMQTTClientConnect = 0;

// ----- RTOS Task Handles -----
TaskHandle_t Task1_MQTT_Handle = NULL;  // MQTT broker & OLED task
TaskHandle_t Task2_LoRa_Handle = NULL;  // LoRa transmission task
TaskHandle_t Watchdog_Handle = NULL;    // Watchdog task

// ----- Inter-Task Communication -----
// Queue for transferring SensorMessage from Task 1 to Task 2.
QueueHandle_t sensorMsgQueue;

// ----- OLED Display Helper Functions -----
// displaySensorData() prints sensor values from the SensorMessage.
// Line 1: "Pi:" and "Po:" values
// Line 2: "i1:", "i2:", "i3:" values
// Line 3: "o1:", "o2:", "o3:" values
// Line 4: "RSSI:" and "SNR:" values (defaults to 0 if not provided)
// Line 5: "MBUS-AP"

void displaySensorData(const SensorMessage &msg, int rssi = 0, float snr = 0.0) {
  display.clearDisplay();

  // Line 1: Sensor summary (Pi and Po)
  display.setCursor(0, 0);
  display.println("Pi:" + String(msg.data.Pi, 2) + " Po:" + String(msg.data.Po, 2));

  // Line 2: Sensor details (i1, i2, i3)
  display.setCursor(0, 10);
  display.println("i1:" + String(msg.data.Pi1, 1) + " i2:" + String(msg.data.Pi2, 1) + " i3:" + String(msg.data.Pi3, 1));

  // Line 3: Sensor details (o1, o2, o3)
  display.setCursor(0, 20);
  display.println("o1:" + String(msg.data.Po1, 1) + " o2:" + String(msg.data.Po2, 1) + " o3:" + String(msg.data.Po3, 1));

  // Line 4: RSSI and SNR values
  display.setCursor(0, 30);
  display.print("RSSI: ");
  display.print(rssi);
  display.print(" SNR: ");
  display.println(snr);

  // Line 5: Display static text "MBUS-AP" (centered horizontally)
  display.setCursor(35, 50);
  display.println("MBUS-AP");

  display.display();
  DEBUG_EXTENDED("OLED updated with sensor data, RSSI, SNR, and MBUS-AP.");
}

// Displays error messages on OLED (line 40).
void displayError(const String &message) {
  display.fillRect(0, 40, SCREEN_WIDTH, 10, SSD1306_BLACK);
  display.setCursor(0, 40);
  display.println(message);
  display.display();
  DEBUG_SIMPLE("Error displayed on OLED: " + message);
}

// ----- MQTT Packet Processing Functions -----
// Decodes the MQTT remaining length field.
bool decodeRemainingLength(uint8_t *buf, int available, int &remainingLength, int &rlBytes) {
  remainingLength = 0;
  rlBytes = 0;
  int multiplier = 1;
  uint8_t digit;
  do {
    if (rlBytes >= available || rlBytes > 4) {
      DEBUG_EXTENDED("decodeRemainingLength: Not enough bytes or too many iterations.");
      return false;
    }
    digit = buf[rlBytes++];
    remainingLength += (digit & 127) * multiplier;
    multiplier *= 128;
  } while (digit & 0x80);
  DEBUG_EXTENDEDF("Decoded remaining length: %d, bytes used: %d\n", remainingLength, rlBytes);
  return true;
}

// Parses a JSON payload into a SensorMessage structure.
SensorMessage parseSensorMessage(const String &payload) {
  SensorMessage msg;
  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    DEBUG_SIMPLE("JSON deserialization failed: " + String(error.c_str()));
    return msg;
  }
  JsonObject sensorObj = doc["z"];
  msg.data.Pi = sensorObj["Pi"] | 0.0;
  msg.data.Po = sensorObj["Po"] | 0.0;
  msg.data.Pi1 = sensorObj["Pi1"] | 0.0;
  msg.data.Pi2 = sensorObj["Pi2"] | 0.0;
  msg.data.Pi3 = sensorObj["Pi3"] | 0.0;
  msg.data.Po1 = sensorObj["Po1"] | 0.0;
  msg.data.Po2 = sensorObj["Po2"] | 0.0;
  msg.data.Po3 = sensorObj["Po3"] | 0.0;
  msg.data.U1 = sensorObj["U1"] | 0.0;
  msg.data.U2 = sensorObj["U2"] | 0.0;
  msg.data.U3 = sensorObj["U3"] | 0.0;
  msg.data.I1 = sensorObj["I1"] | 0.0;
  msg.data.I2 = sensorObj["I2"] | 0.0;
  msg.data.I3 = sensorObj["I3"] | 0.0;
  msg.data.Ei = sensorObj["Ei"] | 0.0;
  msg.data.Eo = sensorObj["Eo"] | 0.0;
  msg.data.Ei1 = sensorObj["Ei1"] | 0.0;
  msg.data.Ei2 = sensorObj["Ei2"] | 0.0;
  msg.data.Eo1 = sensorObj["Eo1"] | 0.0;
  msg.data.Eo2 = sensorObj["Eo2"] | 0.0;
  msg.data.Q5 = sensorObj["Q5"] | 0.0;
  msg.data.Q6 = sensorObj["Q6"] | 0.0;
  msg.data.Q7 = sensorObj["Q7"] | 0.0;
  msg.data.Q8 = sensorObj["Q8"] | 0.0;
  DEBUG_INTERMEDIATE("JSON parsed into SensorMessage.");
  // Update the MQTT message watchdog timer.
  lastMQTTMessageTime = millis();
  return msg;
}

// Processes the received MQTT buffer (RTOS-adapted).
void processBuffer_Rtos(WiFiClient &client, uint8_t *buffer, int len) {
  int index = 0;
  DEBUG_EXTENDEDF("Processing buffer of length: %d\n", len);
  while (index < len) {
    if (len - index < 2) {
      DEBUG_EXTENDED("Error: Not enough bytes for fixed header and remaining length");
      break;
    }
    uint8_t header = buffer[index];
    uint8_t packetType = header >> 4;
    int remainingLength = 0;
    int rlBytes = 0;
    if (!decodeRemainingLength(buffer + index + 1, len - (index + 1), remainingLength, rlBytes)) {
      DEBUG_EXTENDED("Error: Malformed remaining length field");
      break;
    }
    int fixedHeaderSize = 1 + rlBytes;
    int packetTotalLength = fixedHeaderSize + remainingLength;
    if (index + packetTotalLength > len) {
      DEBUG_EXTENDED("Error: Incomplete packet received");
      break;
    }
    // Process PUBLISH packets (packet type 3)
    if (packetType == 3 && remainingLength >= 2) {
      int topicLengthIndex = index + fixedHeaderSize;
      uint16_t topicLength = (buffer[topicLengthIndex] << 8) | buffer[topicLengthIndex + 1];
      if (remainingLength >= 2 + topicLength) {
        String topic = String((char *)(buffer + topicLengthIndex + 2), topicLength);
        DEBUG_SIMPLE("Publish topic: " + topic);
        DEBUG_INTERMEDIATEF("Packet length: %d\n", packetTotalLength);
        uint8_t qos = (header & 0x06) >> 1;
        int extraBytes = (qos > 0) ? 2 : 0;
        int payloadStart = index + fixedHeaderSize + 2 + topicLength + extraBytes;
        int payloadLength = packetTotalLength - (fixedHeaderSize + 2 + topicLength + extraBytes);
        if (payloadLength >= 0) {
          String payload = String((char *)(buffer + payloadStart), payloadLength);
          DEBUG_SIMPLE("Payload: " + payload);
          // Parse the JSON payload into a SensorMessage.
          SensorMessage sensorMsg = parseSensorMessage(payload);
          // Send the SensorMessage to Task2 via the queue.
          if (xQueueSend(sensorMsgQueue, &sensorMsg, 0) != pdPASS) {
            DEBUG_EXTENDED("Failed to send SensorMessage to queue.");
            displayError("Queue full!");
          } else {
            DEBUG_INTERMEDIATE("SensorMessage sent to queue.");
            // Immediately update OLED with MQTT data (using default RSSI and SNR values).
            displaySensorData(sensorMsg);
          }
        }
      }
    }
    // Handle CONNECT packets (packet type 1)
    else if (packetType == 1) {
      uint8_t connack[] = { 0x20, 0x02, 0x00, 0x00 };
      client.write(connack, sizeof(connack));
      DEBUG_SIMPLE("Processed CONNECT packet, sent CONNACK.");
    }
    // Handle PINGREQ packets (packet type 12)
    else if (packetType == 12) {
      uint8_t pingresp[] = { 0xD0, 0x00 };
      client.write(pingresp, sizeof(pingresp));
      DEBUG_SIMPLE("Processed PINGREQ, sent PINGRESP.");
    }
    index += packetTotalLength;
    DEBUG_EXTENDEDF("Moving to next packet at index: %d\n", index);
  }
}

// Processes an MQTT client connection using the RTOS-adapted functions.
void processMQTTClient_Rtos(WiFiClient &client) {
  DEBUG_SIMPLE("New MQTT client connected");
  lastMQTTClientConnect = millis();  // Update connection time
  while (client.connected()) {
    if (client.available()) {
      uint8_t buffer[MAX_PACKET_SIZE] = { 0 };
      int len = client.readBytes(buffer, sizeof(buffer));
      if (len > 0) {
        DEBUG_INTERMEDIATEF("Received MQTT packet of length: %d\n", len);
        processBuffer_Rtos(client, buffer, len);
      } else {
        DEBUG_EXTENDED("Received zero-length packet, skipping.");
      }
    }
    delay(10);  // Short delay to yield
  }
  client.stop();
  DEBUG_SIMPLE("MQTT client disconnected");
}

// ----- CRC16 Calculation Function -----
// Implements the CRC-CCITT algorithm (polynomial 0x1021, initial value 0xFFFF)
uint16_t crc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; i++) {
    crc ^= ((uint16_t)data[i]) << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }
  return crc;
}

// Sends sensor message over LoRa with CRC16 checksum.
void sendSensorMessageLoRaBinary(const SensorMessage &msg) {
  SensorMessagePacked packed;
  // Copy sensor data into the packed structure.
  packed.data.Pi = msg.data.Pi;
  packed.data.Po = msg.data.Po;
  packed.data.Pi1 = msg.data.Pi1;
  packed.data.Pi2 = msg.data.Pi2;
  packed.data.Pi3 = msg.data.Pi3;
  packed.data.Po1 = msg.data.Po1;
  packed.data.Po2 = msg.data.Po2;
  packed.data.Po3 = msg.data.Po3;
  packed.data.I1 = msg.data.I1;
  packed.data.I2 = msg.data.I2;
  packed.data.I3 = msg.data.I3;
  packed.data.Ei = msg.data.Ei;
  packed.data.Eo = msg.data.Eo;
  packed.data.Ei1 = msg.data.Ei1;
  packed.data.Ei2 = msg.data.Ei2;
  packed.data.Eo1 = msg.data.Eo1;
  packed.data.Eo2 = msg.data.Eo2;
  packed.data.Q5 = msg.data.Q5;
  packed.data.Q6 = msg.data.Q6;
  packed.data.Q7 = msg.data.Q7;
  packed.data.Q8 = msg.data.Q8;

  // Calculate CRC16 over the sensor data (excluding the crc16 field).
  uint8_t *dataBytes = (uint8_t *)&packed.data;
  size_t dataSize = sizeof(packed.data) - sizeof(packed.data.parity);
  uint16_t crc = crc16(dataBytes, dataSize);
  packed.data.parity = crc;

  // Transmit the packed structure over LoRa.
  LoRa.beginPacket();
  LoRa.write((uint8_t *)&packed, sizeof(packed));
  LoRa.endPacket();
  DEBUG_SIMPLEF("LoRa binary message with CRC16 sent, size: %d\n", sizeof(packed));
}

// ----- sendWithAck() Function -----
// Sends a sensor message via LoRa and waits for an ACK.
// Retransmits up to MAX_RETRIES if no ACK is received.
bool sendWithAck(const SensorMessage &msg) {
  int retries = MAX_RETRIES;
  bool ackReceived = false;
  while (retries > 0 && !ackReceived) {
    sendSensorMessageLoRaBinary(msg);
    DEBUG_SIMPLE("LoRa message sent. Waiting for ACK...");
    // Put LoRa into receive mode.
    LoRa.receive();
    unsigned long startTime = millis();
    bool ackThisTrial = false;
    while (millis() - startTime < ACK_TIMEOUT) {
      int packetSize = LoRa.parsePacket();
      if (packetSize > 0) {
        uint8_t ackBuffer[packetSize];
        int bytesRead = LoRa.readBytes(ackBuffer, packetSize);
        if (bytesRead > 0 && ackBuffer[0] == ACK_BYTE) {
          // Read RSSI and SNR values from the LoRa module.
          int rssi = LoRa.packetRssi();
          float snr = LoRa.packetSnr();
          DEBUG_SIMPLE("ACK received.");
          // Update the ACK watchdog timer.
          lastAckTime = millis();
          // Display sensor data along with the actual RSSI and SNR values.
          displaySensorData(msg, rssi, snr);
          ackReceived = true;
          ackThisTrial = true;
          break;
        }
      }
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    if (!ackThisTrial) {
      DEBUG_SIMPLE("No ACK received, retransmitting...");
      retries--;
    }
  }
  if (!ackReceived) {
    DEBUG_SIMPLE("Message not acknowledged after maximum retries.");
  }
  return ackReceived;
}

// ----- Watchdog Task -----
// Monitors the time since the last MQTT message and the last ACK.
// Reboots the ESP32 if either exceeds the defined timeouts.
void WatchdogTask(void *parameter) {
  for (;;) {
    unsigned long now = millis();
    if (now - lastMQTTMessageTime > MQTT_MSG_TIMEOUT) {
      rebootSystem("No MQTT message received in 1 minute");
    }
    if (now - lastAckTime > ACK_WD_TIMEOUT) {
      rebootSystem("No ACK received in 1 minute");
    }
    // Check every 5 seconds.
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

// ----- Hardware Initialization -----
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
  DEBUG_INTERMEDIATE("OLED initialized successfully.");

  // Configure the ESP32 as an Access Point.
  IPAddress local_ip(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  WiFi.softAP(ssid, password);
  DEBUG_SIMPLEF("Access Point started, IP: %s", WiFi.softAPIP().toString().c_str());

  // Start the MQTT server.
  mqttServer.begin();
  DEBUG_SIMPLE("MQTT Broker started on port 1883");

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
  DEBUG_SIMPLE("LoRa init OK!");
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("LoRa init OK!");
  display.display();
  return true;
}

void rebootSystem(String reason) {
  DEBUG_SIMPLE("Critical error: " + reason);
  displayError(reason);
  delay(3000);
  ESP.restart();
}

// ----- RTOS Tasks -----
// Task 1: MQTT Broker and OLED update task.
// Listens for MQTT clients, processes packets, and sends parsed SensorMessage structures via a queue.
void Task1_MQTT(void *parameter) {
  for (;;) {
    WiFiClient client = mqttServer.available();
    if (client) {
      DEBUG_SIMPLE("MQTT client available, processing connection...");
      processMQTTClient_Rtos(client);
      lastMQTTClientConnect = millis();
      DEBUG_EXTENDED("Updated lastMQTTClientConnect after client processing.");
    }
    // Reboot if no client has connected within the defined timeout.
    if (millis() - lastMQTTClientConnect > MQTT_CONNECT_TIMEOUT) {
      rebootSystem("No MQTT client connected within 30 seconds");
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Yield to other tasks.
  }
}

// Task 2: LoRa Transmission Task.
// Waits for a SensorMessage from Task 1 via the queue and sends it over LoRa using sendWithAck.
void Task2_LoRa(void *parameter) {
  SensorMessage sensorMsg;
  for (;;) {
    if (xQueueReceive(sensorMsgQueue, &sensorMsg, portMAX_DELAY) == pdPASS) {
      DEBUG_SIMPLE("Received SensorMessage from queue, transmitting via LoRa.");
      bool ackOk = sendWithAck(sensorMsg);
      if (ackOk) {
        DEBUG_INTERMEDIATE("LoRa transmission completed successfully.");
      } else {
        DEBUG_SIMPLE("LoRa transmission failed after retries.");
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);  // Yield to other tasks.
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;  // Wait for Serial to initialize
  }
  Serial.println("Starting ESP32 RTOS sketch with MQTT, LoRa, and Watchdog tasks with extended debug...");
  Serial.println();
  Serial.println("---------------------------------------------------------------");

  Serial.print("File: ");
  Serial.println(__SHORT_FILE__);
  Serial.println("---------------------------------------------------------------");
  Serial.println();
  lastMQTTClientConnect = millis();
  lastMQTTMessageTime = millis();  // Initialize watchdog timers.
  lastAckTime = millis();

  if (!initializeHardware()) {
    Serial.println("Hardware initialization failed. Halting.");
    while (true) {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }

  // Create a queue for SensorMessage transfers between tasks.
  sensorMsgQueue = xQueueCreate(10, sizeof(SensorMessage));
  if (sensorMsgQueue == NULL) {
    Serial.println("Failed to create sensor message queue.");
    while (true) {
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }

  // Create RTOS Task 1: MQTT Broker and OLED update.
  xTaskCreate(Task1_MQTT, "Task1_MQTT", 8192, NULL, 1, &Task1_MQTT_Handle);

  // Create RTOS Task 2: LoRa Transmission.
  xTaskCreate(Task2_LoRa, "Task2_LoRa", 8192, NULL, 1, &Task2_LoRa_Handle);

  // Create the Watchdog Task.
  xTaskCreate(WatchdogTask, "WatchdogTask", 4096, NULL, 1, &Watchdog_Handle);
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);  // Idle loop
}
