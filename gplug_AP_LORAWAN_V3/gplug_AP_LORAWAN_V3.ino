#include <WiFi.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>
#include <RadioLib.h>

// ----- Sensor Data Structures -----
// Pi, Po, Ei, Eo, Q5, Q6, Q7, Q8, I1, I2, I3, Ei1, Ei2, Eo1, Eo2
typedef struct {
  struct {
    float Pi;
    float Po;
    float Ei;
    float Eo;
    float Q5;
    float Q6;
    float Q7;
    float Q8;
    float I1;
    float I2;
    float I3;
    float Ei1;
    float Ei2;
    float Eo1;
    float Eo2;
  } data;
} SensorMessage;

typedef struct __attribute__((packed)) {
  struct {
    float Pi;
    float Po;
    float Ei;
    float Eo;
    float Q5;
    float Q6;
    float Q7;
    float Q8;
    float I1;
    float I2;
    float I3;
    float Ei1;
    float Ei2;
    float Eo1;
    float Eo2;
  } data;
} SensorMessageLoRaWAN;

// ----- Global variable for message counting.
volatile int messageCount = 0;

// ----- Timeout Definitions -----
#define MQTT_MSG_TIMEOUT 120000  // 2 minutes timeout for MQTT messages
// Messages before transmission
#define NUMBER_AVERAGED_MESSAGES 6

#define DEBUG_LEVEL 2
#define DEVICE 1  // 1 is productive, 2 is test

// Uncomment the following line to enable fake data injection for testing.
// #define SIMULATE_FAKE_MQTT


// ----- Three-Level Debugging Macros -----
#if DEBUG_LEVEL >= 1
#define DEBUG_ERROR(x) Serial.println(x)
#define DEBUG_ERRORF(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_ERROR(x)
#define DEBUG_ERRORF(fmt, ...)
#endif

#if DEBUG_LEVEL >= 2
#define DEBUG_INFO(x) Serial.println(x)
#define DEBUG_INFOF(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_INFO(x)
#define DEBUG_INFOF(fmt, ...)
#endif

#if DEBUG_LEVEL >= 3
#define DEBUG_DEBUG(x) Serial.println(x)
#define DEBUG_DEBUGF(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_DEBUG(x)
#define DEBUG_DEBUGF(fmt, ...)
#endif

// ----- LoRa (now LoRaWAN) Settings -----
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_CS 18
#define LORA_RST 23
#define LORA_DIO0 26
#define LORA_DIO1 33

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

// ----- Global Variables for Watchdog -----
unsigned long lastMQTTMessageTime = 0;
unsigned long lastMQTTClientConnect = 0;

// ----- RTOS Task Handles -----
TaskHandle_t Task1_MQTT_Handle = NULL;  // MQTT broker & OLED task
TaskHandle_t Task2_LoRa_Handle = NULL;  // LoRaWAN transmission task
TaskHandle_t Watchdog_Handle = NULL;    // Watchdog task

// ----- Inter-Task Communication -----
QueueHandle_t sensorMsgQueue;

// This function returns a SensorMessage with stable fake data between 1 and 100000.
SensorMessage generateFakeSensorMessage() {
  SensorMessage msg = {};
  msg.data.Pi = 12345;  // Example stable value
  msg.data.Po = 23456;
  msg.data.Ei = 34567;
  msg.data.Eo = 45678;
  msg.data.Q5 = 56789;
  msg.data.Q6 = 67890;
  msg.data.Q7 = 78901;
  msg.data.Q8 = 89012;
  msg.data.I1 = 90123;
  msg.data.I2 = 81234;
  msg.data.I3 = 72345;
  msg.data.Ei1 = 63456;
  msg.data.Ei2 = 54567;
  msg.data.Eo1 = 45678;
  msg.data.Eo2 = 36789;
  return msg;
}

// ----- OLED Display Helper Functions -----

// Clears the region of the specified line (assumes 10-pixel high lines) and displays the given message.
void displayStatus(const String &message, int line = 1) {
  // Clear the area for the given line
  display.fillRect(0, line * 10, SCREEN_WIDTH, 10, SSD1306_BLACK);
  display.setCursor(0, line * 10);
  display.println(message);
  display.display();
  DEBUG_DEBUG("OLED Status (line " + String(line) + "): " + message);
}

// Clears only the area reserved for the message count (line 50) and displays the updated count.
void updateMessageCountDisplay(int count) {
  // Clear only the area reserved for the message count.
  display.fillRect(0, 50, SCREEN_WIDTH, 10, SSD1306_BLACK);
  display.setCursor(0, 50);
  display.println("Msg#:" + String(count));
  display.display();
}

// Clears only the sensor data region (assumed from y=0 to y=40) and then displays the sensor values.
void displaySensorData(const SensorMessage &msg) {
  // Clear only the sensor data region.
  display.fillRect(0, 0, SCREEN_WIDTH, 40, SSD1306_BLACK);

  display.setTextSize(1);  // Use text size 1 for more lines
  display.setCursor(0, 0);

  // Display the selected sensor fields.
  display.println("Pi:" + String(msg.data.Pi, 0) + " Po:" + String(msg.data.Po, 0));
  display.println("Ei:" + String(msg.data.Ei, 0) + " Eo:" + String(msg.data.Eo, 0));
  display.println("Ei1:" + String(msg.data.Ei1, 0) + " Ei2:" + String(msg.data.Ei2, 0));
  display.println("Eo1:" + String(msg.data.Eo1, 0) + " Eo2:" + String(msg.data.Eo2, 0));

  display.display();  // Update the physical display
  DEBUG_DEBUG("Task1: OLED updated with sensor data.");
}

// Clears only the error display region (assumed to be y=40 to y=50) and displays the error message.
void displayError(const String &message) {
  display.fillRect(0, 40, SCREEN_WIDTH, 10, SSD1306_BLACK);
  display.setCursor(0, 40);
  display.println(message);
  display.display();
  DEBUG_ERRORF("Task1: Error displayed on OLED: %s\n", message.c_str());
}

// ----- MQTT Packet Processing Functions -----
bool decodeRemainingLength(uint8_t *buf, int available, int &remainingLength, int &rlBytes) {
  remainingLength = 0;
  rlBytes = 0;
  int multiplier = 1;
  uint8_t digit;
  do {
    if (rlBytes >= available || rlBytes > 4) {
      DEBUG_DEBUGF("Task1: decodeRemainingLength: Not enough bytes or too many iterations. Available: %d, rlBytes: %d\n", available, rlBytes);
      return false;
    }
    digit = buf[rlBytes++];
    remainingLength += (digit & 127) * multiplier;
    multiplier *= 128;
  } while (digit & 0x80);
  DEBUG_DEBUGF("Task1: Decoded remaining length: %d, bytes used: %d\n", remainingLength, rlBytes);
  return true;
}

SensorMessage parseSensorMessage(const String &payload) {
  SensorMessage msg = {};  // Zero-initialize all fields.

  const int MIN_VALID_PAYLOAD_LENGTH = 30;
  if (payload.length() < MIN_VALID_PAYLOAD_LENGTH) {
    DEBUG_DEBUGF("Task1: Payload too short (%d characters), skipping: %s\n", payload.length(), payload.c_str());
    return msg;
  }

  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    DEBUG_ERRORF("Task1: JSON deserialization failed: %s | Payload: %s\n", error.c_str(), payload.c_str());
    return msg;
  }
  JsonObject sensorObj = doc["z"];
  msg.data.Pi = (sensorObj["Pi"] | 0.0) * 1000;
  msg.data.Po = (sensorObj["Po"] | 0.0) * 1000;
  msg.data.Ei = sensorObj["Ei"] | 0.0;
  msg.data.Eo = sensorObj["Eo"] | 0.0;
  msg.data.Q5 = sensorObj["Q5"] | 0.0;
  msg.data.Q6 = sensorObj["Q6"] | 0.0;
  msg.data.Q7 = sensorObj["Q7"] | 0.0;
  msg.data.Q8 = sensorObj["Q8"] | 0.0;
  msg.data.I1 = sensorObj["I1"] | 0.0;
  msg.data.I2 = sensorObj["I2"] | 0.0;
  msg.data.I3 = sensorObj["I3"] | 0.0;
  msg.data.Ei1 = sensorObj["Ei1"] | 0.0;
  msg.data.Ei2 = sensorObj["Ei2"] | 0.0;
  msg.data.Eo1 = sensorObj["Eo1"] | 0.0;
  msg.data.Eo2 = sensorObj["Eo2"] | 0.0;

  // Update the last MQTT message time.
  lastMQTTMessageTime = millis();

  DEBUG_DEBUGF("Task1: MQTT message parsed: Pi=%.2f, Po=%.2f, Ei=%.2f, Eo=%.2f, Q5=%.2f, Q6=%.2f, Q7=%.2f, Q8=%.2f, I1=%.2f, I2=%.2f, I3=%.2f, Ei1=%.2f, Ei2=%.2f, Eo1=%.2f, Eo2=%.2f\n",
               msg.data.Pi, msg.data.Po, msg.data.Ei, msg.data.Eo,
               msg.data.Q5, msg.data.Q6, msg.data.Q7, msg.data.Q8,
               msg.data.I1, msg.data.I2, msg.data.I3,
               msg.data.Ei1, msg.data.Ei2, msg.data.Eo1, msg.data.Eo2);
  return msg;
}

// ----- Accumulate & Average Sensor Messages -----
void accumulateSensorMessage(const SensorMessage &msg) {
  static SensorMessage accumMsg = {};  // Zero-initialized accumulator.

  // Accumulate all sensor fields.
  accumMsg.data.Pi += msg.data.Pi;
  accumMsg.data.Po += msg.data.Po;
  accumMsg.data.Ei += msg.data.Ei;
  accumMsg.data.Eo += msg.data.Eo;
  accumMsg.data.Q5 += msg.data.Q5;
  accumMsg.data.Q6 += msg.data.Q6;
  accumMsg.data.Q7 += msg.data.Q7;
  accumMsg.data.Q8 += msg.data.Q8;
  accumMsg.data.I1 += msg.data.I1;
  accumMsg.data.I2 += msg.data.I2;
  accumMsg.data.I3 += msg.data.I3;
  accumMsg.data.Ei1 += msg.data.Ei1;
  accumMsg.data.Ei2 += msg.data.Ei2;
  accumMsg.data.Eo1 += msg.data.Eo1;
  accumMsg.data.Eo2 += msg.data.Eo2;

  messageCount++;

  // Debug: Log the full details of the current message.
  DEBUG_INFOF("Task1: Received SensorMessage #%d: Pi=%.2f, Po=%.2f, Ei=%.2f, Eo=%.2f, Q5=%.2f, Q6=%.2f, Q7=%.2f, Q8=%.2f, I1=%.2f, I2=%.2f, I3=%.2f, Ei1=%.2f, Ei2=%.2f, Eo1=%.2f, Eo2=%.2f\n",
              messageCount,
              msg.data.Pi, msg.data.Po, msg.data.Ei, msg.data.Eo,
              msg.data.Q5, msg.data.Q6, msg.data.Q7, msg.data.Q8,
              msg.data.I1, msg.data.I2, msg.data.I3,
              msg.data.Ei1, msg.data.Ei2, msg.data.Eo1, msg.data.Eo2);

  // Update the OLED message count every time a new MQTT message arrives.
  updateMessageCountDisplay(messageCount);

  // Debug: Log the current accumulated values.
  DEBUG_DEBUGF("Task1: Accumulated count: %d\n", messageCount);
  DEBUG_DEBUGF("Task1: Accumulated values: Pi=%.2f, Po=%.2f, Ei=%.2f, Eo=%.2f, Q5=%.2f, Q6=%.2f, Q7=%.2f, Q8=%.2f, I1=%.2f, I2=%.2f, I3=%.2f, Ei1=%.2f, Ei2=%.2f, Eo1=%.2f, Eo2=%.2f\n",
               accumMsg.data.Pi, accumMsg.data.Po, accumMsg.data.Ei, accumMsg.data.Eo,
               accumMsg.data.Q5, accumMsg.data.Q6, accumMsg.data.Q7, accumMsg.data.Q8,
               accumMsg.data.I1, accumMsg.data.I2, accumMsg.data.I3,
               accumMsg.data.Ei1, accumMsg.data.Ei2, accumMsg.data.Eo1, accumMsg.data.Eo2);

  if (messageCount == NUMBER_AVERAGED_MESSAGES) {
    SensorMessage avgMsg = {};  // Zero-initialize the averaged message.
    avgMsg.data.Pi = accumMsg.data.Pi / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Po = accumMsg.data.Po / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Ei = accumMsg.data.Ei / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Eo = accumMsg.data.Eo / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Q5 = accumMsg.data.Q5 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Q6 = accumMsg.data.Q6 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Q7 = accumMsg.data.Q7 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Q8 = accumMsg.data.Q8 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.I1 = accumMsg.data.I1 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.I2 = accumMsg.data.I2 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.I3 = accumMsg.data.I3 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Ei1 = accumMsg.data.Ei1 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Ei2 = accumMsg.data.Ei2 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Eo1 = accumMsg.data.Eo1 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Eo2 = accumMsg.data.Eo2 / NUMBER_AVERAGED_MESSAGES;

    DEBUG_DEBUGF("Task1: Averaged values to be sent: Pi=%.2f, Po=%.2f, Ei=%.2f, Eo=%.2f, Q5=%.2f, Q6=%.2f, Q7=%.2f, Q8=%.2f, I1=%.2f, I2=%.2f, I3=%.2f, Ei1=%.2f, Ei2=%.2f, Eo1=%.2f, Eo2=%.2f\n",
                 avgMsg.data.Pi, avgMsg.data.Po, avgMsg.data.Ei, avgMsg.data.Eo,
                 avgMsg.data.Q5, avgMsg.data.Q6, avgMsg.data.Q7, avgMsg.data.Q8,
                 avgMsg.data.I1, avgMsg.data.I2, avgMsg.data.I3,
                 avgMsg.data.Ei1, avgMsg.data.Ei2, avgMsg.data.Eo1, avgMsg.data.Eo2);

    // Send the averaged message to the queue.
    if (xQueueSend(sensorMsgQueue, &avgMsg, 0) != pdPASS) {
      DEBUG_ERROR("Task1: Error - Queue full! Averaged SensorMessage not sent.");
      displayError("Queue full!");
    } else {
      DEBUG_DEBUG("Task1: Averaged SensorMessage sent to queue.");
      // Call displaySensorData() only with the data sent via LoRaWAN (averaged message).
      displaySensorData(avgMsg);
    }
    // Reset the accumulator.
    accumMsg = {};
    messageCount = 0;
  }
}

void processBuffer_Rtos(WiFiClient &client, uint8_t *buffer, int len) {
  // Build a raw string from the entire buffer and print it (DEBUG level).
  String rawMsg = "";
  for (int i = 0; i < len; i++) {
    rawMsg += (char)buffer[i];
  }
  DEBUG_DEBUG("Task1: Raw MQTT message: " + rawMsg);

  int index = 0;
  DEBUG_DEBUGF("Task1: Processing buffer of length: %d\n", len);

  while (index < len) {
    // Ensure at least 2 bytes remain for the fixed header and remaining length.
    if (len - index < 2) {
      DEBUG_DEBUG("Task1: Error: Not enough bytes for fixed header and remaining length.");
      break;
    }

    uint8_t header = buffer[index];
    uint8_t packetType = header >> 4;
    int remainingLength = 0, rlBytes = 0;

    // Decode the remaining length field.
    if (!decodeRemainingLength(buffer + index + 1, len - (index + 1), remainingLength, rlBytes)) {
      DEBUG_DEBUG("Task1: Error: Malformed remaining length field.");
      break;
    }

    int fixedHeaderSize = 1 + rlBytes;
    int packetTotalLength = fixedHeaderSize + remainingLength;

    // Ensure the full packet is available.
    if (index + packetTotalLength > len) {
      DEBUG_DEBUGF("Task1: Incomplete packet received. Discarding remainder of buffer.\n");
      break;
    }

    if (packetType == 3 && remainingLength >= 2) {  // PUBLISH packet
      int topicLengthIndex = index + fixedHeaderSize;
      uint16_t topicLength = (buffer[topicLengthIndex] << 8) | buffer[topicLengthIndex + 1];

      if (remainingLength >= 2 + topicLength) {
        String topic = String((char *)(buffer + topicLengthIndex + 2), topicLength);
        DEBUG_DEBUG("Task1: Publish topic: " + topic);

        // Extract the portion after the last '/'
        String topicSuffix = topic.substring(topic.lastIndexOf("/") + 1);

        if (topicSuffix == "SENSOR") {
          uint8_t qos = (header & 0x06) >> 1;
          int extraBytes = (qos > 0) ? 2 : 0;
          int payloadStart = index + fixedHeaderSize + 2 + topicLength + extraBytes;
          int payloadLength = packetTotalLength - (fixedHeaderSize + 2 + topicLength + extraBytes);

          if (payloadLength >= 0) {
            String payload = String((char *)(buffer + payloadStart), payloadLength);
            // Report the raw payload and message count (expected count = messageCount+1)
            DEBUG_INFOF("Task1: Received SensorMessage #%d on topic %s: %s\n", messageCount + 1, topic.c_str(), payload.c_str());
            SensorMessage sensorMsg = parseSensorMessage(payload);
            accumulateSensorMessage(sensorMsg);
          } else {
            DEBUG_DEBUG("Task1: Negative payload length computed.");
          }
        } else if (topic == "gPlugM/STATE") {
          // Report STATE topic messages at debug level.
          uint8_t qos = (header & 0x06) >> 1;
          int extraBytes = (qos > 0) ? 2 : 0;
          int payloadStart = index + fixedHeaderSize + 2 + topicLength + extraBytes;
          int payloadLength = packetTotalLength - (fixedHeaderSize + 2 + topicLength + extraBytes);
          if (payloadLength >= 0) {
            String payload = String((char *)(buffer + payloadStart), payloadLength);
            DEBUG_DEBUG("Task1: Received STATE message: " + payload);
          }
        } else {
          DEBUG_DEBUG("Task1: Ignoring topic: " + topic);
        }
      } else {
        DEBUG_DEBUG("Task1: Remaining length less than expected topic length.");
      }
    }

    else if (packetType == 1) {  // CONNECT packet
      uint8_t connack[] = { 0x20, 0x02, 0x00, 0x00 };
      client.write(connack, sizeof(connack));
      DEBUG_INFO("Task1: Processed CONNECT packet, sent CONNACK.");
    } else if (packetType == 12) {  // PINGREQ packet
      uint8_t pingresp[] = { 0xD0, 0x00 };
      client.write(pingresp, sizeof(pingresp));
      DEBUG_DEBUG("Task1: Processed PINGREQ, sent PINGRESP.");
    }

    index += packetTotalLength;
    DEBUG_DEBUGF("Task1: Moving to next packet at index: %d\n", index);
  }
}

void processMQTTClient_Rtos(WiFiClient &client) {
  DEBUG_INFO("Task1: New MQTT client connected");
  lastMQTTClientConnect = millis();
  while (client.connected()) {
    if (client.available()) {
      uint8_t buffer[MAX_PACKET_SIZE] = { 0 };
      int len = client.readBytes(buffer, sizeof(buffer));
      if (len > 0) {
        // Convert the raw buffer to a String for display
        String rawMsg = "";
        for (int i = 0; i < len; i++) {
          rawMsg += (char)buffer[i];
        }
        DEBUG_DEBUG("Task1: MQTT raw message: " + rawMsg);
        DEBUG_DEBUGF("Task1: Received MQTT packet of length: %d\n", len);
        processBuffer_Rtos(client, buffer, len);
      } else {
        DEBUG_DEBUG("Task1: Received zero-length packet, skipping.");
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  client.stop();
  DEBUG_INFO("Task1: MQTT client disconnected");
}

// ----- LoRaWAN (RadioLib) Global Variables and Configuration -----
SX1276 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1);
const uint32_t uplinkIntervalSeconds = 5UL * 60UL;

#if DEVICE == 1
#define RADIOLIB_LORAWAN_JOIN_EUI 0x0000000000000000
#ifndef RADIOLIB_LORAWAN_DEV_EUI
#define RADIOLIB_LORAWAN_DEV_EUI 0x70B3D57ED006F35A
#endif
#ifndef RADIOLIB_LORAWAN_APP_KEY
#define RADIOLIB_LORAWAN_APP_KEY 0x30, 0xFC, 0xD1, 0x35, 0x15, 0x98, 0x62, 0x9A, 0xE9, 0x8E, 0xD9, 0xC4, 0x92, 0x28, 0x47, 0x15
#endif
#ifndef RADIOLIB_LORAWAN_NWK_KEY
#define RADIOLIB_LORAWAN_NWK_KEY 0xBF, 0xE4, 0xCE, 0x6A, 0xE1, 0xA7, 0xC1, 0xFE, 0x92, 0x66, 0x0A, 0xA3, 0xD3, 0x79, 0x65, 0xB0
#endif
#endif

#if DEVICE == 2
#define RADIOLIB_LORAWAN_JOIN_EUI 0x0000000000000000
#ifndef RADIOLIB_LORAWAN_DEV_EUI
#define RADIOLIB_LORAWAN_DEV_EUI 0x70B3D57ED006F8C6
#endif
#ifndef RADIOLIB_LORAWAN_APP_KEY
#define RADIOLIB_LORAWAN_APP_KEY 0x64, 0x3C, 0x5A, 0x25, 0xF8, 0x5A, 0x84, 0xE7, 0xFC, 0xA0, 0xE6, 0xEE, 0x8E, 0xF1, 0xEB, 0x5A
#endif
#ifndef RADIOLIB_LORAWAN_NWK_KEY
#define RADIOLIB_LORAWAN_NWK_KEY 0xBE, 0xE7, 0xA0, 0x57, 0x98, 0x66, 0xDD, 0x45, 0x18, 0x40, 0x59, 0x4D, 0x67, 0x0C, 0xAB, 0xCC
#endif
#endif

const LoRaWANBand_t Region = EU868;
const uint8_t subBand = 0;
uint64_t joinEUI = RADIOLIB_LORAWAN_JOIN_EUI;
uint64_t devEUI = RADIOLIB_LORAWAN_DEV_EUI;
uint8_t appKey[] = { RADIOLIB_LORAWAN_APP_KEY };
uint8_t nwkKey[] = { RADIOLIB_LORAWAN_NWK_KEY };

LoRaWANNode node(&radio, &Region, subBand);

String stateDecode(const int16_t result) {
  switch (result) {
    case RADIOLIB_ERR_NONE: return "ERR_NONE";
    case RADIOLIB_ERR_CHIP_NOT_FOUND: return "ERR_CHIP_NOT_FOUND";
    case RADIOLIB_ERR_PACKET_TOO_LONG: return "ERR_PACKET_TOO_LONG";
    case RADIOLIB_ERR_RX_TIMEOUT: return "ERR_RX_TIMEOUT";
    case RADIOLIB_ERR_CRC_MISMATCH: return "ERR_CRC_MISMATCH";
    case RADIOLIB_ERR_INVALID_BANDWIDTH: return "ERR_INVALID_BANDWIDTH";
    case RADIOLIB_ERR_INVALID_SPREADING_FACTOR: return "ERR_INVALID_SPREADING_FACTOR";
    case RADIOLIB_ERR_INVALID_CODING_RATE: return "ERR_INVALID_CODING_RATE";
    case RADIOLIB_ERR_INVALID_FREQUENCY: return "ERR_INVALID_FREQUENCY";
    case RADIOLIB_ERR_INVALID_OUTPUT_POWER: return "ERR_INVALID_OUTPUT_POWER";
    case RADIOLIB_ERR_NETWORK_NOT_JOINED: return "RADIOLIB_ERR_NETWORK_NOT_JOINED";
    case RADIOLIB_ERR_DOWNLINK_MALFORMED: return "RADIOLIB_ERR_DOWNLINK_MALFORMED";
    case RADIOLIB_ERR_INVALID_REVISION: return "RADIOLIB_ERR_INVALID_REVISION";
    case RADIOLIB_ERR_INVALID_PORT: return "RADIOLIB_ERR_INVALID_PORT";
    case RADIOLIB_ERR_NO_RX_WINDOW: return "RADIOLIB_ERR_NO_RX_WINDOW";
    case RADIOLIB_ERR_INVALID_CID: return "RADIOLIB_ERR_INVALID_CID";
    case RADIOLIB_ERR_UPLINK_UNAVAILABLE: return "RADIOLIB_ERR_UPLINK_UNAVAILABLE";
    case RADIOLIB_ERR_COMMAND_QUEUE_FULL: return "RADIOLIB_ERR_COMMAND_QUEUE_FULL";
    case RADIOLIB_ERR_COMMAND_QUEUE_ITEM_NOT_FOUND: return "RADIOLIB_ERR_COMMAND_QUEUE_ITEM_NOT_FOUND";
    case RADIOLIB_ERR_JOIN_NONCE_INVALID: return "RADIOLIB_ERR_JOIN_NONCE_INVALID";
    case RADIOLIB_ERR_N_FCNT_DOWN_INVALID: return "RADIOLIB_ERR_N_FCNT_DOWN_INVALID";
    case RADIOLIB_ERR_A_FCNT_DOWN_INVALID: return "RADIOLIB_ERR_A_FCNT_DOWN_INVALID";
    case RADIOLIB_ERR_DWELL_TIME_EXCEEDED: return "RADIOLIB_ERR_DWELL_TIME_EXCEEDED";
    case RADIOLIB_ERR_CHECKSUM_MISMATCH: return "RADIOLIB_ERR_CHECKSUM_MISMATCH";
    case RADIOLIB_ERR_NO_JOIN_ACCEPT: return "RADIOLIB_ERR_NO_JOIN_ACCEPT";
    case RADIOLIB_LORAWAN_SESSION_RESTORED: return "RADIOLIB_LORAWAN_SESSION_RESTORED";
    case RADIOLIB_LORAWAN_NEW_SESSION: return "RADIOLIB_LORAWAN_NEW_SESSION";
    case RADIOLIB_ERR_NONCES_DISCARDED: return "RADIOLIB_ERR_NONCES_DISCARDED";
    case RADIOLIB_ERR_SESSION_DISCARDED: return "RADIOLIB_ERR_SESSION_DISCARDED";
    default: return "Unknown error";
  }
}

void debug(bool failed, const __FlashStringHelper *message, int state, bool halt) {
  if (failed) {
    Serial.print("Task2: ");
    Serial.print(message);
    Serial.print(" - ");
    Serial.print(stateDecode(state));
    Serial.print(" (");
    Serial.print(state);
    Serial.println(")");
    while (halt) { delay(1); }
  }
}

void sendSensorMessageLoRaWAN(const SensorMessage &msg) {
  DEBUG_DEBUGF("Task2: SensorMessage before packing: Pi=%.2f, Po=%.2f, Ei=%.2f, Eo=%.2f, Q5=%.2f, Q6=%.2f, Q7=%.2f, Q8=%.2f, I1=%.2f, I2=%.2f, I3=%.2f, Ei1=%.2f, Ei2=%.2f, Eo1=%.2f, Eo2=%.2f\n",
               msg.data.Pi, msg.data.Po, msg.data.Ei, msg.data.Eo,
               msg.data.Q5, msg.data.Q6, msg.data.Q7, msg.data.Q8,
               msg.data.I1, msg.data.I2, msg.data.I3,
               msg.data.Ei1, msg.data.Ei2, msg.data.Eo1, msg.data.Eo2);

  // Pack the values into a LoRaWAN packet.
  SensorMessageLoRaWAN packed;
  packed.data.Pi = msg.data.Pi;
  packed.data.Po = msg.data.Po;
  packed.data.Ei = msg.data.Ei;
  packed.data.Eo = msg.data.Eo;
  packed.data.Q5 = msg.data.Q5;
  packed.data.Q6 = msg.data.Q6;
  packed.data.Q7 = msg.data.Q7;
  packed.data.Q8 = msg.data.Q8;
  packed.data.I1 = msg.data.I1;
  packed.data.I2 = msg.data.I2;
  packed.data.I3 = msg.data.I3;
  packed.data.Ei1 = msg.data.Ei1;
  packed.data.Ei2 = msg.data.Ei2;
  packed.data.Eo1 = msg.data.Eo1;
  packed.data.Eo2 = msg.data.Eo2;

  DEBUG_INFOF("Task2: Sending LoRaWAN packet with values: Pi=%.2f, Po=%.2f, Ei=%.2f, Eo=%.2f, Q5=%.2f, Q6=%.2f, Q7=%.2f, Q8=%.2f, I1=%.2f, I2=%.2f, I3=%.2f, Ei1=%.2f, Ei2=%.2f, Eo1=%.2f, Eo2=%.2f\n",
              packed.data.Pi, packed.data.Po, packed.data.Ei, packed.data.Eo,
              packed.data.Q5, packed.data.Q6, packed.data.Q7, packed.data.Q8,
              packed.data.I1, packed.data.I2, packed.data.I3,
              packed.data.Ei1, packed.data.Ei2, packed.data.Eo1, packed.data.Eo2);

  int16_t state = node.sendReceive((uint8_t *)&packed, sizeof(packed));
  int16_t rssi = radio.getRSSI();
  String rssiMessage = "RSSI: " + String(rssi);
  displayStatus(rssiMessage, 4);
  debug(state < RADIOLIB_ERR_NONE, F("Error in sendReceive"), state, false);
  if (state > 0) {
    Serial.println(F("Task2: Downlink received"));
  } else {
    DEBUG_DEBUG("Task2: No downlink received");
  }
  DEBUG_DEBUG("Task2: LoRa message sent");
}

void WatchdogTask(void *parameter) {
  for (;;) {
    unsigned long now = millis();
    DEBUG_DEBUGF("Watchdog: Checking MQTT message timeout... %lu ms\n", now - lastMQTTMessageTime);
    if (now - lastMQTTMessageTime > MQTT_MSG_TIMEOUT) {
      // Critical error: no MQTT message received in timeout period.
      DEBUG_ERROR("Task1: Critical error - No MQTT message received in 2 minutes");
      displayError("No MQTT msg!");
      vTaskDelay(3000);
      ESP.restart();
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

bool initializeHardware() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    DEBUG_ERROR("Task1: SSD1306 allocation failed");
    return false;
  }
  display.clearDisplay();
  display.display();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("OLED init OK!");
  display.display();
  DEBUG_INFO("Task1: OLED initialized successfully.");

  IPAddress local_ip(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  WiFi.softAP(ssid, password);
  DEBUG_INFOF("Task1: Access Point started, IP: %s\n", WiFi.softAPIP().toString().c_str());
  displayStatus("WiFi started!", 1);

  mqttServer.begin();
  DEBUG_INFO("Task1: MQTT Broker started on port 1883");

  return true;
}

#ifdef SIMULATE_FAKE_MQTT
void Task1_MQTT(void *parameter) {
  for (;;) {
    // Generate a fake sensor message with stable data.
    SensorMessage fakeMsg = generateFakeSensorMessage();
    accumulateSensorMessage(fakeMsg);
    // You can adjust the delay to simulate message frequency.
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Inject every second
  }
}
#else
void Task1_MQTT(void *parameter) {
  for (;;) {
    WiFiClient client = mqttServer.available();
    if (client) {
      DEBUG_INFO("Task1: MQTT client available, processing connection...");
      displayStatus("MQTT broker connected", 3);
      processMQTTClient_Rtos(client);
      lastMQTTClientConnect = millis();
      DEBUG_INFO("Task1: Updated lastMQTTClientConnect after client processing.");
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
#endif


void Task2_LoRa(void *parameter) {
  Serial.println(F("Task2: Initializing LoRa radio"));
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  int16_t state = radio.begin();
  debug(state != RADIOLIB_ERR_NONE, F("Radio initialization failed"), state, false);

  Serial.println(F("Task2: Setting up OTAA session"));
  state = node.beginOTAA(joinEUI, devEUI, nwkKey, appKey);
  debug(state != RADIOLIB_ERR_NONE, F("Node initialization failed"), state, false);

  Serial.println(F("Task2: Joining the LoRaWAN Network"));
  while (true) {
    state = node.activateOTAA();
    if (state == RADIOLIB_LORAWAN_NEW_SESSION) {
      int16_t rssi = radio.getRSSI();
      Serial.print(F("Task2: LoRaWAN Joined Successfully! RSSI: "));
      Serial.println(rssi);
      displayStatus("Joined! RSSI: " + String(rssi), 2);
      break;
    } else {
      Serial.print(F("Task2: LoRaWAN join failed with state "));
      Serial.println(state);
      displayStatus("LoRa join error!", 2);
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
  }

  SensorMessage sensorMsgForLoRa;
  for (;;) {
    if (xQueueReceive(sensorMsgQueue, &sensorMsgForLoRa, portMAX_DELAY) == pdPASS) {
      DEBUG_DEBUG("Task2: Received SensorMessage from queue, transmitting via LoRaWAN.");
      sendSensorMessageLoRaWAN(sensorMsgForLoRa);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("Starting ESP32 RTOS sketch with MQTT, LoRaWAN, and Watchdog tasks...");
  Serial.println("---------------------------------------------------------------");
  Serial.print("File: ");
  Serial.println(__FILE__);
  Serial.println("---------------------------------------------------------------");

  lastMQTTClientConnect = millis();
  lastMQTTMessageTime = millis();

  if (!initializeHardware()) {
    Serial.println("Task1: Hardware initialization failed. Halting.");
    while (true) { vTaskDelay(1000 / portTICK_PERIOD_MS); }
  }

  sensorMsgQueue = xQueueCreate(10, sizeof(SensorMessage));
  if (sensorMsgQueue == NULL) {
    DEBUG_ERROR("Task1: Failed to create sensor message queue.");
    while (true) { vTaskDelay(1000 / portTICK_PERIOD_MS); }
  }

  xTaskCreate(Task1_MQTT, "Task1_MQTT", 8192, NULL, 1, &Task1_MQTT_Handle);
  xTaskCreate(Task2_LoRa, "Task2_LoRa", 16384, NULL, 1, &Task2_LoRa_Handle);
  xTaskCreate(WatchdogTask, "WatchdogTask", 4096, NULL, 1, &Watchdog_Handle);
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
