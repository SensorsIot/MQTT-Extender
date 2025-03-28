#include <WiFi.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoJson.h>
#include "MBUS_data_structure.h"  // Contains SensorMessage and SensorMessagePacked
#include <RadioLib.h>             // LoRaWAN library

// Global variable for message counting.
volatile int messageCount = 0;

// ----- Timeout Definitions -----
#define MQTT_MSG_TIMEOUT 120000  // 1 minute timeout for MQTT messages
// Messages before transmission
#define NUMBER_AVERAGED_MESSAGES 12

// ----- Three-Level Debugging Macros -----
#define DEBUG_LEVEL 2

#if DEBUG_LEVEL >= 1
#define DEBUG_SIMPLE(x) Serial.println(x)
#define DEBUG_SIMPLEF(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_SIMPLE(x)
#define DEBUG_SIMPLEF(fmt, ...)
#endif

// The intermediate debug macros are reserved for two new messages only.
#if DEBUG_LEVEL >= 2
#define DEBUG_INTERMEDIATE(x) Serial.println(x)
#define DEBUG_INTERMEDIATEF(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_INTERMEDIATE(x)
#define DEBUG_INTERMEDIATEF(fmt, ...)
#endif

#if DEBUG_LEVEL >= 3
#define DEBUG_EXTENDED(x) Serial.println(x)
#define DEBUG_EXTENDEDF(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_EXTENDED(x)
#define DEBUG_EXTENDEDF(fmt, ...)
#endif

// ----- LoRa (now LoRaWAN) Settings -----
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_CS 18
#define LORA_RST 23
// Note: Removed duplicate definition of LORA_RST (was defined twice)
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

// ----- OLED Display Helper Functions -----
void displayStatus(const String &message, int line = 1) {
  display.clearDisplay();
  display.setCursor(0, line * 10);
  display.println(message);
  display.display();
  DEBUG_SIMPLE("OLED Status (line " + String(line) + "): " + message);
}


void displaySensorData(const SensorMessage &msg) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Pi:" + String(msg.data.Pi, 2) + " Po:" + String(msg.data.Po, 2));
  display.setCursor(0, 10);
  display.println("i1:" + String(msg.data.Pi1, 1) + " i2:" + String(msg.data.Pi2, 1) + " i3:" + String(msg.data.Pi3, 1));
  display.setCursor(0, 20);
  display.println("o1:" + String(msg.data.Po1, 1) + " o2:" + String(msg.data.Po2, 1) + " o3:" + String(msg.data.Po3, 1));
  display.setCursor(0, 30);  // Position corresponding to line 4 (adjust as needed)
  display.println("Message#: " + String(messageCount));
  display.display();
  DEBUG_EXTENDED("Task1: OLED updated with sensor data.");
}

void displayError(const String &message) {
  display.fillRect(0, 40, SCREEN_WIDTH, 10, SSD1306_BLACK);
  display.setCursor(0, 40);
  display.println(message);
  display.display();
  DEBUG_SIMPLEF("Task1: Error displayed on OLED: %s\n", message.c_str());
}

// ----- MQTT Packet Processing Functions -----
bool decodeRemainingLength(uint8_t *buf, int available, int &remainingLength, int &rlBytes) {
  remainingLength = 0;
  rlBytes = 0;
  int multiplier = 1;
  uint8_t digit;
  do {
    if (rlBytes >= available || rlBytes > 4) {
      DEBUG_EXTENDEDF("Task1: decodeRemainingLength: Not enough bytes or too many iterations. Available: %d, rlBytes: %d\n", available, rlBytes);
      return false;
    }
    digit = buf[rlBytes++];
    remainingLength += (digit & 127) * multiplier;
    multiplier *= 128;
  } while (digit & 0x80);
  DEBUG_EXTENDEDF("Task1: Decoded remaining length: %d, bytes used: %d\n", remainingLength, rlBytes);
  return true;
}

SensorMessage parseSensorMessage(const String &payload) {
  SensorMessage msg = {};  // Zero-initialize all fields.
  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    DEBUG_SIMPLEF("Task1: JSON deserialization failed: %s | Payload: %s\n", error.c_str(), payload.c_str());
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

  // Update the last MQTT message time.
  lastMQTTMessageTime = millis();

  // NEW: Print a single DEBUG_INTERMEDIATEF message with all values from the MQTT message.
  DEBUG_INTERMEDIATEF("Task1: MQTT message parsed: Pi=%.2f, Po=%.2f, Pi1=%.2f, Pi2=%.2f, Pi3=%.2f, Po1=%.2f, Po2=%.2f, Po3=%.2f, U1=%.2f, U2=%.2f, U3=%.2f, I1=%.2f, I2=%.2f, I3=%.2f, Ei=%.2f, Eo=%.2f, Ei1=%.2f, Ei2=%.2f, Eo1=%.2f, Eo2=%.2f, Q5=%.2f, Q6=%.2f, Q7=%.2f, Q8=%.2f\n",
                      msg.data.Pi, msg.data.Po, msg.data.Pi1, msg.data.Pi2, msg.data.Pi3,
                      msg.data.Po1, msg.data.Po2, msg.data.Po3,
                      msg.data.U1, msg.data.U2, msg.data.U3,
                      msg.data.I1, msg.data.I2, msg.data.I3,
                      msg.data.Ei, msg.data.Eo, msg.data.Ei1, msg.data.Ei2, msg.data.Eo1, msg.data.Eo2,
                      msg.data.Q5, msg.data.Q6, msg.data.Q7, msg.data.Q8);
  return msg;
}

// ----- New: Accumulate & Average Sensor Messages -----
// This function accumulates five messages and then sends the average.
void accumulateSensorMessage(const SensorMessage &msg) {
  static SensorMessage accumMsg = {};  // Zero-initialized accumulator.

  accumMsg.data.Pi += msg.data.Pi;
  accumMsg.data.Po += msg.data.Po;
  accumMsg.data.Pi1 += msg.data.Pi1;
  accumMsg.data.Pi2 += msg.data.Pi2;
  accumMsg.data.Pi3 += msg.data.Pi3;
  accumMsg.data.Po1 += msg.data.Po1;
  accumMsg.data.Po2 += msg.data.Po2;
  accumMsg.data.Po3 += msg.data.Po3;
  accumMsg.data.U1 += msg.data.U1;
  accumMsg.data.U2 += msg.data.U2;
  accumMsg.data.U3 += msg.data.U3;
  accumMsg.data.I1 += msg.data.I1;
  accumMsg.data.I2 += msg.data.I2;
  accumMsg.data.I3 += msg.data.I3;
  accumMsg.data.Ei += msg.data.Ei;
  accumMsg.data.Eo += msg.data.Eo;
  accumMsg.data.Ei1 += msg.data.Ei1;
  accumMsg.data.Ei2 += msg.data.Ei2;
  accumMsg.data.Eo1 += msg.data.Eo1;
  accumMsg.data.Eo2 += msg.data.Eo2;
  accumMsg.data.Q5 += msg.data.Q5;
  accumMsg.data.Q6 += msg.data.Q6;
  accumMsg.data.Q7 += msg.data.Q7;
  accumMsg.data.Q8 += msg.data.Q8;

  messageCount++;
  displaySensorData(msg);
  DEBUG_SIMPLEF("Task1: Accumulated message count: %d\n", messageCount);
  DEBUG_SIMPLEF("Task1: Accumulated values so far: Pi=%.2f, Po=%.2f, Pi1=%.2f, Pi2=%.2f, Pi3=%.2f\n",
                accumMsg.data.Pi, accumMsg.data.Po, accumMsg.data.Pi1, accumMsg.data.Pi2, accumMsg.data.Pi3);

  if (messageCount == NUMBER_AVERAGED_MESSAGES) {
    SensorMessage avgMsg = {};  // Zero-initialize the averaged message.
    avgMsg.data.Pi = accumMsg.data.Pi / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Po = accumMsg.data.Po / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Pi1 = accumMsg.data.Pi1 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Pi2 = accumMsg.data.Pi2 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Pi3 = accumMsg.data.Pi3 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Po1 = accumMsg.data.Po1 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Po2 = accumMsg.data.Po2 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Po3 = accumMsg.data.Po3 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.U1 = accumMsg.data.U1 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.U2 = accumMsg.data.U2 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.U3 = accumMsg.data.U3 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.I1 = accumMsg.data.I1 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.I2 = accumMsg.data.I2 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.I3 = accumMsg.data.I3 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Ei = accumMsg.data.Ei / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Eo = accumMsg.data.Eo / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Ei1 = accumMsg.data.Ei1 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Ei2 = accumMsg.data.Ei2 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Eo1 = accumMsg.data.Eo1 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Eo2 = accumMsg.data.Eo2 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Q5 = accumMsg.data.Q5 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Q6 = accumMsg.data.Q6 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Q7 = accumMsg.data.Q7 / NUMBER_AVERAGED_MESSAGES;
    avgMsg.data.Q8 = accumMsg.data.Q8 / NUMBER_AVERAGED_MESSAGES;

    DEBUG_SIMPLEF("Task1: Averaged values to be sent: Pi=%.2f, Po=%.2f, Pi1=%.2f, Pi2=%.2f, Pi3=%.2f\n",
                  avgMsg.data.Pi, avgMsg.data.Po, avgMsg.data.Pi1, avgMsg.data.Pi2, avgMsg.data.Pi3);
    // Send the averaged message.
    if (xQueueSend(sensorMsgQueue, &avgMsg, 0) != pdPASS) {
      DEBUG_SIMPLE("Task1: Error - Queue full! Averaged SensorMessage not sent.");
      displayError("Queue full!");
    } else {
      DEBUG_EXTENDED("Task1: Averaged SensorMessage sent to queue.");
      displaySensorData(avgMsg);
    }
    // Reset the accumulator.
    accumMsg = {};
    messageCount = 0;
  }
}

void processBuffer_Rtos(WiFiClient &client, uint8_t *buffer, int len) {
  int index = 0;
  DEBUG_EXTENDEDF("Task1: Processing buffer of length: %d\n", len);
  while (index < len) {
    if (len - index < 2) {
      DEBUG_EXTENDED("Task1: Error: Not enough bytes for fixed header and remaining length. Index: " + String(index) + ", Buffer length: " + String(len));
      break;
    }
    uint8_t header = buffer[index];
    uint8_t packetType = header >> 4;
    int remainingLength = 0;
    int rlBytes = 0;
    if (!decodeRemainingLength(buffer + index + 1, len - (index + 1), remainingLength, rlBytes)) {
      DEBUG_EXTENDED("Task1: Error: Malformed remaining length field at index: " + String(index));
      break;
    }
    int fixedHeaderSize = 1 + rlBytes;
    int packetTotalLength = fixedHeaderSize + remainingLength;
    if (index + packetTotalLength > len) {
      String partialPacket = "";
      for (int i = index; i < len; i++) {
        partialPacket += String(buffer[i], HEX) + " ";
      }
      DEBUG_EXTENDEDF("Task1: Incomplete packet received at index: %d, packetTotalLength: %d, available: %d. Content: %s\nDiscarding remainder of buffer.\n",
                      index, packetTotalLength, len - index, partialPacket.c_str());
      break;
    }

    if (packetType == 3 && remainingLength >= 2) {
      int topicLengthIndex = index + fixedHeaderSize;
      uint16_t topicLength = (buffer[topicLengthIndex] << 8) | buffer[topicLengthIndex + 1];
      if (remainingLength >= 2 + topicLength) {
        String topic = String((char *)(buffer + topicLengthIndex + 2), topicLength);
        DEBUG_EXTENDED("Task1: Publish topic: " + topic);
        DEBUG_EXTENDEDF("Task1: Packet length: %d\n", packetTotalLength);
        uint8_t qos = (header & 0x06) >> 1;
        int extraBytes = (qos > 0) ? 2 : 0;
        int payloadStart = index + fixedHeaderSize + 2 + topicLength + extraBytes;
        int payloadLength = packetTotalLength - (fixedHeaderSize + 2 + topicLength + extraBytes);
        if (payloadLength >= 0) {
          String payload = String((char *)(buffer + payloadStart), payloadLength);
          DEBUG_SIMPLE("Task1: Payload: " + payload);
          SensorMessage sensorMsg = parseSensorMessage(payload);
          accumulateSensorMessage(sensorMsg);
        } else {
          DEBUG_EXTENDED("Task1: Negative payload length computed at index: " + String(index));
        }
      } else {
        DEBUG_EXTENDED("Task1: Remaining length less than expected topic length at index: " + String(index));
      }
    } else if (packetType == 1) {
      uint8_t connack[] = { 0x20, 0x02, 0x00, 0x00 };
      client.write(connack, sizeof(connack));
      DEBUG_SIMPLE("Task1: Processed CONNECT packet, sent CONNACK.");
    } else if (packetType == 12) {
      uint8_t pingresp[] = { 0xD0, 0x00 };
      client.write(pingresp, sizeof(pingresp));
      DEBUG_EXTENDED("Task1: Processed PINGREQ, sent PINGRESP.");
    }
    index += packetTotalLength;
    DEBUG_EXTENDEDF("Task1: Moving to next packet at index: %d\n", index);
  }
}

void processMQTTClient_Rtos(WiFiClient &client) {
  DEBUG_SIMPLE("Task1: New MQTT client connected");
  lastMQTTClientConnect = millis();
  while (client.connected()) {
    if (client.available()) {
      uint8_t buffer[MAX_PACKET_SIZE] = { 0 };
      int len = client.readBytes(buffer, sizeof(buffer));
      if (len > 0) {
        DEBUG_SIMPLEF("Task1: Received MQTT packet of length: %d\n", len);
        processBuffer_Rtos(client, buffer, len);
      } else {
        DEBUG_EXTENDED("Task1: Received zero-length packet, skipping.");
      }
    }
    delay(10);
  }
  client.stop();
  DEBUG_SIMPLE("Task1: MQTT client disconnected");
}

// ----- LoRaWAN (RadioLib) Global Variables and Configuration -----
SX1276 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, LORA_DIO1);
const uint32_t uplinkIntervalSeconds = 5UL * 60UL;

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
  // NEW: Print a single DEBUG_INTERMEDIATEF message with all sensor values before packing.
  DEBUG_INTERMEDIATEF("Task2: SensorMessage before packing: Pi=%.2f, Po=%.2f, Pi1=%.2f, Pi2=%.2f, Pi3=%.2f, Po1=%.2f, Po2=%.2f, Po3=%.2f, U1=%.2f, U2=%.2f, U3=%.2f, I1=%.2f, I2=%.2f, I3=%.2f, Ei=%.2f, Eo=%.2f, Ei1=%.2f, Ei2=%.2f, Eo1=%.2f, Eo2=%.2f, Q5=%.2f, Q6=%.2f, Q7=%.2f, Q8=%.2f\n",
                      msg.data.Pi, msg.data.Po, msg.data.Pi1, msg.data.Pi2, msg.data.Pi3,
                      msg.data.Po1, msg.data.Po2, msg.data.Po3,
                      msg.data.U1, msg.data.U2, msg.data.U3,
                      msg.data.I1, msg.data.I2, msg.data.I3,
                      msg.data.Ei, msg.data.Eo, msg.data.Ei1, msg.data.Ei2, msg.data.Eo1, msg.data.Eo2,
                      msg.data.Q5, msg.data.Q6, msg.data.Q7, msg.data.Q8);

  // Pack the values into a LoRaWAN packet.
  SensorMessageLoRaWAN packed;
  // For illustration, we now copy values from the averaged SensorMessage.
  packed.data.Pi = msg.data.Pi;
  packed.data.Po = msg.data.Po;
  packed.data.Pi1 = msg.data.Pi1;
  packed.data.Pi2 = msg.data.Pi2;
  packed.data.Pi3 = msg.data.Pi3;
  packed.data.Po1 = msg.data.Po1;
  packed.data.Po2 = msg.data.Po2;
  packed.data.Po3 = msg.data.Po3;
  // Note: SensorDataLoRaWAN does not include U1, U2, U3.
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

  DEBUG_SIMPLEF("Task2: Sending LoRaWAN packet with values: Pi=%.2f, Po=%.2f, Pi1=%.2f, Pi2=%.2f, Pi3=%.2f\n",
                packed.data.Pi, packed.data.Po, packed.data.Pi1, packed.data.Pi2, packed.data.Pi3);
  int16_t state = node.sendReceive((uint8_t *)&packed, sizeof(packed));
  int16_t rssi = radio.getRSSI();
  String rssiMessage = "RSSI: " + String(rssi);
  displayStatus(rssiMessage, 4);
  debug(state < RADIOLIB_ERR_NONE, F("Error in sendReceive"), state, false);
  if (state > 0) {
    Serial.println(F("Task2: Downlink received"));

  } else {
    DEBUG_EXTENDED("Task2: No downlink received");
  }
  DEBUG_SIMPLE("Task2: LoRa message sent");
}

void WatchdogTask(void *parameter) {
  for (;;) {
    unsigned long now = millis();
    DEBUG_EXTENDEDF("Watchdog: Checking MQTT message timeout... %lu ms\n", now - lastMQTTMessageTime);
    if (now - lastMQTTMessageTime > MQTT_MSG_TIMEOUT) {
      rebootSystem("No MQTT message received in 1 minute");
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

bool initializeHardware() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    DEBUG_SIMPLE("Task1: SSD1306 allocation failed");
    return false;
  }
  display.clearDisplay();
  display.display();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("OLED init OK!");
  display.display();
  DEBUG_SIMPLE("Task1: OLED initialized successfully.");

  IPAddress local_ip(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  WiFi.softAP(ssid, password);
  DEBUG_SIMPLEF("Task1: Access Point started, IP: %s\n", WiFi.softAPIP().toString().c_str());
  displayStatus("WiFi started!", 3);

  mqttServer.begin();
  DEBUG_SIMPLE("Task1: MQTT Broker started on port 1883");

  return true;
}

void rebootSystem(String reason) {
  DEBUG_SIMPLE("Task1: Critical error: " + reason);
  displayError(reason);
  delay(3000);
  ESP.restart();
}

void Task1_MQTT(void *parameter) {
  for (;;) {
    WiFiClient client = mqttServer.available();
    if (client) {
      DEBUG_SIMPLE("Task1: MQTT client available, processing connection...");
      displayStatus("MQTT broker connected!", 4);
      processMQTTClient_Rtos(client);
      lastMQTTClientConnect = millis();
      DEBUG_SIMPLE("Task1: Updated lastMQTTClientConnect after client processing.");
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

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
      // Retrieve RSSI of the join response.
      int16_t rssi = radio.getRSSI();
      Serial.print(F("Task2: LoRaWAN Joined Successfully! RSSI: "));
      Serial.println(rssi);
      displayStatus("Joined! RSSI: " + String(rssi), 2);
      break;
    } else {
      Serial.print(F("Task2: LoRaWAN join failed with state "));
      Serial.println(state);
      displayStatus("LoRa join error!", 2);
      displayError("LoRa join error!");
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
  }

  SensorMessage SensorMessageLoRaWAN;
  for (;;) {
    if (xQueueReceive(sensorMsgQueue, &SensorMessageLoRaWAN, portMAX_DELAY) == pdPASS) {
      DEBUG_EXTENDED("Task2: Received SensorMessage from queue, transmitting via LoRaWAN.");
      sendSensorMessageLoRaWAN(SensorMessageLoRaWAN);
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
    DEBUG_SIMPLE("Task1: Failed to create sensor message queue.");
    while (true) { vTaskDelay(1000 / portTICK_PERIOD_MS); }
  }

  // The accumulator resets are handled inside accumulateSensorMessage,
  // so there is no need to reset them here.

  xTaskCreate(Task1_MQTT, "Task1_MQTT", 8192, NULL, 1, &Task1_MQTT_Handle);
  xTaskCreate(Task2_LoRa, "Task2_LoRa", 16384, NULL, 1, &Task2_LoRa_Handle);
  xTaskCreate(WatchdogTask, "WatchdogTask", 4096, NULL, 1, &Watchdog_Handle);
}

void loop() {
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
