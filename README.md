# 📡 MQTT-Extender

> Extend MQTT sensor data over long distances using LoRa and LoRaWAN technologies

[![YouTube Video](https://img.shields.io/badge/YouTube-Watch%20Video-red?style=for-the-badge&logo=youtube)](https://youtu.be/EMUxSV9rrCg)
[![License](https://img.shields.io/badge/License-MIT-blue.svg?style=for-the-badge)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-ESP32-green.svg?style=for-the-badge&logo=espressif)](https://www.espressif.com/)

---

## 📋 Overview

MQTT-Extender bridges local MQTT sensor networks with remote data collection systems using LoRa/LoRaWAN wireless communication. Perfect for IoT deployments where WiFi/Ethernet connectivity is limited or unavailable.

### ✨ Key Features

- 🔌 **MQTT Broker**: Built-in lightweight MQTT broker on ESP32
- 📶 **LoRa/LoRaWAN**: Dual support for point-to-point LoRa and LoRaWAN networks
- 📊 **Data Aggregation**: Averages multiple sensor readings before transmission
- 🔐 **CRC16 Validation**: Ensures data integrity with checksum verification
- ✅ **ACK/NACK Protocol**: Reliable delivery with acknowledgment mechanism
- 📺 **OLED Display**: Real-time status monitoring
- 🔄 **OTA Updates**: Over-the-air firmware updates support
- ⚡ **Watchdog Protection**: Automatic recovery from failures

---

## 🏗️ Architecture

### System 1: Point-to-Point LoRa (V8)
```
Sensor → MQTT → [ESP32 AP] ─── LoRa 433MHz ───→ [ESP32 Gateway] → MQTT Broker
                    ↓                                      ↓
                WiFi AP                              WiFi Station
```

### System 2: LoRaWAN (V3)
```
Sensor → MQTT → [ESP32 AP] ─── LoRaWAN EU868 ───→ LoRaWAN Network Server
                    ↓
                WiFi AP
```

---

## 📁 Repository Structure

```
MQTT-Extender/
│
├── 🧪 Test Projects (433MHz)
│   ├── 433MHz_Test_Repeater/          # LoRa repeater with RSSI/SNR display
│   └── 433MHz_Test_Transmitter/       # LoRa test transmitter
│
├── 🔵 Point-to-Point LoRa System (V8)
│   ├── gplug_AP_V8/                   # Access Point + LoRa Transmitter
│   │   ├── gplug_AP_V8.ino
│   │   └── MBUS_data_structure.h
│   └── gplug_MQTT_gateway_V8/         # LoRa Receiver + MQTT Publisher
│       ├── gplug_MQTT_gateway_V8.ino
│       └── MBUS_data_structure.h
│
└── 🟢 LoRaWAN System (V3)
    └── gplug_AP_LORAWAN_V3/           # Access Point + LoRaWAN Gateway
        └── gplug_AP_LORAWAN_V3.ino
```

---

## 🚀 Getting Started

### Hardware Requirements

- 2x **TTGO LoRa32** (ESP32 with LoRa module)
- **SSD1306 OLED Display** (128x64)
- **433MHz** or **868MHz** antenna (depending on region)

### Software Requirements

- **Arduino IDE** or **PlatformIO**
- **Libraries**:
  - `RadioLib` (for LoRaWAN)
  - `LoRa` (for point-to-point)
  - `ArduinoJson`
  - `Adafruit GFX` + `Adafruit SSD1306`
  - `PubSubClient` (for MQTT)

### Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/SensorsIot/MQTT-Extender.git
   ```

2. Open the desired sketch in Arduino IDE

3. Install required libraries via Library Manager

4. Configure your settings:
   - WiFi credentials
   - MQTT broker settings
   - LoRa frequency and spreading factor
   - LoRaWAN keys (for V3)

5. Upload to your ESP32 boards

---

## ⚙️ Configuration

### Point-to-Point LoRa (V8)

**Access Point (gplug_AP_V8):**
- Creates WiFi AP: `ESP32_AP` / `esp32password`
- MQTT broker on port `1883`
- LoRa frequency: Configure in `secrets.h`
- Debug level: `DEBUG_LEVEL 1`

**Gateway (gplug_MQTT_gateway_V8):**
- Connects to your WiFi network
- Publishes to MQTT topics:
  - `MBUS/values` - Sensor data
  - `MBUS/signal` - RSSI/SNR
  - `MBUS/log` - Error logs

### LoRaWAN (V3)

**Access Point (gplug_AP_LORAWAN_V3):**
- Creates WiFi AP: `ESP32_AP` / `esp32password`
- MQTT broker on port `1883`
- LoRaWAN region: `EU868`
- Averages 6 messages before transmission
- Debug level: `DEBUG_LEVEL 2`
- Configure device keys in code (lines 462-486)

---

## 📊 Data Structure

### Full Structure (V8 - 32 fields, 128 bytes)
- Power: `Pi, Po, Pi1, Pi2, Pi3, Po1, Po2, Po3`
- Voltage: `U1, U2, U3`
- Current: `I1, I2, I3`
- Energy: `Ei, Eo, Ei1, Ei2, Eo1, Eo2`
- Reactive: `Q5, Q6, Q7, Q8, Q51, Q52, Q61, Q62, Q71, Q72, Q81, Q82`

### Compact Structure (V3 - 15 fields, 60 bytes)
- Power: `Pi, Po`
- Current: `I1, I2, I3`
- Energy: `Ei, Eo, Ei1, Ei2, Eo1, Eo2`
- Reactive: `Q5, Q6, Q7, Q8`

---

## 🎯 Use Cases

- 🏭 **Industrial Monitoring**: Remote equipment monitoring
- 🌾 **Agriculture**: Soil sensors in fields without connectivity
- 🏘️ **Smart Cities**: Distributed sensor networks
- ⚡ **Energy Monitoring**: Smart meter data collection
- 🌡️ **Environmental**: Weather stations and air quality monitoring

---

## 🐛 Troubleshooting

### Common Issues

**LoRa not transmitting:**
- Check antenna connection
- Verify frequency settings match between TX/RX
- Ensure spreading factor is compatible

**MQTT broker not responding:**
- Verify WiFi connection
- Check MQTT port (default 1883)
- Review debug output for errors

**Watchdog reboots:**
- Check MQTT message timeout settings
- Verify sensor is publishing data
- Review ACK timeout values (V8 only)

### Debug Levels

Set `DEBUG_LEVEL` in code:
- `1` - Error messages only
- `2` - Info messages (recommended)
- `3` - Full debugging output

---

## 📺 Video Tutorial

For a complete walkthrough and demonstration, watch the video:

[![MQTT Extender Video](https://img.youtube.com/vi/EMUxSV9rrCg/maxresdefault.jpg)](https://youtu.be/EMUxSV9rrCg)

---

## 📄 License

This project is open source and available under the MIT License.

---

## 👤 Author

**Andreas Spiess**

- YouTube: [@AndreasSpiess](https://www.youtube.com/AndreasSpiess)
- GitHub: [@SensorsIot](https://github.com/SensorsIot)

---

## 🤝 Contributing

Contributions, issues, and feature requests are welcome!

---

## ⭐ Show Your Support

If this project helped you, please give it a ⭐️!
