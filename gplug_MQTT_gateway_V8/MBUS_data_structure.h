#ifndef MBUS_DATA_STRUCTURE_H
#define MBUS_DATA_STRUCTURE_H

#define LORA_FREQUENCY 430.8E6  // 434 MHz
#define LORA_SF 9

// --------- Dynamic Structures (for parsing JSON) ---------
struct SensorData {
  float Pi;
  float Po;
  float Pi1;
  float Pi2;
  float Pi3;
  float Po1;
  float Po2;
  float Po3;
  float U1;
  float U2;
  float U3;
  float I1;
  float I2;
  float I3;
  float Ei;
  float Eo;
  float Ei1;
  float Ei2;
  float Eo1;
  float Eo2;
  float Q5;
  float Q6;
  float Q7;
  float Q8;
  float Q51;
  float Q52;
  float Q61;
  float Q62;
  float Q71;
  float Q72;
  float Q81;
  float Q82;
};

struct SensorMessage {
  SensorData data;  // Contains only sensor data (no timestamp)
};

// --------- Packed Structures (for raw binary transmission) ---------
// These structures are defined as packed so that no padding is inserted.
struct __attribute__((packed)) SensorDataPacked {
  float Pi;
  float Po;
  float Pi1;
  float Pi2;
  float Pi3;
  float Po1;
  float Po2;
  float Po3;
  float I1;
  float I2;
  float I3;
  float Ei;
  float Eo;
  float Ei1;
  float Ei2;
  float Eo1;
  float Eo2;
  float Q5;
  float Q6;
  float Q7;
  float Q8;
  uint16_t parity;
};

struct __attribute__((packed)) SensorMessagePacked {
  SensorDataPacked data;
};

//----------------------------------------

struct __attribute__((packed)) SensorDataLoRaWAN {
  float Pi;
  float Po;
  float Pi1;
  float Pi2;
  float Pi3;
  float Po1;
  float Po2;
  float Po3;
  float I1;
  float I2;
  float I3;
  float Ei;
  float Eo;
  float Ei1;
  float Ei2;
  float Eo1;
  float Eo2;
  float Q5;
  float Q6;
  float Q7;
  float Q8;
};

struct __attribute__((packed)) SensorMessageLoRaWAN {
  SensorDataLoRaWAN data;
};

#endif // MBUS_DATA_STRUCTURE_H