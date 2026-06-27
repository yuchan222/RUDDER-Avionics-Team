#ifndef PACKET_H
#define PACKET_H

#include <Arduino.h>

struct __attribute__((packed)) DataPacket {
  uint16_t stx = 0xAA55;
  uint8_t from = 1, id = 1;
  uint16_t len = 0;
  uint32_t ms;
  int16_t acc[3];
  int16_t gyro[3];
  int16_t mag[3];
  int16_t quat[4];   // w,x,y,z  ×10000
  int32_t pressure;  // Pa
  int16_t bmp_temp;  // 0.01°C
  uint16_t pkt_no = 0;
  uint16_t crc16;
  uint16_t etx = 0x55AA;
};
/*
struct __attribute__((packed)) DataPacket {
  uint8_t stx = 0xAA;
  uint8_t from = 1, id = 1;
  uint8_t packetType;
  uint16_t len = 0;
  uint32_t ms;
  int16_t acc[3];
  int16_t gyro[3];
  int16_t mag[3];
  int16_t quat[4];   // w,x,y,z  ×10000
  uint32_t pressure;  // Pa
  uint8_t bmp_temp;  // 0.01°C
  uint16_t pkt_no = 0;
  uint16_t crc16;
  uint8_t etx = 0x55;
};
*/


struct __attribute__((packed)) ConfirmPacket {
  uint8_t stx = 0xAA;
  uint8_t packetType;
  uint8_t len = 0;
  uint32_t ms;
  uint8_t message1;
  uint8_t message2;
  uint16_t pkt_no = 123;
  uint8_t crc8;
  uint8_t etx = 0x55;
};

struct __attribute__((packed)) RxPacket1 {
  uint8_t stx = 0xAA;
  uint8_t packetType;
  uint8_t len = 0;
  uint32_t ms;
  uint8_t message1;
  uint8_t message2;
  uint16_t pkt_no = 123;
  uint8_t crc8;
  uint8_t etx = 0x55;
};

struct __attribute__((packed)) RxPacket2 {
  uint8_t stx = 0xAA;
  uint8_t packetType;
  uint8_t len = 0;
  uint32_t ms;
  uint8_t message1;
  uint8_t message2;
  uint16_t pkt_no = 123;
  uint8_t crc8;
  uint8_t etx = 0x55;
};


#endif
