#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_INA219.h>
#include "Packet.h"
#include "Config.h"

void clearI2CBus();
void initSensors();
void collectBaseline();
void readSensors(DataPacket &p);

float   getBaselinePressure();
float   getBaselineTemp();
uint8_t getSystemStatus();     // STATUS_BMP|STATUS_IMU|STATUS_INA 비트 반환

extern Adafruit_MPU6050 mpu;
extern Adafruit_BMP3XX  bmp;
extern Adafruit_INA219  ina219;

// getAltitude() 삭제: readSensors()의 altitude_cm 값만 사용 (#18)

#endif
