#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include "MPU9250.h"
#include "Packet.h"

void initSensors();
void readSensors(DataPacket &p);
void clearI2CBus(uint8_t sdaPin = 20, uint8_t sclPin = 21);
extern Adafruit_BMP3XX bmp;
extern MPU9250 imu;
extern MPU9250Setting setting;

#endif
