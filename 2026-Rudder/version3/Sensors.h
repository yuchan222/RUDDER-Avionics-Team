#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "Packet.h"
#include "Config.h"

// 센서 초기화 (IMU는 레지스터 직접제어 — 클론 칩도 동작, WHO_AM_I는 기록만)
void initSensors();

// 지상 기준압 수집 (준비 모드 진입 시 호출, 약 1초 블로킹)
void collectBaseline();

// 센서 일괄 읽기 → 패킷 채움 (50Hz 호출)
// BMP 실패 시 pressure/altitude_cm/bmp_temp = -1
void readSensors(DataPacket &p);

// 현재 센서 상태 비트 (STATUS_BMP|IMU|INA — SD는 Logger 담당)
uint8_t sensorStatus();

#endif
