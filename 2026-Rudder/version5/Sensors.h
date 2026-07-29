#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include "Packet.h"
#include "Config.h"

// 센서 초기화 (IMU는 레지스터 직접제어 — 클론 칩도 동작, WHO_AM_I는 기록만)
void initSensors();

// 지상 기준압 수집 (준비 모드 진입 시 호출, 약 1초 블로킹)
// 반환값: 50샘플 중 BASELINE_MIN_OK 이상 성공했으면 true (모드 진입 자체는 막지 않음, 상태 표시용)
bool collectBaseline();

// 센서 일괄 읽기 → 패킷 채움 (50Hz 호출)
// BMP 실패 시 pressure/altitude_cm = ALT_INVALID(Config.h), bmp_temp = -1
void readSensors(DataPacket &p);

// 현재 센서 상태 비트 (STATUS_BMP|IMU|INA — SD는 Logger 담당)
uint8_t sensorStatus();

#endif
