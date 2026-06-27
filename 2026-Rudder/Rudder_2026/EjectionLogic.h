#ifndef EJECTION_LOGIC_H
#define EJECTION_LOGIC_H

#include <Arduino.h>
#include <Servo.h>
#include "Config.h"
#include "Packet.h"

// 초기화 (서보 포인터 저장, 닫힘 위치 고정)
void initEjection(Servo &servo);

// 매 루프 호출 — 발사 감지 → Safe Lock → 정점 판단 → 사출
// 사출이 실행된 루프에서 true 반환
bool updateEjection(const DataPacket &p);

// 지상국 비상 명령 즉시 사출
void forceEject();

bool isLaunched();
bool isEjected();

#endif
