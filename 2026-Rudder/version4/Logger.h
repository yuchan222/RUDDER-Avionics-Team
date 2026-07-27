#ifndef LOGGER_H
#define LOGGER_H

#include "Packet.h"
#include "Config.h"

// version2에서 검증 완료된 SD 로깅 모듈 (사실상 동일 — 잘 되는 건 안 건드림)

bool initSD();                           // SD 초기화 + 새 LOGxxxxx.RAW 열기
bool isLogReady();
void finalizePacket(DataPacket &p);      // len + CRC16 계산 (저장/송신 직전 호출)
void writePacket(const DataPacket &p);
void flushLog();
void closeLog();

#endif
