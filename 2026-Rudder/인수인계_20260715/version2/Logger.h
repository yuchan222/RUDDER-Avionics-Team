#ifndef LOGGER_H
#define LOGGER_H

#include <SdFat.h>
#include "Packet.h"
#include "Config.h"

extern SdFat sd;

bool initSD();
bool isLogReady();                       // SD 초기화 성공 여부 (#11)
bool openLogFile();
void finalizePacket(DataPacket &p);      // len + CRC16 계산, SD/텔레메트리 전 호출 (#5, #6)
void writePacket(const DataPacket &p);   // finalizePacket() 완료 후 호출
void flushLog();
void closeLog();

#endif
