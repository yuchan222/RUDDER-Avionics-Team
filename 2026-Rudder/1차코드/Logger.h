#ifndef LOGGER_H
#define LOGGER_H

#include <SdFat.h>
#include "Packet.h"
#include "Config.h"

// sd 객체는 Logger.cpp에서 정의 — 다른 파일에서 extern으로 참조
extern SdFat sd;

bool initSD();
bool openLogFile();
void writePacket(const DataPacket &p);
void flushLog();
void closeLog();

#endif
