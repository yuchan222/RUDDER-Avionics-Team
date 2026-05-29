#ifndef LOGGER_H
#define LOGGER_H

#include <SdFat.h>
#include "Packet.h"
#include "CommandRx.h"

#define SD_CS_PIN 10
#define FILE_PREALLOC_MB 8
#define SAMPLES_SYNC 100

void initSD();
void initSDlog();
void writePacket(const DataPacket &p);
void startLogging();
void stopLogging();
extern bool logging;
extern SdFat sd;
extern SdFile file;
extern uint8_t finalValue;
#endif
