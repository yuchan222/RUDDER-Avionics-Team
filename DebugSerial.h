#ifndef DEBUG_SERIAL_H
#define DEBUG_SERIAL_H

#include <Arduino.h>
#include "packet.h"  // DataPacket 구조체 정의 포함

void displayPacket(const DataPacket& pkt);

#endif
