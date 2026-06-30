#ifndef EJECTION_LOGIC_H
#define EJECTION_LOGIC_H

#include <Arduino.h>
#include <Servo.h>
#include "Config.h"
#include "Packet.h"

void initEjection(Servo &servo);
bool updateEjection(const DataPacket &p);
bool checkLanded(const DataPacket &p);   // Mode3→4 자동전환 판단 (#24)
void forceEject();

bool isLaunched();
bool isEjected();

#endif
