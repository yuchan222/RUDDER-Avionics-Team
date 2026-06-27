#ifndef COMMAND_RX_H
#define COMMAND_RX_H

#include <Arduino.h>
#include "Packet.h"
#include "ServoControl.h"
//#include "ModeManager.h"
#include "EjectionCheck.h"
extern ServoControl myServo;
//extern ModeManager mode;
class CommandRx; // 전방 선언만
class CommandRx {
public:
  CommandRx(Stream& stream, uint8_t headerByte = 0x3C, uint8_t dataCount = 4);
  void update();  // loop()에서 주기적으로 호출
  void sendConfirmPacket(uint8_t msg1, uint8_t msg2);
  void sendConfirm();
  void sendRxPacket1(uint8_t packetType, uint8_t msg1, uint8_t msg2);
  void sendRxPacket2(uint8_t packetType, uint8_t msg1, uint8_t msg2);


private:
  enum State {
    WAIT_FOR_HEADER,
    READ_DATA
  } state;

  Stream& serial;
  const uint8_t HEADER_BYTE;
  const uint8_t DATA_COUNT;

  uint8_t headerCount;
  uint8_t dataIndex;
  uint8_t data[8];

  void handleByte(uint8_t byte);
  void processData();
};

#endif
