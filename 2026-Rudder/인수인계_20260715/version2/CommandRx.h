#ifndef COMMAND_RX_H
#define COMMAND_RX_H

#include <Arduino.h>

// ── 수신 명령 코드 (#12: NEXT/PREV → SET_STANDBY/ARMED으로 교체) ─────────
#define CMD_SET_STANDBY  0x0B   // Mode 0 (대기) 직접 이동
#define CMD_SET_ARMED    0x16   // Mode 1 (Armed) 직접 이동
#define CMD_FORCE_EJECT  0x42   // 비상 사출 (Mode ≥ 1 에서만 허용) (#13)
#define CMD_SYSRESET     0x63   // 소프트 리셋 (Mode 0 한정) (#14)
// CMD_TIMER_A 0x4D — 삭제 (미사용) (#15)
// CMD_STOP    0x58 — 삭제 (미사용) (#15)

// ── 프로토콜: 0x3C 0x3C | CMD×4 | CRC8 (7바이트) ────────────────────────
class CommandRx {
public:
  explicit CommandRx(HardwareSerial &serial) : _serial(serial) {}
  void    begin(uint32_t baud);
  bool    update();
  uint8_t lastCmd() const;

private:
  HardwareSerial &_serial;
  uint8_t _buf[8] = {};
  uint8_t _pos    = 0;
  uint8_t _last   = 0;
  bool parsePacket();
};

#endif
