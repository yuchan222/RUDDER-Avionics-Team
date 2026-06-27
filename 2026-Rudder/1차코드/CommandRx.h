#ifndef COMMAND_RX_H
#define COMMAND_RX_H

#include <Arduino.h>

// ── 수신 명령 코드 ────────────────────────────────────────────────────────
#define CMD_MODE_NEXT  0x0B   // 모드 +1
#define CMD_MODE_PREV  0x16   // 모드 -1
#define CMD_EJECT      0x42   // 비상 사출
#define CMD_TIMER_A    0x4D   // 백업 타이머 즉시 시작 (millis 기반 — 미사용 예비)
#define CMD_STOP       0x58   // 타이머 정지 예비
#define CMD_SYSRESET   0x63   // 소프트 리셋

// ── 프로토콜: 0x3C 0x3C | CMD×4 | CRC8 (7바이트) ────────────────────────
class CommandRx {
public:
  explicit CommandRx(HardwareSerial &serial) : _serial(serial) {}

  void    begin(uint32_t baud);
  bool    update();           // 매 루프 호출, 새 명령 수신 시 true
  uint8_t lastCmd() const;

private:
  HardwareSerial &_serial;
  uint8_t _buf[8] = {};
  uint8_t _pos    = 0;
  uint8_t _last   = 0;

  bool parsePacket();
};

#endif
