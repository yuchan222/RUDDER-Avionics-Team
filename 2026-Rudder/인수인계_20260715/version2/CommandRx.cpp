#include "CommandRx.h"

static uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (uint8_t)(crc << 1);
  }
  return crc;
}

void CommandRx::begin(uint32_t baud) {
  _serial.begin(baud);
}

bool CommandRx::update() {
  while (_serial.available()) {
    uint8_t byte = (uint8_t)_serial.read();

    // 헤더 동기화: 첫 바이트, 두 번째 바이트 모두 0x3C
    if (_pos == 0 && byte != 0x3C) continue;
    if (_pos == 1 && byte != 0x3C) { _pos = 0; continue; }

    _buf[_pos++] = byte;

    // 패킷 완성: [0x3C][0x3C][CMD×4][CRC8] = 7바이트
    if (_pos >= 7) {
      bool ok = parsePacket();
      _pos = 0;
      if (ok) return true;
    }
  }
  return false;
}

bool CommandRx::parsePacket() {
  // _buf[0..1] = 0x3C 0x3C, _buf[2..5] = CMD×4, _buf[6] = CRC8
  uint8_t c0 = _buf[2], c1 = _buf[3], c2 = _buf[4], c3 = _buf[5];

  // 4바이트 중 2개 이상 일치하는 값 다수결
  uint8_t cmd = 0;
  bool found = false;
  uint8_t candidates[4] = {c0, c1, c2, c3};
  for (uint8_t i = 0; i < 4 && !found; i++) {
    for (uint8_t j = i + 1; j < 4 && !found; j++) {
      if (candidates[i] == candidates[j]) {
        cmd   = candidates[i];
        found = true;
      }
    }
  }
  if (!found) return false;

  if (_buf[6] != crc8(_buf, 6)) return false;

  _last = cmd;
  return true;
}

uint8_t CommandRx::lastCmd() const { return _last; }
