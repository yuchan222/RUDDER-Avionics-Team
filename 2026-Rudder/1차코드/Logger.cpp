#include "Logger.h"
#include <stddef.h>    // offsetof

SdFat sd;

static FsFile    s_file;
static uint32_t  s_pktCount = 0;

static uint16_t crc16(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (uint16_t)(crc << 1);
  }
  return crc;
}

bool initSD() {
  return sd.begin(PIN_SD_CS, SD_SCK_MHZ(4));
}

bool openLogFile() {
  char fname[16];
  for (uint16_t n = 1; n <= 99999; n++) {
    snprintf(fname, sizeof(fname), "LOG%05u.RAW", n);
    if (!sd.exists(fname)) break;
  }

  s_file = sd.open(fname, O_WRONLY | O_CREAT | O_TRUNC);
  if (!s_file) return false;

  s_file.preAllocate((uint32_t)FILE_PREALLOC_MB * 1024UL * 1024UL);
  s_pktCount = 0;
  Serial.print("[SD] log: "); Serial.println(fname);
  return true;
}

void writePacket(const DataPacket &p) {
  DataPacket tmp = p;
  // CRC16 계산: crc16 필드 직전까지
  tmp.crc16 = crc16((uint8_t*)&tmp, (uint16_t)offsetof(DataPacket, crc16));
  s_file.write((const uint8_t*)&tmp, sizeof(tmp));

  if (++s_pktCount % SAMPLES_SYNC == 0) s_file.sync();
}

void flushLog()  { s_file.sync();  }
void closeLog()  { s_file.close(); }
