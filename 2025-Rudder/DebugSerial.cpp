#include "DebugSerial.h"

void displayPacket(const DataPacket& pkt) {
  Serial.println(F("======= DataPacket ======="));
  Serial.print(F("STX: 0x")); Serial.println(pkt.stx, HEX);
  Serial.print(F("From: ")); Serial.print(pkt.from);
  Serial.print(F("  ID: ")); Serial.println(pkt.id);
  Serial.print(F("Length: ")); Serial.println(pkt.len);
  Serial.print(F("Millis: ")); Serial.println(pkt.ms);

  Serial.print(F("Accel: "));
  for (int i = 0; i < 3; i++) {
    Serial.print(pkt.acc[i]); Serial.print(" ");
  }
  Serial.println();

  Serial.print(F("Gyro:  "));
  for (int i = 0; i < 3; i++) {
    Serial.print(pkt.gyro[i]); Serial.print(" ");
  }
  Serial.println();

  Serial.print(F("Mag:   "));
  for (int i = 0; i < 3; i++) {
    Serial.print(pkt.mag[i]); Serial.print(" ");
  }
  Serial.println();

  Serial.print(F("Quat:  "));
  for (int i = 0; i < 4; i++) {
    Serial.print(pkt.quat[i] / 10000.0, 4); Serial.print(" ");
  }
  Serial.println();

  Serial.print(F("Pressure: ")); Serial.print(pkt.pressure); Serial.println(F(" Pa"));
  Serial.print(F("BMP Temp: ")); Serial.print(pkt.bmp_temp ); Serial.println(F(" °C"));
  Serial.print(F("Packet #: ")); Serial.println(pkt.pkt_no);
  Serial.print(F("CRC16: 0x")); Serial.println(pkt.crc16, HEX);
  Serial.print(F("ETX: 0x")); Serial.println(pkt.etx, HEX);
  Serial.println();
}
