#include "CommandRx.h"
#include "ModeManager.h"
#define CONFIRM_PACKET_TYPE 0x42  // packetType 고정값 정의
static uint16_t confirmPktCounter = 0;
static uint8_t prevFinalValue = 0xFF;
static uint8_t sendCount = 0;
uint8_t finalValue = 0xFF;

extern ModeManager mode;  
CommandRx::CommandRx(Stream& stream, uint8_t headerByte, uint8_t dataCount)
  : serial(stream), HEADER_BYTE(headerByte), DATA_COUNT(dataCount),
    state(WAIT_FOR_HEADER), headerCount(0), dataIndex(0) {
}

void CommandRx::update() {
  // 단일 바이트 처리로 블로킹 없이 다른 작업과 병행 가능
  if (serial.available()) {
    uint8_t byte = serial.read();
    Serial.println("READ");
    Serial.print("BYTE: 0x");
    Serial.println(byte, HEX);
    Serial.print(state);
    Serial.print("==");
    Serial.println(WAIT_FOR_HEADER);
    handleByte(byte);
  }
}

void CommandRx::handleByte(uint8_t byte) {
  if (state == WAIT_FOR_HEADER) {
    if (byte == HEADER_BYTE) {
      Serial.println("HEAD");
      headerCount++;
      if (headerCount >= 2) {
        state = READ_DATA;
        dataIndex = 0;
        headerCount = 0;
      }
    } else {
      headerCount = 0;
    }
  }
  else if (state == READ_DATA) {
    if (byte != HEADER_BYTE) {
      Serial.println("DATA");
      data[dataIndex++] = byte;
    }
    if (dataIndex >= DATA_COUNT) {
      processData();
      state = WAIT_FOR_HEADER;
      dataIndex = 0;
    }
  }
}

void CommandRx::processData() {
  Serial.println();
  Serial.print("[헤더 감지] ");
  Serial.print(HEADER_BYTE, HEX); Serial.print(" ");
  Serial.print(HEADER_BYTE, HEX); Serial.println();

  Serial.print("수신된 4개 데이터: ");
  for (uint8_t i = 0; i < DATA_COUNT; i++) {
    Serial.print("0x");
    if (data[i] < 0x10) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  finalValue = 0xFF;
  for (uint8_t i = 0; i < DATA_COUNT; i++) {
    uint8_t count = 1;
    for (uint8_t j = i + 1; j < DATA_COUNT; j++) {
      if (data[i] == data[j]) count++;
    }
    if (count >= 2) {
      finalValue = data[i];
      break;
    }
  }

  if (finalValue != 0xFF) { // 명령 들어옴
    Serial.print("최종 선택된 데이터: 0x");
    if (finalValue < 0x10) Serial.print("0");
    Serial.println(finalValue, HEX);

    // 확인 패킷 전송 (공통 동작)
    sendConfirmPacket(finalValue, finalValue);

    // 명령에 따라 행동 분기
    switch (finalValue) {
      case 0x0B:  // 11
        Serial.println("[CMD] 명령 11 수행");
        mode.nextMode();
        mode.save(); 
        // 실행할 함수 또는 동작
        break;

      case 0x16:  // 22
        Serial.println("[CMD] 명령 22 수행");
        mode.prevMode();
        mode.save(); 
        // 실행할 함수 또는 동작
        break;

      case 0x21:  // 33
        Serial.println("[CMD] 명령 33 수행");
        break;

      case 0x2C:  // 44
        Serial.println("[CMD] 명령 44 수행");
        
        break;

      case 0x37:  // 55
        Serial.println("[CMD] 명령 55 수행");
        myServo.close();
        break;

      case 0x42:  // 66
        Serial.println("[CMD] 명령 66 수행 사출");
        myServo.open();
        break;

      case 0x4D:  // 77
        Serial.println("[CMD] 명령 77 수행");
        timer6.start();
        break;

      case 0x58:  // 88
        Serial.println("[CMD] 명령 88 수행");
        timer6.stop();
        timer7.stop();
        break;

      case 0x63:  // 99
        Serial.println("[CMD] 명령 99 수행");
        sendConfirmPacket(finalValue, finalValue);
        NVIC_SystemReset();  // sys reset
        break;

      case 0x65:  // 101
        Serial.println("[CMD] 명령 101 수행");
        break;
      
      case 0x6F:  // 111
      Serial.println("[CMD] 명령 111 수행");
      break;

      case 0x7A:  // 122
        Serial.println("[CMD] 명령 122 수행");
        break;

      case 0x85:  // 133
        Serial.println("[CMD] 명령 133 수행");
        break;

      case 0x90:  // 144
        Serial.println("[CMD] 명령 144 수행");
        break;

      case 0x9B:  // 155
        Serial.println("[CMD] 명령 155 수행");
        break;

      case 0xA6:  // 166
        Serial.println("[CMD] 명령 166 수행");
        break;

      case 0xB1:  // 177
        Serial.println("[CMD] 명령 177 수행");
        break;

      case 0xBC:  // 188
        Serial.println("[CMD] 명령 188 수행");
        break;
        
      case 0xC7:  // 199
        Serial.println("[CMD] 명령 199 수행");
        break;

      default:
        Serial.println("[CMD] 알 수 없는 명령");
        break;
    }
  } else {
    //Serial.println("※ 중복된 값 없음 → 유효 데이터 없음");
  }
}

static uint8_t calcCRC8(const uint8_t* data, size_t len) {
  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++)
      crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
  }
  return crc;
}

void CommandRx::sendConfirmPacket(uint8_t msg1, uint8_t msg2) {
  ConfirmPacket cp;

  cp.packetType = CONFIRM_PACKET_TYPE;  // 고정 packetType 사용
  //cp.len = sizeof(ConfirmPacket);
  cp.ms = millis();
  cp.message1 = msg1;
  cp.message2 = msg2;
  cp.pkt_no = ++confirmPktCounter;

  cp.crc8 = calcCRC8((uint8_t*)&cp, sizeof(ConfirmPacket) - 1);
  cp.len = sizeof(cp.message1)+sizeof(cp.message2);
  Serial1.write((uint8_t*)&cp, sizeof(ConfirmPacket));
}

void CommandRx::sendConfirm() {
  if (finalValue != 0xFF) {
    if (finalValue != prevFinalValue) {
      sendCount = 0;
      prevFinalValue = finalValue;
    }

    if (sendCount < 5) {
      sendConfirmPacket(finalValue, finalValue);
      sendCount++;
      //delay(20);  // 간격 조정 가능
    }
  }
}

void CommandRx::sendRxPacket1(uint8_t packetType, uint8_t msg1, uint8_t msg2) {
  RxPacket1 pkt;
  pkt.packetType = packetType;     // 외부 인자로 설정
  pkt.ms = millis();
  pkt.message1 = msg1;
  pkt.message2 = msg2;
  pkt.len = sizeof(pkt.message1) + sizeof(pkt.message2);
  pkt.crc8 = calcCRC8((uint8_t*)&pkt, sizeof(RxPacket1) - 2);  // ETX 제외
  Serial1.write((uint8_t*)&pkt, sizeof(RxPacket1));
}

void CommandRx::sendRxPacket2(uint8_t packetType, uint8_t msg1, uint8_t msg2) {
  RxPacket2 pkt;
  pkt.packetType = packetType;     // 외부 인자로 설정
  pkt.ms = millis();
  pkt.message1 = msg1;
  pkt.message2 = msg2;
  pkt.len = sizeof(pkt.message1) + sizeof(pkt.message2);
  pkt.crc8 = calcCRC8((uint8_t*)&pkt, sizeof(RxPacket2) - 2);  // ETX 제외
  Serial1.write((uint8_t*)&pkt, sizeof(RxPacket2));
}

