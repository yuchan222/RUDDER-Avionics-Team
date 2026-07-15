#include <Servo.h>

#define PIN_SERVO   9
#define PIN_SER     4    // 74HC595 DS
#define PIN_SRCLK   5    // 74HC595 SHCP
#define PIN_RCLK    6    // 74HC595 STCP
#define PIN_BUZ     8    // 능동 부저 (LOW = ON)

#define SERVO_CLOSED_DEG 0
#define SERVO_EJECT_DEG  90

Servo myServo;

static void shiftOut595(uint8_t data) {
  digitalWrite(PIN_RCLK, LOW);
  for (int8_t i = 7; i >= 0; i--) {
    digitalWrite(PIN_SRCLK, LOW);
    digitalWrite(PIN_SER, (data >> i) & 1 ? HIGH : LOW);
    digitalWrite(PIN_SRCLK, HIGH);
  }
  digitalWrite(PIN_RCLK, HIGH);
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}

  pinMode(PIN_SER,   OUTPUT);
  pinMode(PIN_SRCLK, OUTPUT);
  pinMode(PIN_RCLK,  OUTPUT);
  pinMode(PIN_BUZ,   OUTPUT);
  digitalWrite(PIN_BUZ, HIGH);   // 부저 OFF

  myServo.attach(PIN_SERVO);
  myServo.write(SERVO_CLOSED_DEG);

  Serial.println("=== Servo / LED(74HC595) / Buzzer 테스트 ===");
  Serial.println("서보가 0도<->90도로 움직이는지, LED 8개 켜졌다 꺼지는지, 짧은 삑 소리 나는지 눈/귀로 확인하세요.");
}

void loop() {
  Serial.println("[TEST] Servo -> 90deg (EJECT), LED ALL ON");
  myServo.write(SERVO_EJECT_DEG);
  shiftOut595(0xFF);
  delay(1000);

  Serial.println("[TEST] Buzzer beep");
  digitalWrite(PIN_BUZ, LOW);
  delay(200);
  digitalWrite(PIN_BUZ, HIGH);

  Serial.println("[TEST] Servo -> 0deg (CLOSED), LED ALL OFF");
  myServo.write(SERVO_CLOSED_DEG);
  shiftOut595(0x00);
  delay(1500);
}
