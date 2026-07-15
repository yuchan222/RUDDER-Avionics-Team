// MPU-6050 단독 테스트 — 연결 상태 + 실시간 가속도/자이로 출력
// 배선: VCC→5V(또는 3.3V), GND→GND, SDA→A4, SCL→A5

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;
static bool    s_inited    = false;
static uint8_t s_foundAddr = 0;

static bool probe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) {}
  Wire.begin();
  Serial.println("=== MPU-6050 단독 테스트 ===");
}

void loop() {
  if (!s_inited) {
    // 1단계: I2C 버스에서 주소 응답 확인
    s_foundAddr = probe(0x68) ? 0x68 : (probe(0x69) ? 0x69 : 0);

    if (s_foundAddr == 0) {
      Serial.println("[X] I2C 미인식 — VCC/GND/SDA(A4)/SCL(A5) 배선 확인");
    } else if (mpu.begin(s_foundAddr)) {
      mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
      Serial.print("[O] MPU-6050 초기화 성공! 주소 0x");
      Serial.println(s_foundAddr, HEX);
      if (s_foundAddr == 0x69)
        Serial.println("    (주의: 0x69 = AD0 HIGH 상태. Config.h IMU_ADDR와 다름)");
      s_inited = true;
    } else {
      Serial.print("[!] 주소 0x"); Serial.print(s_foundAddr, HEX);
      Serial.println(" 응답은 있으나 init 실패 — 전원 전압/납땜 확인");
    }
    delay(1000);
    return;
  }

  // 2단계: 실시간 값 출력 (보드를 기울이면 acc 값이 변해야 정상)
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);
  Serial.print("[MPU] acc(m/s2) X="); Serial.print(a.acceleration.x, 2);
  Serial.print(" Y=");                Serial.print(a.acceleration.y, 2);
  Serial.print(" Z=");                Serial.print(a.acceleration.z, 2);
  Serial.print(" | gyro(rad/s) X="); Serial.print(g.gyro.x, 2);
  Serial.print(" Y=");               Serial.print(g.gyro.y, 2);
  Serial.print(" Z=");               Serial.print(g.gyro.z, 2);
  Serial.print(" | temp=");          Serial.print(t.temperature, 1);
  Serial.println(" C");

  // 연결 끊김 감지 → 재탐색 모드로 복귀
  if (!probe(s_foundAddr)) {
    Serial.println("[X] 연결 끊김 감지 — 재탐색 시작");
    s_inited = false;
  }
  delay(500);
}
