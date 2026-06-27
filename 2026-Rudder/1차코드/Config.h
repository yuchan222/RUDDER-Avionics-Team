#ifndef CONFIG_H
#define CONFIG_H

// ── 보드: Arduino UNO R4 Minima ──────────────────────────────────────────
// Due(pins 20/21 = SDA/SCL, 22~24 = spare) → UNO R4 Minima(A4/A5 = SDA/SCL)
// DueTimer 미지원 → millis() 기반 소프트웨어 타이머로 대체

// ── 핀 매핑 ──────────────────────────────────────────────────────────────
#define PIN_SERVO        9
#define PIN_SD_CS       10

// 74HC595 shift register (Due의 22/23/24 → UNO R4 spare 핀으로 이동)
#define PIN_SER          4    // DS
#define PIN_SRCLK        5    // SHCP
#define PIN_RCLK         6    // STCP

#define PIN_BUZ          8    // 능동 부저 (LOW = ON)

// I2C: UNO R4 Minima A4(SDA)/A5(SCL)
#define I2C_SDA_PIN     18    // A4
#define I2C_SCL_PIN     19    // A5

// ── 통신 ─────────────────────────────────────────────────────────────────
#define BAUD_USB       115200
#define BAUD_TELEMETRY  57600  // SiK radio (Serial1 = D0/D1)

// ── 센서 I2C 주소 ─────────────────────────────────────────────────────────
#define BMP388_ADDR     0x76
#define IMU_ADDR        0x68

// ── SD 카드 ───────────────────────────────────────────────────────────────
#define FILE_PREALLOC_MB  8
#define SAMPLES_SYNC    100

// ── 사출 판단 임계값 ──────────────────────────────────────────────────────
#define APOGEE_DROP_M      3.0f   // 최고점에서 이 값 이상 하강 시 정점 판단
#define APOGEE_DROP_COUNT  5      // 연속 N회 조건 충족 시 확정 (노이즈 방지)
#define MIN_APOGEE_ALT_M  30.0f   // 최소 상승 고도 — 이 이하면 무시 (보고서 미명시, 지면 반사 노이즈 방지용 자체 추가값)
#define SAFE_LOCK_MS      3000    // 발사 후 이 시간 동안 사출 판단 잠금 [ms]

#define ZACC_THRESH_MG    2000    // Z축 가속도 임계 [mg] (발사 감지)
#define ZACC_HOLD_MS       300    // Z축 조건 유지 시간 [ms]
#define LAUNCH_ALT_THRESH_M 2.0f  // 발사 감지 이중 조건: BMP 고도 증가 임계 [m]

// ── 비행 타이머 (millis 기반) ─────────────────────────────────────────────
#define TIMER_A_SEC       12.0f   // 발사 감지 후 백업 사출 지연 [s]

// ── 베이스라인 수집 샘플 수 ───────────────────────────────────────────────
#define BASELINE_SAMPLES   50

#endif
