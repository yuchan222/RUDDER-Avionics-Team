#ifndef CONFIG_H
#define CONFIG_H

// ── 보드: Arduino UNO R4 Minima ──────────────────────────────────────────

// ── 핀 매핑 ──────────────────────────────────────────────────────────────
#define PIN_SERVO        9
#define PIN_SD_CS       10
#define PIN_SER          4    // 74HC595 DS
#define PIN_SRCLK        5    // 74HC595 SHCP
#define PIN_RCLK         6    // 74HC595 STCP
#define PIN_BUZ          8    // 능동 부저 (LOW = ON)

// I2C: UNO R4 Minima A4(SDA)/A5(SCL)
#define I2C_SDA_PIN     18
#define I2C_SCL_PIN     19

// ── 통신 ─────────────────────────────────────────────────────────────────
#define BAUD_USB       115200
#define BAUD_TELEMETRY  57600

// ── 센서 I2C 주소 ─────────────────────────────────────────────────────────
#define BMP388_ADDR     0x76   // 실패 시 0x77 자동재시도
#define IMU_ADDR        0x68
#define INA219_ADDR     0x40

// ── SD 카드 ───────────────────────────────────────────────────────────────
#define FILE_PREALLOC_MB  8
#define SAMPLES_SYNC    100

// ── 사출 판단 임계값 ──────────────────────────────────────────────────────
#define APOGEE_DROP_M      3.0f
#define APOGEE_DROP_COUNT  5
#define MIN_APOGEE_ALT_M  30.0f
#define SAFE_LOCK_MS      3000

#define ZACC_THRESH_MG    2000
#define ZACC_HOLD_MS       300
#define LAUNCH_ALT_THRESH_M 2.0f

// ── 비행 타이머 ───────────────────────────────────────────────────────────
// 시뮬레이션: 최고고도 271m / 8.51s / 총비행 68s (2026-06-30 팀 합의)
#define TIMER_A_SEC       10.0f

// ── 서보 각도 (Config로 분리) ─────────────────────────────────────────────
#define SERVO_CLOSED_DEG    0
#define SERVO_EJECT_DEG    90

// ── 루프 주기 ─────────────────────────────────────────────────────────────
#define SENSOR_INTERVAL_MS   20    // 센서 샘플링 / 로그 주기 → 50 Hz
#define TELEM_INTERVAL_MS   200    // 텔레메트리 주기 → 5 Hz

// ── 베이스라인 수집 샘플 수 ───────────────────────────────────────────────
#define BASELINE_SAMPLES   50

// ── 착지 감지 임계값 ──────────────────────────────────────────────────────
#define LAND_STABLE_MS      5000   // 안정 유지 시간 [ms]
#define LAND_ALT_CM_THRESH   200   // 고도 변화 상한 [cm] (2m)
#define LAND_ACC_MIN_MG      800   // 착지 가속도 하한 [mg]
#define LAND_ACC_MAX_MG     1200   // 착지 가속도 상한 [mg]

// ── 시스템 상태 비트 ──────────────────────────────────────────────────────
#define STATUS_BMP   0x01
#define STATUS_IMU   0x02
#define STATUS_SD    0x04
#define STATUS_INA   0x08

#endif
