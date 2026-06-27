/* ---------------------------------------------------------------
 * 74HC595 + 능동형 피에조부저(LOW = ON) 제어용 헤더
 *  - MCU  : Arduino Due (3.3 V 로직)
 *  - LED  : 74HC595 Q0(R)·Q1(G)·Q2(Y) 3색 → 680 Ω → LED → GND
 *  - BUZZ : D8 → 1 kΩ → NPN → 부저(–),   부저(+) 5 V
 *  ------------------------------------------------------------- */
 
/* ───────── 핀 매핑 (Due 디지털 핀) ─────────
   • SER   = 직렬 데이터(DS)   → 74HC595 핀 14
   • SRCLK = 시프트 클럭      → 74HC595 핀 11, 상승엣지마다 1 bit 이동
   • RCLK  = 래치  클럭       → 74HC595 핀 12, 상승엣지에서 LED 일괄 갱신
   • BUZ   = 능동부저 제어핀   (LOW = ON, HIGH = OFF)
 ───────── LED 비트 매핑 (Q0·Q1·Q2) ─────────
   ledByte 변수의 개별 비트 의미를 명시적으로 구분
 ───────── Avionics 동작 모드 ─────────
   로켓 전원 ON 이후 순차적으로 진입하는 상태를 가정
   M0: 대기 / M1: 기동 / M2: 비행 / M3: 사출 / M4: 회수 / M5: 셧다운
 ---------------------------------------------------------------
 * ledBuzBegin()
 *   - 모든 GPIO를 OUTPUT 모드로 설정
 *   - 부저를 OFF(HIGH) / LED 전부 OFF(shift595(0))
 * ledBuzUpdate( curMode )
 *   - loop() 또는 타이머 콜백에서 주기적으로 호출 (1~10 ms 권장)
 *   - 현재 모드에 맞춰 LED 2 Hz 점멸, 순환, 셧다운 패턴 실행
 *   - BUZZER 주기(1 s 주기 100 ms 등) 또한 비차단 FSM으로 제어
 * (선택) shift595()
 *   - 외부 모듈이 직접 8 bit 패턴을 출력하고 싶을 때 호출
 *   - 내부에서 shiftOut() + 래치 펄스를 수행
 * ------------------------------------------------------------- */
#ifndef LED_BUZZER_H
#define LED_BUZZER_H

/* ── 매크로·상수 ───────────────────────── */
#ifndef _BV
#define _BV(b) (1U << (b))
#endif

#include <Arduino.h>

constexpr uint8_t PIN_SER   = 22;   // DS  (yellow jumper)
constexpr uint8_t PIN_SRCLK = 23;   // SHCP(green  jumper)
constexpr uint8_t PIN_RCLK  = 24;   // STCP(blue   jumper)
constexpr uint8_t PIN_BUZ   =  8;   // 부저 제어 (LOW→ON)
constexpr uint16_t BUZ_FREQ = 2700;   // 2.7 kHz (피에조 공진 근처)

constexpr uint8_t LED_R = 5;        // Q0 : RED
constexpr uint8_t LED_G = 6;        // Q1 : GREEN
constexpr uint8_t LED_Y = 7;        // Q2 : YELLOW

enum AvionicsMode : uint8_t { M0, M1, M2, M3, M4, M5 };

void ledBuzBegin();
void ledBuzUpdate(AvionicsMode curMode);
void shift595(uint8_t value);

#endif /* LED_BUZZER_H */
