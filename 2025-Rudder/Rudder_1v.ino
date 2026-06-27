#include "Logger.h"
#include "SensorManager.h"
#include "DebugSerial.h"
#include "ModeManager.h"
#include "ServoControl.h"
#include "CommandRx.h"
#include "EjectionCheck.h"
#include "LEDBuzzer.h"

//#define BAUDRATE 115200
//#define SAMPLE_HZ 50

int counterino = 98;
CommandRx commandReceiver(Serial1);  // 전역 객체 선언
SdFat sd;
DataPacket p;                        // SD카드 저장용
ModeManager mode;
ServoControl myServo(9);
bool sdInitialized = false;

void setup() {
  Serial.begin(115200);  // 시리얼 통신 (PC)
  //while (!Serial); //
  myServo.begin();         //
  myServo.close();         // 서보 닫기
  //myServo.open(); 사출
  Serial1.begin(57600);    // 시리얼 통신 (SiK Telemetry)
  Serial.println("CommandRx 수신기 시작...");
  clearI2CBus();           // 센서 I2C오류 방지

  //delay(1000);
  Serial.println("Start up");
  ledBuzBegin();

  initSD();
  //initSDlog(); // 로크파일 만들고 저장 준비
  mode.load(); // 이전 모드 불러오기
  //delay(1000);
  // 현재 모드 출력
  //Serial.print("현재 모드: ");
  //Serial.println(mode.getMode()); 현재모드 가져오는 코드
  //delay(1000);
  // 테스트: 모드 변경
  //mode.nextMode(); 다음 모드 시작
  //mode.save();  //  모드 저장 SD카드에 저장
  //mode.prevMode(); //이전모드로 돌아가기
  //mode.save();  // 저장
  initSensors(); // 센서 세팅 값 입력
}

void loop() {
  //if (Serial.available()) {
  //  String cmd = Serial.readStringUntil('\n');
  //  cmd.trim(); cmd.toUpperCase();
  //  if (cmd == "START" && !logging) startLogging();
  //  else if (cmd == "STOP" && logging) stopLogging();
  
  ledBuzUpdate(static_cast<AvionicsMode>(mode.getMode()));
  //if (!logging) return;

  //if (Serial.available()) {
  //  char ch = Serial.read();
  //  if (ch == '1') {
  //    timer6.start();    // Timer6은 이름상 Timer1
  //  } else if (ch == '2') {
  //    timer7.start();    // Timer7은 이름상 Timer2
  //  } else if (ch == 'a') {
  //    timer6.stop();
  //  } else if (ch == 'b') {
  //    timer7.stop();
  //  }
  //}
  //DataPacket p;

  // 이전 모드 불러오기
  //mode.load();
  //delay(1000);
  // 현재 모드 출력
  //Serial.print("현재 모드: ");
  //Serial.println(mode.getMode());

  commandReceiver.update();//명령 받는코드
  commandReceiver.sendConfirm();// 받은명령 확인 답장코드
  //Serial.println("Rx"); 
  readSensors(p);// IMU DMP 값읽기 p에 저장
  //Serial.println("read"); 
  //writePacket(p);// SD 카드에 데이터 저장 p sd 카드 저장
  //Serial.println("write"); 
  Noseangle(); // 노즈 각도 구하는 고드

  //Serial.println("nose"); 
  //angleCheck();// 노즈 각도 확인후 연속적 설정값 이상일시 서보 작동
  //ZaccCheck(); // z가속도 확인후 timer2 실행
  //Serial.println("angle");
  counterino++; 
   if (counterino >= 100) {
    commandReceiver.sendRxPacket1(0x50, nose_angle_deg, nose_angle_deg);//노즈 각도 지상국 메세지 송신
    commandReceiver.sendRxPacket1(0x50, mode.getMode(), mode.getMode());//모드 지상국 메세지 송신
    counterino = 0;       // 다시 0으로 초기화
  }

  //delay(1000);
  //++p.pkt_no;
  //Serial.print("Pkt#"); Serial.println(p.pkt_no);


  switch (mode.getMode()) {
    case 0:
      myServo.close();         // 서보 닫기
      timer6.stop();           // 카운트 다운 중지
      timer7.stop();
      break;

    case 1:
      if (!sdInitialized) {
        initSDlog();          // 로그 초기화는 한 번만 실행
        sdInitialized = true; // 이후에는 건너뜀
      }
      writePacket(p);         // 로그 sd 카드 기록
      break;

    case 2:
      Noseangle(); // 노즈 각도 구하는 고드
      angleCheck();// 노즈 각도 확인후 연속적 설정값 이상일시 서보 작동
      ZaccCheck(); // z가속도 확인후 timer2 실행
      writePacket(p); // 로그 sd 카드 기록
      break;

    case 3:
      writePacket(p); // 로그 sd 카드 기록
      break;

    case 4:

      break;

    case 5:

      break;

    case 6:

      break;

    default:

      break;
  }


}
