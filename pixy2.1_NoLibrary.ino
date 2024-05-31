// pixy2.1 물체 추적 코드에서 라이브러리 최대한 없는 방향으로 진행
// 핀이 현재 9, 10번을 설정되어 있는데, 5, 6번(OC0B, OC0A)로 변경 바람

#include <Pixy2.h>
#include <PIDLoop.h>

Pixy2 pixy;
PIDLoop panLoop(150, 0, 600, true);  // 조정된 PID 파라미터
PIDLoop tiltLoop(300, 0, 400, true); // 조정된 PID 파라미터

// 서보 모터 제어용 상수
#define SERVO_REFRESH_INTERVAL 20000  // 서보 모터 리프레시 주기 (마이크로초)

// 서보 모터 핀 정의
const int panServoPin = 9;  // Timer1을 사용하기 위해 9번 핀 사용
const int tiltServoPin = 10; // Timer1을 사용하기 위해 10번 핀 사용

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");

  // Pixy2 초기화
  pixy.init();
  pixy.changeProg("color_connected_components");

  // 서보 모터 제어를 위한 Timer1 설정
  pinMode(panServoPin, OUTPUT);
  pinMode(tiltServoPin, OUTPUT);
  
  // Timer1 설정 (서보 모터 제어용 PWM 생성)
  TCCR1A = (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1); // Fast PWM, non-inverted
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);   // Prescaler 8
  ICR1 = SERVO_REFRESH_INTERVAL;  // TOP 값 설정 (20ms 주기)
  
  // 초기 위치 설정 (팬: 0도, 틸트: 180도)
  int panInitialPulseWidth = map(0, 0, 180, -3000, 5000);
  int tiltInitialPulseWidth = map(180, 0, 180, 544, 2400);

  OCR1A = panInitialPulseWidth; // 팬 서보 모터 초기화 (0도 위치)
  OCR1B = tiltInitialPulseWidth; // 틸트 서보 모터 초기화 (180도 위치)
}

void loop()
{  
  int32_t panOffset, tiltOffset, newPanOffset;
  int32_t oldPanOffset = -1;

  // Pixy에서 블록 데이터 가져오기
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks)
  {        
    // 팬 및 틸트 오프셋 계산
    newPanOffset = (int32_t)pixy.frameWidth / 2 - (int32_t)pixy.ccc.blocks[0].m_x;

    if(newPanOffset > oldPanOffset/2){
      panOffset = newPanOffset;
    } else if(oldPanOffset == -1) {
      panOffset = newPanOffset;
    }
    oldPanOffset = panOffset;
    tiltOffset = (int32_t)pixy.ccc.blocks[0].m_y - (int32_t)pixy.frameHeight / 2;  

    // PID 루프 업데이트
    panLoop.update(panOffset);
    tiltLoop.update(tiltOffset);

    // PID 루프에서 팬 및 틸트 명령 가져오기
    int panCommand = panLoop.m_command;
    int tiltCommand = tiltLoop.m_command;

    // 명령 값 제한
    panCommand = constrain(panCommand, -400, 400);
    tiltCommand = constrain(tiltCommand, -400, 400);

    // 명령 값을 서보 모터 펄스 길이로 매핑
    int panPulseWidth = map(panCommand, -400, 400, -3000, 5000);
    int tiltPulseWidth = map(tiltCommand, -400, 400, 544, 2400);

    // 서보 모터 제어
    OCR1A = constrain(panPulseWidth, -3000, 5000);
    OCR1B = constrain(tiltPulseWidth, 544, 2400);

    // 팬 서보 모터의 각도 계산
    int panAngle = calculateAngle(panPulseWidth, -3000, 5000, -180, 180);

    // 틸트 서보 모터의 각도 계산
    int tiltAngle = calculateAngle(tiltPulseWidth, 544, 2400, 0, 180);

    Serial.print("Pan Angle: ");
    Serial.println(panAngle);
    Serial.print("Tilt Angle: ");
    Serial.println(tiltAngle);
  }
}

int calculateAngle(int pulseWidth, int minPulseWidth, int maxPulseWidth, int minAngle, int maxAngle) {
  // 펄스 폭을 각도로 변환
  return map(pulseWidth, minPulseWidth, maxPulseWidth, minAngle, maxAngle);
}
