#include <Pixy2.h>
#include <PIDLoop.h>
#include <Servo.h>

Pixy2 pixy;
PIDLoop panLoop(400, 0, 400, true);
PIDLoop tiltLoop(500, 0, 500, true);

Servo servoX; // 서보 모터 X축
Servo servoY; // 서보 모터 Y축

const int servoXPin = 9; // X축 서보 핀
const int servoYPin = 10; // Y축 서보 핀

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");
 
  // Pixy2 초기화
  pixy.init();
  
  // Pixy2를 색상 연결된 구성 요소 프로그램으로 변경
  pixy.changeProg("color_connected_components");
  
  // 서보 모터 초기화
  servoX.attach(servoXPin);
  servoY.attach(servoYPin);
  
  // 서보 초기 위치 설정 (중앙으로 설정)
  servoX.write(90);
  servoY.write(90);
}

void loop()
{  
  int j;
  char buf[64]; 
  int32_t panOffset, tiltOffset;
  
  // Pixy2에서 활성 블록 가져오기
  pixy.ccc.getBlocks();
  
  if (pixy.ccc.numBlocks)
  {        
    // 첫 번째 객체(가장 큰 객체) 기준으로 팬과 틸트 "오류" 계산
    panOffset = (int32_t)pixy.frameWidth / 2 - (int32_t)pixy.ccc.blocks[0].m_x;
    tiltOffset = (int32_t)pixy.ccc.blocks[0].m_y - (int32_t)pixy.frameHeight / 2;  
  
    // PID 루프 업데이트
    panLoop.update(panOffset);
    tiltLoop.update(tiltOffset);

    // 서보 모터 위치 설정
    int servoXPos = 90 + panLoop.m_command / 4; // PID 출력 값을 서보 모터 위치로 변환
    int servoYPos = 90 + tiltLoop.m_command / 4; // PID 출력 값을 서보 모터 위치로 변환
    
    servoX.write(constrain(servoXPos, 0, 180)); // 서보 모터 범위 제한
    servoY.write(constrain(servoYPos, 0, 180)); // 서보 모터 범위 제한
  }  
  else // 객체가 인식되지 않으면 리셋 상태로 진입
  {
    panLoop.reset();
    tiltLoop.reset();
    servoX.write(90); // 초기 위치로 설정
    servoY.write(90); // 초기 위치로 설정
  }
}

