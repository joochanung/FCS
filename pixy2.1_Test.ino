/* 
준비물: pixy2.1, SG90 tilt용 서보모터, ICSP-pixy2.1 연결 케이블
전기 회로 구성 방법: 1. 아두이노 우노의 ICSP(6핀)랑 pixy2.1(10핀)이랑 연결한다.
                    2. pixy2.1의 6핀은 모터 2개를 연결할 수 있는 부분이다. 지금은 1개만 연결해서 상하로 회전하여 위치를 조정한다.
                    3. pixy2.1이랑 컴퓨터랑 연결한다. 그 후로 pixymon2를 실행해 물체 하나를 학습시킨다.
                    4. 이러면 회로 구성은 완료했고 코드를 직접 실행시키면, 모터가 상화로 회전하면서 pixy2.1도 상하로 회전해서 물체를 추적한다.

PS: 서보 핀 9번으로 설정했는데, 사실은 pixy2.1에 직접 연결한 모터인데 이것을 지우면 오류가 생겨서 일단 놔둠.
이것은 pixy2.1만 테스트한 것으로 중간 발표용으로 써지 않아도 됨.
*/

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
  static int i = 0;
  int j;
  char buf[64]; 
  int32_t panOffset, tiltOffset;
  
  // Pixy2에서 활성 블록 가져오기
  pixy.ccc.getBlocks();
  
  if (pixy.ccc.numBlocks)
  {        
    i++;
    
    if (i % 60 == 0)
      Serial.println(i);   
    
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
   
#if 0 // 디버깅용
    sprintf(buf, "%ld %ld %ld %ld", panLoop.m_command, tiltLoop.m_command, servoXPos, servoYPos);
    Serial.println(buf);   
#endif

  }  
  else // 객체가 인식되지 않으면 리셋 상태로 진입
  {
    panLoop.reset();
    tiltLoop.reset();
    servoX.write(90); // 초기 위치로 설정
    servoY.write(90); // 초기 위치로 설정
  }
}

