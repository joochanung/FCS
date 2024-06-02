/* 
준비물: pixy2.1, SG90 tilt용 서보모터, ICSP-pixy2.1 연결 케이블
전기 회로 구성 방법: 1. 아두이노 우노의 ICSP(6핀)랑 pixy2.1(10핀)이랑 연결한다.
                    2. pixy2.1의 6핀은 모터 2개를 연결할 수 있는 부분이다. 지금은 1개만 연결해서 상하로 회전하여 위치를 조정한다.
                    3. pixy2.1이랑 컴퓨터랑 연결한다. 그 후로 pixymon2를 실행해 물체 하나를 학습시킨다.
                    4. 이러면 회로 구성은 완료했고 코드를 직접 실행시키면, 모터가 상화로 회전하면서 pixy2.1도 상하로 회전해서 물체를 추적한다.

PS: 서보 핀 9번으로 설정했는데, 사실은 pixy2.1에 직접 연결한 모터인데 이것을 지우면 오류가 생겨서 일단 놔둠.
이것은 pixy2.1만 테스트한 것으로 중간 발표용으로 써지 않아도 됨.
*/

#include <SPI.h>
#include <Pixy2.h>
#include <PIDLoop.h>
#include <Servo.h>

Pixy2 pixy;
Servo servo;

int panPos = 90;  // 초기 서보 위치를 90도로 설정
int servoPin = 9; // 서보 핀을 Arduino의 핀 9로 설정

void setup() {
  pixy.init();
  // Use color connected components program for the pan tilt to track 
  pixy.changeProg("color_connected_components");

  // Attach servos to pins
  panServo.attach(5);
  tiltServo.attach(6);
  panServo.write(0);  // Center position (중앙으로 초기화)
  tiltServo.write(180); // Center position (중앙으로 초기화)
}

void loop()
{  
  int32_t panOffset, tiltOffset;

  // get active blocks from Pixy
  pixy.ccc.getBlocks();
  
  if (pixy.ccc.numBlocks) {
    // 첫 번째 블록의 중심 Y 좌표를 가져옴
    int blockY = pixy.ccc.blocks[0].m_y;
    int centerY = (pixy.frameHeight / 2);  // Pixy2.1의 화면 세로 중심

    // 서보 위치를 제한 범위 내로 유지 (0도 ~ 180도)
    panPos = constrain(panPos, 0, 180);
    
    // 물체가 중앙에서 벗어나 있는 경우 중앙에 위치할 때까지 서보를 움직임
    if (blockY < centerY - 10) {
      while(blockY - centerY <=0){
        panPos += 1; // 물체가 중앙보다 아래에 있으면 서보를 위로 이동
        servo.write(panPos);
        delay(10);
      }
    } 
    else if (blockY > centerY + 10) {
      while(blockY - centerY >= 0){
        panPos -= 1; // 물체가 중앙보다 위에 있으면 서보를 아래로 이동
        servo.write(panPos);
        delay(10);
      }
    }
    delay(10);
  }
  
  delay(20); // 서보 모터의 이동 시간을 위해 딜레이 추가
}
