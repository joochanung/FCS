/* 
준비물: pixy2.1, SG90 tilt용 서보모터, MG996R pan용 서보모터, ICSP-pixy2.1 연결 케이블, VL53L1X 센서
전기 회로 구성 방법: 1. 아두이노 우노의 ICSP(6핀)랑 pixy2.1(10핀)이랑 연결한다.
                    2. pixy2.1의 6핀은 모터 2개를 연결할 수 있는 부분이다. SG90, MG996R 모터 1개씩 연결한다.
                    3. pixy2.1이랑 컴퓨터랑 연결한다. 그 후로 pixymon2를 실행해 물체 하나를 학습시킨다. (학습 방법: Action/Set Signature 1을 선택해 물체를 선택한다.)
                    4. 이러면 pixy2.1 회로 구성은 완료된다. 그 다음에 VL53L1X의 연결 방법을 설명하겠다. 
                    5. VCC: 5V, GND: GND, SDA: A4 pin, SCL: A5 pin, XSHUT: D3 pin, GPI01: D2 pin
                       (VL53L1X pin: Uno pin으로 표현)
                    6. pixy2.1 근처에 VL53L1X를 설치해 거리 오차를 최대한 줄인다.
                    7. 코드를 실행하면 pixy2.1은 물체를 추적하는 동시에 VL53L1X로 거리 측정을 한다.
*/

#include "Adafruit_VL53L1X.h"
#include <Pixy2.h>
#include <PIDLoop.h>

#define IRQ_PIN 2
#define XSHUT_PIN 3

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);
Pixy2 pixy;
// PID 제어 루프를 초기화
/* P: 오류에 비례하는 제어 출력
   I: 오류의 누적 값에 비례하는 제어 출력
   D: 오류의 변화율에 비례하는 제어 출력
   적분 제한 플래그: 적분 제어기의 누적 값이 특정 한도를 초과하지 않도록 제한
*/
PIDLoop panLoop(300, 0, 300, true);
PIDLoop tiltLoop(300, 0, 300, true);

unsigned long lastValidTime = 0; // 마지막으로 유효한 블록을 감지한 시간
unsigned long timeout = 1000;    // 타임아웃 시간 (밀리초)

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(100);

  // VL53L1X 센서 초기화
  Wire.begin();
  
  // 초기화가 실패한 경우, Error 발생
  if (!vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1) delay(100); // 프로그램이 진행되는 것을 막음
  }
  // 측정 시작이 안 되는 경우, Error 발생
  if (!vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1) delay(100); // 프로그램이 진행되는 것을 막음
  }
  
  // 50ms 주기로 VL53L1X에서 거리 측정함
  vl53.setTimingBudget(50);
  
  // Pixy2 초기화
  pixy.init();
  pixy.changeProg("color_connected_components");
}

void loop() {
  static int i = 0;
  int j;
  char buf[64]; 
  int32_t panOffset, tiltOffset;
  
  // Pixy2 블록 데이터 가져오기
  pixy.ccc.getBlocks();
  
  if (pixy.ccc.numBlocks) {
    i++;
    lastValidTime = millis(); // 마지막 유효 시간 업데이트

    // 팬 및 틸트 오류 계산
    panOffset = (int32_t)pixy.frameWidth / 2 - (int32_t)pixy.ccc.blocks[0].m_x;
    tiltOffset = (int32_t)pixy.ccc.blocks[0].m_y - (int32_t)pixy.frameHeight / 2;  
    
    // 루프 업데이트
    panLoop.update(panOffset);
    tiltLoop.update(tiltOffset);
    
    // 팬 및 틸트 서보 모터 위치 이동 (pixy2.1 화면의 중앙으로 이동)
    pixy.setServos(panLoop.m_command, tiltLoop.m_command);
  } 
  // 일정 시간 동안 블록을 감지하지 못한 경우 초기화
  else if (millis() - lastValidTime > timeout) { 
    panLoop.reset();
    tiltLoop.reset();
    pixy.setServos(panLoop.m_command, tiltLoop.m_command); // 서보 모터 위치 초기화
  }
  
  // VL53L1X 거리 측정
  int16_t distance;
  if (vl53.dataReady()) {
    distance = vl53.distance();
    // 거리 측정이 안 되는 경우
    if (distance == -1) {
      Serial.print(F("Couldn't get distance: "));
      Serial.println(vl53.vl_status);
    }
    //거리 측정이 되는 경우
    else {
      Serial.print(F("Distance: "));
      Serial.print(distance);
      Serial.println(" mm");
    }

    // 다음 데이터가 준비될 때 인터럽트를 발생
    vl53.clearInterrupt();
  }
}
