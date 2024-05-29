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

#include <Pixy2.h>
#include <PIDLoop.h>
#include <Wire.h>
#include "Adafruit_VL53L1X.h"

#define IRQ_PIN 2
#define XSHUT_PIN 3

// Pixy2 객체 생성
Pixy2 pixy;
PIDLoop panLoop(400, 0, 400, true);
PIDLoop tiltLoop(500, 0, 500, true);

// VL53L1X 객체 생성
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  /*
  Serial.println(F("Starting Pixy2 and VL53L1X sensor demo"));
  */

  // Pixy2 초기화
  pixy.init();
  pixy.changeProg("color_connected_components");

  // VL53L1X 초기화
  Wire.begin();
  /*
  if (!vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error initializing VL53L1X: "));
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }
  Serial.println(F("VL53L1X sensor initialized"));

  // 거리 측정 시작
  if (!vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1) delay(10);
  }
  Serial.println(F("Ranging started"));
  */
  // 타이밍 예산 설정
  vl53.setTimingBudget(50);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(vl53.getTimingBudget());
}

void loop() {
  static int i = 0;
  int32_t panOffset, tiltOffset;
  int16_t distance;

  // Pixy2로부터 블록 데이터를 가져옴
  pixy.ccc.getBlocks();
  
  if (pixy.ccc.numBlocks) {
    i++;
    if (i % 60 == 0) Serial.println(i);

    // 팬 및 틸트 오프셋 계산
    panOffset = (int32_t)pixy.frameWidth / 2 - (int32_t)pixy.ccc.blocks[0].m_x;
    tiltOffset = (int32_t)pixy.ccc.blocks[0].m_y - (int32_t)pixy.frameHeight / 2;

    // PID 루프 업데이트
    panLoop.update(panOffset);
    tiltLoop.update(tiltOffset);

    // 서보 모터 제어
    pixy.setServos(panLoop.m_command, tiltLoop.m_command);

    // VL53L1X 거리 측정
    if (vl53.dataReady()) {
      distance = vl53.distance();
      if (distance == -1) {
        Serial.print(F("Couldn't get distance: "));
        Serial.println(vl53.vl_status);
      } 
      else {
        Serial.print(F("Distance: "));
        Serial.print(distance);
        Serial.println(" mm");
      }
      vl53.clearInterrupt();
    }
  } 
  else {
    // 블록이 인식되지 않으면 팬 및 틸트 루프 리셋
    panLoop.reset();
    tiltLoop.reset();
    pixy.setServos(panLoop.m_command, tiltLoop.m_command);
  }

  // 측정 간격 조절
  delay(100);
}
