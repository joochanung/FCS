// VL53L1X 라이브러리 예제 문제로 중간 발표에 포함 안 해도 됨.

#include "Adafruit_VL53L1X.h"

#define IRQ_PIN 2      // IRQ: Interrupt Request
#define XSHUT_PIN 3    // XSHUT: Shutdown (활성화 or 비활성화 가능)

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

void setup() {
  Serial.begin(115200); // 시리얼 통신 설정
  while (!Serial) delay(100);

  Serial.println(F("Adafruit VL53L1X sensor demo"));

  Wire.begin();  // I2C 통신 초기화
  // 센서의 초기화가 실패한 경우
  if (! vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(100);   // 프로그램 종료 방지
  }
  // 초기화 완료하면 준비가 완료된 것임
  Serial.println(F("VL53L1X sensor OK!"));

  Serial.print(F("Sensor ID: 0x"));
  Serial.println(vl53.sensorID(), HEX);

  // 거리 측정이 불가능한 경우
  if (! vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1)       delay(100);   // 프로그램 종료 방지
  }
  // 거리 측정 준비가 완료된 경우
  Serial.println(F("Ranging started"));

  // 타이밍 예산 50ms로 설정
  vl53.setTimingBudget(50);
  Serial.print(F("Timing budget (ms): "));
  Serial.println(vl53.getTimingBudget());
}

void loop() {
  int16_t distance;

  // 새로운 측정 데이터가 준비되었는지 확인
  if (vl53.dataReady()) {
    // 준비된 경우 새롭게 거리 측정
    distance = vl53.distance();
    if (distance == -1) {
      // 오류가 생긴 경우(-1로 출력되는 경우)
      Serial.print(F("Couldn't get distance: "));
      Serial.println(vl53.vl_status);
      return;
    }

    // 측정된 경우 시리얼 모니터에 출력
    Serial.print(F("Distance: "));
    Serial.print(distance);
    Serial.println(" mm");

    // 인터럽트 clear, 다음 측정 준비
    vl53.clearInterrupt();
  }
}
