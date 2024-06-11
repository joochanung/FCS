// pixy2에 VL53L1X 모듈을 합친 코드 (모든 라이브러리 이용) -> 최종에서는 일부 라이브러리는 없이 구현 완료

#include <Pixy2.h>
#include <Servo.h>
#include "Adafruit_VL53L1X.h"

#define IRQ_PIN 2      // IRQ: Interrupt Request for VL53L1X
#define XSHUT_PIN 3    // XSHUT: Shutdown for VL53L1X
#define ADDITIONAL_SERVO_PIN 9  // Additional servo pin

// This is the main Pixy object 
Pixy2 pixy;

// Create servo objects
Servo servoX;
Servo servoY;
Servo additionalServo;

// Define pin numbers for servos
const int servoXPin = 5;
const int servoYPin = 6;

// Servo positions
int servoXPos = 180;
int servoYPos = 180;


// Screen center coordinates
const int screenWidth = 316;
const int screenHeight = 208;
const int centerX = screenWidth / 2;
const int centerY = screenHeight / 2;

// Servo movement increment
const int stepSize = 2;

// VL53L1X object
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

void setup() {
  Serial.begin(9600); // 시리얼 통신 설정
  while (!Serial) delay(100);

  Wire.begin();  // I2C 통신 초기화
  
  // Pixy2 초기화
  pixy.init();
  
  // Attach the servos
  servoX.attach(servoXPin);
  servoY.attach(servoYPin);
  additionalServo.attach(ADDITIONAL_SERVO_PIN);

  // Initialize servo positions
  servoX.write(servoXPos);
  servoY.write(servoYPos);
  additionalServo.write(170);

  // VL53L1X 초기화
  if (!vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1) delay(100);   // 프로그램 종료 방지
  }

  // 거리 측정 시작
  if (!vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1) delay(100);   // 프로그램 종료 방지
  }

  // 타이밍 예산 50ms로 설정
  vl53.setTimingBudget(50);
}

void loop() {
  int i;
  int16_t distance;

  // Pixy2 블록 감지
  pixy.ccc.getBlocks();
  

  
  // Pixy2 블록이 감지된 경우
  if (pixy.ccc.numBlocks) {
    Serial.print("Detected ");
    Serial.println(pixy.ccc.numBlocks);
    for (i = 0; i < pixy.ccc.numBlocks; i++) {
      int blockX = pixy.ccc.blocks[i].m_x;
      int blockY = pixy.ccc.blocks[i].m_y;

      int errorX = blockX - centerX;
      int errorY = blockY - centerY;

      bool lockonX = false;
      bool lockonY = false;

      // 서보 모터 위치 조정
      if (abs(errorX) > 10) {
        if (errorX > 0)
          servoXPos -= stepSize;
        else
          servoXPos += stepSize;
      }

      else{
        lockonX = true;
      }

      if (abs(errorY) > 10) {
        if (errorY > 0)
          servoYPos += stepSize;
        else
          servoYPos -= stepSize;
      }

      else{
        lockonY = true;
      }

      servoXPos = constrain(servoXPos, 0, 180);
      servoYPos = constrain(servoYPos, 0, 180);

      servoX.write(servoXPos);
      servoY.write(servoYPos);

      // VL53L1X 거리 측정
      if (lockonX && lockonY) {
        // VL53L1X 거리 측정
        if (vl53.dataReady()) {
          distance = vl53.distance();
          if (distance == -1) {
            Serial.print(F("Couldn't get distance: "));
            Serial.println(vl53.vl_status);
            return;
          }

          Serial.print(F("Distance: "));
          Serial.print(distance);
          Serial.println(" mm");

          vl53.clearInterrupt();

          // 추가 서보 모터 제어
          float y = radians(180 - servoYPos); // 서보 모터 각도를 라디안으로 변환
          float tanYPrime = (distance * sin(y) + 70) / (distance * cos(y));
          float yPrime = atan(tanYPrime); // y' 계산
          int additionalServoPos = 170 - degrees(yPrime); // 170 - y' (서보 모터 이슈)
          additionalServoPos = constrain(additionalServoPos, 0, 170); // 유효 범위로 제한

          // VL53L1X 센서 끄기
          digitalWrite(XSHUT_PIN, LOW);
          delay(100); // 센서가 완전히 꺼질 때까지 잠시 대기

          // 추가 서보 모터 제어
          additionalServo.write(additionalServoPos);
          delay(20); // 추가 서보 모터가 움직일 시간을 줌

          // VL53L1X 센서 다시 켜기
          digitalWrite(XSHUT_PIN, HIGH);
          delay(100); // 센서가 다시 켜질 때까지 잠시 대기

          // VL53L1X 센서 재초기화
          if (!vl53.begin(0x29, &Wire)) {
            Serial.print(F("Error on reinit of VL sensor: "));
            Serial.println(vl53.vl_status);
            while (1) delay(100);   // 프로그램 종료 방지
          }

          if (!vl53.startRanging()) {
            Serial.print(F("Couldn't restart ranging: "));
            Serial.println(vl53.vl_status);
            while (1) delay(100);   // 프로그램 종료 방지
          }

          Serial.print("Additional Servo Angle: ");
          Serial.println(additionalServoPos);
        }
      }

      Serial.print("Pan Angle: ");
      Serial.println(servoXPos);
      Serial.print("Tilt Angle: ");
      Serial.println(servoYPos);
    }
  }
}
