#include <Pixy2.h>
#include "Adafruit_VL53L1X.h"

#define IRQ_PIN 2 // IRQ (PD2)
#define XSHUT_PIN 3 // XSHUT (PD3)    
#define servoXPin 5 // X축 서보모터 (PD5)
#define servoYPin 6 // Y축 서보모터 (PD6)
#define servoLPin 9 // 레이저 서보모터 (PB1)

#define SS PB2 // SPI SS핀
#define MOSI PB3 // SPI MOSI핀
#define MISO PB4 // SPI MISO핀
#define SCK PB5 // SPI CLK핀

#define signalPin PD7 // 시그널 전달 핀 (PD7)

// pixy 객체 생성
Pixy2 pixy;

// VL53L1X 객체 생성
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X(XSHUT_PIN, IRQ_PIN);

// Master로부터 데이터 받는 변수 선언
volatile bool newDataReceived = false;
volatile int receivedData;

// pixy2 화면 가로, 세로 선언 & 중앙점 선언(pixel 기준)
const int screenWidth = 316;
const int screenHeight = 208;
const int centerX = screenWidth / 2;
const int centerY = screenHeight / 2;

// 서보모터 움직이는 정도 선언
const int stepSize = 1;

// 초기 서보 모터 위치 선언
int servoXPos = 180;
int servoYPos = 180;
int servoLPos = 170;

int lockon = 0;
int pixyflag = 0;

// 조준 여부 확인하는 bool형 변수
bool lockonX = false;
bool lockonY = false;

void setup() {
  Serial.begin(9600);

  // I2C 통신 초기화
  Wire.begin(); 

  DDRB |= (1 << MISO); // MISO 핀을 출력으로 설정
  SPCR |= (1 << SPE) | (1 << SPIE); // SPI 활성화, SPI 인터럽트 활성화

  // 서보X, 서보Y 세팅
  DDRD |= (1 << servoXPin)|(1 << servoYPin);
  delay(10);
  servo_write(servoXPin, servoXPos);
  servo_write(servoYPin, servoYPos);


  // 서보L 세팅
  DDRB |= (1 << servoLPin);
  delay(10);
  servo_write(servoLPin, servoLPos);


  // VL53L1X 초기화
  VL53L1X_init();

  DDRD |= (1 << signalPin);
}

void loop() {
  // SPI 데이터 수신 처리
  if (newDataReceived) {
    newDataReceived = false;  // 플래그 초기화
  }

  int status = receivedData;

  if (status == 2) { // 기동간 사격
    // Pixy2 활성화
    if (pixyflag == 0) {
      pixy.init();
      pixyflag = 1;
    }

    int i;
    int16_t distance;
    bool blockDetected = false;

    pixy.ccc.getBlocks();
    
    // Pixy2 블록이 감지된 경우
    if (pixy.ccc.numBlocks) {
      Serial.println(pixy.ccc.numBlocks);
      
      // 가장 오른쪽에 있는 블록을 찾기
      int max = 0;
      for (int i = 1; i < pixy.ccc.numBlocks; i++) {
        if (compareBlockSize(pixy.ccc.blocks[i], pixy.ccc.blocks[max])) {
          max = i;
        }
      }

      // 가장 오른쪽에 있는 블록을 맨 처음으로 정렬
      if (max != 0) {
        Block temp = pixy.ccc.blocks[0];
        pixy.ccc.blocks[0] = pixy.ccc.blocks[max];
        pixy.ccc.blocks[max] = temp;
      }
      
      // block의 X, Y 위치
      int blockX = pixy.ccc.blocks[0].m_x;
      int blockY = pixy.ccc.blocks[0].m_y;

      // 중앙 위치까지의 거리
      int errorX = blockX - centerX;
      int errorY = blockY - centerY;

      // 빠르면서 부드러운 서보 모터의 회전을 위해 사용
      // float Kpx = map(abs(errorX), 0, pixy.frameWidth / 2, 1, 4);
      float Kpx = abs(errorX) * (4 - 1) / pixy.frameWidth + 1;
      // float Kpy = map(abs(errorY), 0, pixy.frameHeight / 2, 1, 3);
      float Kpy = abs(errorY) * (3 - 1) / pixy.frameHeight + 1;

      // 서보 모터 위치 조정(x축)
      if (abs(errorX) > 20) {
        if (errorX > 0) {
          servoXPos -= Kpx * stepSize;
        }
        else {
          servoXPos += Kpx * 2 * stepSize;
        }
        lockonX = false;
      }

      else {
        lockonX = true;
      }

      // 서보 모터 위치 조정(y축)
      if (abs(errorY) > 10) {
        if (errorY > 0) {
          servoYPos += Kpy * stepSize;
        }

        else {
          servoYPos -= Kpy * stepSize;
        }
        lockonY = false;
      }
           
      else {
        lockonY = true;
      }

      servoXPos = constrain(servoXPos, 0, 180);
      servoYPos = constrain(servoYPos, 0, 180);
      
      servo_write(servoXPin, servoXPos);
      servo_write(servoYPin, servoYPos);
      // 서보 모터의 위치가 오차 내에 있는 경우
      if (lockonX && lockonY) {
        if (vl53.dataReady()) { // VL53L1X 거리 측정
          distance = vl53.distance(); 
          if (distance == -1) { // 거리 측정이 안 되면 오류 발생
            Serial.println(vl53.vl_status);
            return;
          }

          vl53.clearInterrupt();

          // 레이저 모듈에 설치된 서보 모터 제어
          float y = radians(180 - servoYPos); // 서보 모터 각도를 라디안으로 변환
          float tanYPrime = (distance * sin(y) + 100) / (distance * cos(y));
          float yPrime = atan(tanYPrime); // y' 계산
          servoLPos = 165 - (degrees(yPrime)); // 170 - y'
          servoLPos = constrain(servoLPos, 0, 170); // 유효 범위로 제한

          // VL53L1X 센서 끄기
          PORTD &= ~(1 << XSHUT_PIN);
          delay(30); // 센서가 완전히 꺼질 때까지 잠시 대기
            
          // 레이저 모듈 서보 모터 제어  
          servo_write(servoLPin, servoLPos);
          delay(20); // 서보 모터가 움직일 시간을 줌

          // 피에조 부저 시그널 전달
          if (lockon == 0) {
            DDRD |= (1 << signalPin);
            PORTD |= (1 << signalPin);
            DDRD &= ~(1 << signalPin);
            PORTD &= ~(1 << signalPin);
          }

          // lockon을 통해 피에조 부저의 주기를 조절
          if (lockon < 2) {
            lockon += 1;            
          }

          else if (lockon == 2) {
            lockon = 0;
          }
          
          // VL53L1X 센서 다시 켜기
          PORTD |= (1 << XSHUT_PIN);
            
          // VL53L1X 센서 초기화
          VL53L1X_init();
        }
      }
    }
  }
}

ISR(SPI_STC_vect) {
  static byte highByte = 0;
  static bool highByteReceived = false;

  if (!highByteReceived) {
    highByte = SPDR;             // 상위 바이트 읽기
    highByteReceived = true;
  } 
  else {
    byte lowByte = SPDR;         // 하위 바이트 읽기
    receivedData = (highByte << 8) | lowByte; // 상위 바이트와 하위 바이트 결합
    highByteReceived = false;
    newDataReceived = true;     // 데이터 수신 완료 플래그 설정
  }
}

void VL53L1X_init(){
  // VL53L1X 초기화
  if (!vl53.begin(0x29, &Wire)) {
    Serial.print(F("Error on init of VL sensor: "));
    Serial.println(vl53.vl_status);
    while (1) delay(100);  
  }

  // 거리 측정 시작
  if (!vl53.startRanging()) {
    Serial.print(F("Couldn't start ranging: "));
    Serial.println(vl53.vl_status);
    while (1) delay(100);  
  }

  // 타이밍 예산 50ms로 설정
  vl53.setTimingBudget(50);
}

// 비교 함수 (x축의 값으로 비교)
bool compareBlockSize(const Block &a, const Block &b) {
  int A = a.m_x;
  int B = b.m_x; 
  return A > B;
}

// 서보모터 제어 함
void servo_write(uint8_t Pin, uint8_t Angle) {
  int delayTime;

  if (Pin == servoXPin) {
    delayTime = map(Angle, 0, 180, 500, 2400);
    PORTD |= (1 << Pin);
    delayMicroseconds(delayTime);
    PORTD &= ~(1 << Pin);
    delayMicroseconds(20000-delayTime);
  }
  if (Pin == servoYPin) {
    delayTime = map(Angle, 0, 180, 1000, 2500);
    PORTD |= (1 << Pin);
    delayMicroseconds(delayTime);
    PORTD &= ~(1 << Pin);
    delayMicroseconds(20000-delayTime);
  }
  if (Pin == servoLPin) {
    delayTime = map(Angle, 0, 180, 1000, 2000);
    PORTB |= (1 << Pin);
    delayMicroseconds(delayTime);
    PORTB &= ~(1 << Pin);
    delayMicroseconds(20000-delayTime);
  }
}
