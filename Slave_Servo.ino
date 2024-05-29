// SPI + 서보 모터 테스트 코드
// 서보 모터가 회전한 각도를 Master 아두이노로부터 값을 받아와 Serial 모니터에 출력 & Slave에 있는 서보모터에 받아온 값의 크기만큼 회전시키기

#include <SPI.h>
#include <Servo.h>

Servo servo;

volatile bool newAngleReceived = false;
volatile int receivedAngle = 0;

void setup() {
  DDRB |= (1 << MISO);       // MISO 핀을 출력으로 설정
  SPCR |= (1 << SPE);        // SPI 활성화
  SPCR |= (1 << SPIE);       // SPI 인터럽트 활성화

  Serial.begin(9600);
  Serial.println("Slave ready");

  servo.attach(8);
  servo.write(0);
}

void loop() {
  if (newAngleReceived) {
    Serial.print("Received angle: ");
    Serial.println(receivedAngle);     // Serial 모니터에 Master로부터 받아온 값 출력
    newAngleReceived = false;          // 플래그 초기화
    servo.write(receivedAngle);        // 받아온 값으로 Slave에 있는 서보 모터를 회전

  }
}

ISR(SPI_STC_vect) {
  static byte highByte = 0;
  static bool highByteReceived = false;       // 상위 바이트 수신 완료 플래그

  // 상위 바이트 읽기
  if (!highByteReceived) {
    highByte = SPDR;                           // 상위 바이트 읽기
    highByteReceived = true;
  } 
  // 나머지 부분인 하위 바이트 읽기
  else {
    byte lowByte = SPDR;                       // 하위 바이트 읽기
    receivedAngle = (highByte << 8) | lowByte; // 상위 바이트와 하위 바이트 결합
    highByteReceived = false;
    newAngleReceived = true;                   // 데이터 수신 완료 플래그 설정
  }
}
