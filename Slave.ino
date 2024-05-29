#include <SPI.h>

volatile bool newDataReceived = false;
volatile int receivedData = 0;

void setup() {
  DDRB |= (1 << MISO);       // MISO 핀을 출력으로 설정
  SPCR |= (1 << SPE);        // SPI 활성화
  SPCR |= (1 << SPIE);       // SPI 인터럽트 활성화

  Serial.begin(9600);
}

void loop() {
  if (newDataReceived) {
    Serial.print("Received data: ");
    Serial.println(receivedData);
    newDataReceived = false;  // 플래그 초기화
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
