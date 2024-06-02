// 오늘 해야 할 일: pixy2, VL53L1X, 레이저 서보 모터 구현

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
  int status = receivedData;
  if(status == 1){ // 전진
    // pixy2 활성화
  }
  else if(status == 2){ // 기동간 사격
    // pixy2 활성화 및 추적 시작
    // 레이져 모듈의 서보 모터 추적
  }
  else if(status == 3){ // 안전 모드
    // pixy2 활성화
    // 레이져 모듈의 서보 모터 추적 중지
  }
  else if(status == 4){ // 정지
    // 모든 장치 비활성화
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
