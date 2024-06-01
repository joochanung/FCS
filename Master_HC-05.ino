// 코드를 하나로 합치는 과정에서 반드시 필요한 코드 (아두이노 - 스마트폰 간의 통신 과정)
// HC-05가 잘 작동하는지 확인하기 위해 사용한 코드. 그 뿐만 아니라 스마트폰와의 연결, 스마트폰에서의 입력과 아두이노에서의 출력 결과가 잘 이루어져 있는지 확인하는 과정.
// SPI와 모터에 대한 라이브러리를 이용하지 않고 코드를 구현함.

#include <SoftwareSerial.h>

#define BTRXD A2
#define BTTXD A3

SoftwareSerial btserial(BTTXD, BTRXD); // Arduino RX connected to Bluetooth TX, Arduino TX connected to Bluetooth RX

const int slaveSelectPin = 10;

void setup() {
  // SPI 설정
  // SS, MOSI, SCK 핀을 출력 모드로 설정
  DDRB |= (1 << DDB2) | (1 << DDB3) | (1 << DDB5);

  // SPI 설정
  SPCR |= (1 << SPE) | (1 << MSTR) | (1 << SPR0); // SPI 활성화, 마스터 모드, F_CPU/16 속도

  PORTB |= (1 << DDB2);
  btserial.begin(9600);
  Serial.begin(9600);

  // 핀을 출력 모드로 설정
  DDRB |= (1 << DDB1) | (1 << DDB0); // enaPin (PB1), in1Pin (PB0)
  DDRD |= (1 << DDD7) | (1 << DDD6) | (1 << DDD5) | (1 << DDD4); // in2Pin (PD7), enbPin (PD6), in3Pin (PD5), in4Pin (PD4)

  // 초기 값 설정
  OCR1A = 0; // enaPin (PB1) PWM 초기값 0
  OCR0A = 0; // enbPin (PD6) PWM 초기값 0
  PORTB &= ~(1 << PB0); // in1Pin = LOW
  PORTD &= ~(1 << PD7); // in2Pin = LOW
  PORTD &= ~(1 << PD5); // in3Pin = LOW
  PORTD &= ~(1 << PD4); // in4Pin = LOW

  // PWM 설정
  // Timer 1 설정 (PB1)
  TCCR1A |= (1 << COM1A1); // 비반전 모드
  TCCR1A |= (1 << WGM11) | (1 << WGM10); // Fast PWM, 8비트
  TCCR1B |= (1 << WGM12) | (1 << CS11); // 분주비 8

  // Timer 0 설정 (PD6)
  TCCR0A |= (1 << COM0A1); // 비반전 모드
  TCCR0A |= (1 << WGM01) | (1 << WGM00); // Fast PWM, 8비트
  TCCR0B |= (1 << CS01); // 분주비 8

  // 레이저 모듈 (테스트 필요함)
  DDRD |= (1 << DDD2);

  // 피에조 부저 모듈 (태스트 필요함)
  DDRD |= (1 << DDD3);
}

void loop() {
  if (btserial.available()) { // Check if there is any data available from Bluetooth
    char cmd = (char)btserial.read(); // Read the data and convert it from char to int
    int status = cmd - '0';
    Serial.println(status); // Print received data to the Serial Monitor
    if (status == 1) { // 전진
      sendDataToSlave(status);
      
      // 모터 A 제어 (in1Pin = LOW, in2Pin = HIGH, enaPin = 255)
      PORTB &= ~(1 << PB0); // in1Pin = LOW
      PORTD |= (1 << PD7);  // in2Pin = HIGH
      OCR1A = 511;          // enaPin = 255 (최대 속도)

      // 모터 B 제어 (in3Pin = HIGH, in4Pin = LOW, enbPin = 255)
      PORTD |= (1 << PD5);  // in3Pin = HIGH
      PORTD &= ~(1 << PD4); // in4Pin = LOW
      OCR0A = 255;          // enbPin = 255 (최대 속도)
    }
    else if(status == 2){ // 기동간 사격
      sendDataToSlave(status);
      // 피에조 부저 활성화 (추가 예정)
      // 레이저 모듈 활성화 (테스트 필요함)
      PORTD |= (1 << PD2);
    }
    else if(status == 3){ // 안전 모드
      sendDataToSlave(status);
      // 피에조 부저 비활성화 (추가 예정)
      // 레이저 모듈 비활성화 (테스트 필요함)
      PORTD &= ~(1 << PD2);

      // 모터 A 제어 (in1Pin = LOW, in2Pin = HIGH, enaPin = 255)
      PORTB &= ~(1 << PB0); // in1Pin = LOW
      PORTD |= (1 << PD7);  // in2Pin = HIGH
      OCR1A = 511;          // enaPin = 255 (최대 속도)

      // 모터 B 제어 (in3Pin = HIGH, in4Pin = LOW, enbPin = 255)
      PORTD |= (1 << PD5);  // in3Pin = HIGH
      PORTD &= ~(1 << PD4); // in4Pin = LOW
      OCR0A = 255;          // enbPin = 255 (최대 속도)
    }
    else if(status == 4){ // 정지
      sendDataToSlave(status);
      // 모터 A 제어 (in1Pin = LOW, in2Pin = HIGH, enaPin = 255)
      PORTB &= ~(1 << PB0); // in1Pin = LOW
      PORTD &= ~(1 << PD7);  // in2Pin = HIGH
      OCR1A = 0;          // enaPin = 255 (최대 속도)

      // 모터 B 제어 (in3Pin = HIGH, in4Pin = LOW, enbPin = 255)
      PORTD &= ~(1 << PD5);  // in3Pin = HIGH
      PORTD &= ~(1 << PD4); // in4Pin = LOW
      OCR0A = 0;          // enbPin = 255 (최대 속도)
    }
  }
}

void sendDataToSlave(int data) {
  byte highByte = (data >> 8) & 0xFF; // Upper 8 bits
  byte lowByte = data & 0xFF;         // Lower 8 bits

  // Select slave
  PORTB &= ~(1 << DDB2);

  // Transfer high and low bytes
  SPDR = highByte;
  while (!(SPSR & (1 << SPIF))) ; // Wait for transmission complete
  SPDR = lowByte;
  while (!(SPSR & (1 << SPIF))) ; // Wait for transmission complete
  
  // Deselect slave
  PORTB |= (1 << DDB2);

  Serial.print("Sent data: ");
  Serial.println(data);
}
