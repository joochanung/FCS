// FCS MASTER BOARD

#include <SoftwareSerial.h>

// #define BUZZERPIN A0
#define LASERPIN A0
#define BTRXD A2
#define BTTXD A3

#define SCK PB5
#define MISO PB4
#define MOSI PB3
#define SS PB2

#define ENA PB1
#define IN1 PB0
#define IN2 PD7
#define IN4 PD4
#define IN3 PD5
#define ENB PD6

const int buzzerPin = A0;
const int inputPin = 2;

SoftwareSerial btserial(BTTXD, BTRXD); // 블루투스 핀 설정

const int frequency = 392; // 피에조 부저 주파수 설정

// Setup 단계
void setup() {
  // SPI 설정
  DDRB |= (1 << SS) | (1 << MOSI) | (1 << SCK); // SS, MOSI, SCK 핀을 출력 모드로 설정
  SPCR &= ~(1 << SPE); // SPI 비활성화
  SPCR |= (1 << MSTR) | (1 << SPR0); // 마스터 모드, F_CPU/16 속도
  PORTB |= (1 << SS); // SLAVE 보드 비활성화

  btserial.begin(9600);
  Serial.begin(9600);

  // 모터 핀을 출력 모드로 설정
  DDRB |= (1 << ENA) | (1 << IN1);
  DDRD |= (1 << IN2) | (1 << ENB) | (1 << IN3) | (1 << IN4);

  // 초기 값 설정
  PORTB &= ~(1 << IN1); // in1Pin = LOW
  PORTD &= ~(1 << IN2); // in2Pin = LOW
  PORTD &= ~(1 << IN3); // in3Pin = LOW
  PORTD &= ~(1 << IN4); // in4Pin = LOW
  OCR1A = 0; // ENA PWM 초기값 0
  OCR0A = 0; // ENB (PD6) PWM 초기값 0

  // PWM 설정
  // TIMER1 세팅 (MOTOR_A)
  TCCR1A |= (1 << COM1A1); // 비반전 모드
  TCCR1A |= (1 << WGM11) | (1 << WGM10); // Fast PWM, 8비트
  TCCR1B |= (1 << WGM12) | (1 << CS11); // 분주비 8

  // TIMER0 세팅 (MOTOR_B)
  TCCR0A |= (1 << COM0A1); // 비반전 모드
  TCCR0A |= (1 << WGM01) | (1 << WGM00); // Fast PWM, 8비트
  TCCR0B |= (1 << CS01); // 분주비 8

  // 레이저 모듈
  DDRC |= (1 << LASERPIN); 
  PORTC &= ~(1 << LASERPIN);
}

void loop() {
  if (btserial.available()) {
    char cmd = (char)btserial.read();
    int status = cmd - '0';
    Serial.println(status);

    if(status == 0) { // 정지
      PORTC &= ~(1 << LASERPIN); // LASER MODULE 비활성화
      DCmotorOFF();  
    }

    else if(status == 1) { // 전진
      PORTC &= ~(1 << LASERPIN); // LASER MODULE 비활성화
      DCmotorON();
    }
    
    else if(status == 2) { // 기동간 사격
      SPCR |= (1 << SPE); // SPI 활성화
      sendDataToSlave(status); // SLAVE 보드에 전달
      SPCR &= ~(1 << SPE); // SPI 비활성화
      PORTC |= (1 << LASERPIN); // LASER MODULE 활성화
      DCmotorON();
    }
  }
}

// Function 단계
void sendDataToSlave(int data) {
  byte highByte = (data >> 8) & 0xFF; // Upper 8 bits
  byte lowByte = data & 0xFF;         // Lower 8 bits

  // Select slave
  PORTB &= ~(1 << PB2);

  // Transfer high and low bytes
  SPDR = highByte;
  while (!(SPSR & (1 << SPIF))) ; // Wait for transmission complete
  SPDR = lowByte;
  while (!(SPSR & (1 << SPIF))) ; // Wait for transmission complete
  
  // Deselect slave
  PORTB |= (1 << PB2);

  Serial.print("Sent data: ");
  Serial.println(data);
}

void DCmotorON() {
  // 모터 A 제어
  PORTB &= ~(1 << PB0); // in1Pin = LOW
  PORTD |= (1 << PD7);  // in2Pin = HIGH
  OCR1A = 511;          // enaPin = 255 (최대 속도)

  // 모터 B 제어 (in3Pin = HIGH, in4Pin = LOW, enbPin = 255)
  PORTD |= (1 << PD5);  // in3Pin = HIGH
  PORTD &= ~(1 << PD4); // in4Pin = LOW
  OCR0A = 255;          // enbPin = 255 (최대 속도)
}

void DCmotorOFF() {
  // 모터 A 제어
  PORTB &= ~(1 << PB0); // in1Pin = LOW
  PORTD &= ~(1 << PD7); // in2Pin = LOW
  OCR1A = 0;          

  // 모터 B 제어
  PORTD &= ~(1 << PD5); // in3Pin = LOW
  PORTD &= ~(1 << PD4); // in4Pin = LOW
  OCR0A = 0;   
}
