// FCS MASTER BOARD
#include <SoftwareSerial.h>

#define LASERPIN 0 
#define BTRXD A2
#define BTTXD A3
#define BUZZER_INT PD2
#define BUZZERPWM PD3
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

  // DC 모터 초기 설정
  Motor_init();

  // LASERPIN 초기 설정
  DDRC |= (1 << LASERPIN);
  PORTC &= ~(1 << LASERPIN);

  // 피에조 부저 초기 설정
  Piezo_init();
}

// Loop 단계
void loop() {
  // 스마트폰으로부터 값 받아오기 (블루투스 통신 이용)
  if (btserial.available()) {
    char cmd = (char)btserial.read();
    int status = cmd - '0';
    Serial.println(status);

    if(status == 0) { // 정지
      PORTC &= ~(1 << LASERPIN); // LASER MODULE 비활성화
      DCmotorOFF(); // DC 모터 정지
    }

    else if(status == 1) { // 전진
      PORTC &= ~(1 << LASERPIN); // LASER MODULE 비활성화
      DCmotorON(); // DC 모터 가동
    }
    
    else if(status == 2) { // 기동간 사격
      SPCR |= (1 << SPE); // SPI 활성화
      sendDataToSlave(status); // SLAVE 보드에 전달
      SPCR &= ~(1 << SPE); // SPI 비활성화
      PORTC |= (1 << LASERPIN); // LASER MODULE 활성화
      DCmotorON(); // DC 모터 가동
    }

    else if(status == 3){ // 정지 및 조준 모드
      SPCR |= (1 << SPE); // SPI 활성화
      sendDataToSlave(status); // SLAVE 보드에 전달
      SPCR &= ~(1 << SPE); // SPI 비활성화
      PORTC |= (1 << LASERPIN); // LASER MODULE 활성화
      DCmotorOFF(); // DC 모터 정지
    }
  }
}

// SPI를 이용해 slave에게 데이터 전송
void sendDataToSlave(int data) {
  byte highByte = (data >> 8) & 0xFF; // Upper 8 bits
  byte lowByte = data & 0xFF;         // Lower 8 bits

  PORTB &= ~(1 << SS); // SLAVE 보드 활성화

  // high & low bytes 변환
  SPDR = highByte;
  while (!(SPSR & (1 << SPIF))) ; // 다른 변환 완료까지 대기
  SPDR = lowByte;
  while (!(SPSR & (1 << SPIF))) ; // 다른 변환 완료까지 대기
  
  PORTB |= (1 << SS); // SLAVE 보드 비활성화

  // 시리얼 모니터에서 SLAVE로 데이터 전송 확인
  Serial.print("Sent data: ");
  Serial.println(data);
}

void Motor_init(){
  // 모터 핀을 출력 모드로 설정
  DDRB |= (1 << ENA) | (1 << IN1);
  DDRD |= (1 << IN2) | (1 << ENB) | (1 << IN3) | (1 << IN4);

  // 초기 값 설정
  PORTB &= ~(1 << IN1); // in1Pin = LOW
  PORTD &= ~(1 << IN2); // in2Pin = LOW
  PORTD &= ~(1 << IN3); // in3Pin = LOW
  PORTD &= ~(1 << IN4); // in4Pin = LOW
  OCR1A = 0; // ENA (PB1) PWM 초기값 0
  OCR0A = 0; // ENB (PD6) PWM 초기값 0

  // PWM 설정
  // TIMER1 세팅 (MOTOR_A)
  TCCR1A |= (1 << COM1A1); // COM1A 10 >> Clear OC1A/OC1B on Compare Match, set OC1A/OC1B at BOTTOM (non-inverting mode)
  TCCR1A |= (1 << WGM10);
  TCCR1B |= (1 << WGM12) | (1 << CS11); // WGM1 0101 >> Fast PWM, 8-bit, 0x00FF~BOTTOM, TOP / CS1 010 >> prescaler 8

  // TIMER0 세팅 (MOTOR_B)
  TCCR0A |= (1 << COM0A1); // COM0A 10 >> Clear OC0A on Compare Match, set OC0A at BOTTOM (non-inverting mode)
  TCCR0A |= (1 << WGM01) | (1 << WGM00); // WGM0 011 >> Fast PWM, 0xFF~BOTTOM, MAX
  TCCR0B |= (1 << CS01); // CS0 010 >> prescaler 8
}

void DCmotorON() {
  // 모터 A 제어
  PORTB &= ~(1 << IN1); // in1Pin = LOW
  PORTD |= (1 << IN2);  // in2Pin = HIGH
  OCR1A = 150;          // enaPin = 150

  // 모터 B 제어 
  PORTD |= (1 << IN3);  // in3Pin = HIGH
  PORTD &= ~(1 << IN4); // in4Pin = LOW
  OCR0A = 150;          // enbPin = 150
}

void DCmotorOFF() {
  // 모터 A 제어
  PORTB &= ~(1 << IN1); // in1Pin = LOW
  PORTD &= ~(1 << IN2); // in2Pin = LOW
  OCR1A = 0;          

  // 모터 B 제어
  PORTD &= ~(1 << IN3); // in3Pin = LOW
  PORTD &= ~(1 << IN4); // in4Pin = LOW
  OCR0A = 0;   
}

void Piezo_init() {
  // 피에조 부저 관련 추가 코드
  DDRD &= ~(1 << BUZZER_INT); // PD2 (인터럽트 핀)를 입력으로 설정
  PORTD |= (1 << BUZZER_INT); // PD2 핀에 풀업 저항 설정
  DDRD &= ~(1 << BUZZERPWM); // PD3 (피에조 부저 핀)을 입력으로 설정
  PORTD &= ~(1 << BUZZERPWM);

  EICRA |= (1 << ISC01); // 인터럽트 신호의 Falling edge에서 인터럽트 발생
  EIMSK |= (1 << INT0); // INT0 인터럽트 활성화

  // 타이머2 설정 - 피에조 부저 주파수 설정
  TCCR2A |= (1 << WGM21) | (1 << WGM20); // WGM2 011 >> Fast PWM, 0xFF, BOTTOM, MAX 
  TCCR2A |= (1 << COM2B1); // COM2B 10 >> Clear OC2B on Compare Match, set OC2B at BOTTOM (non-inverting mode)
  TCCR2B |= (1 << CS21); // CS2 010 >> prescaler 8
  OCR2A = (16000000 / (2 * 8 * frequency)) - 1; // 주파수 설정
  OCR2B = OCR2A / 10; // 10% 듀티 사이클 설정 (소리 크기 조절)
}

// Intterupt 단계
ISR(INT0_vect) {
  // 인터럽트가 발생하면 피에조 부저를 킴
  DDRD |= (1 << BUZZERPWM);
  PORTD |= (1 << BUZZERPWM);
  
  for(uint32_t j = 0; j < 5; j++) {
    for (uint32_t i = 0; i < 64000; i++) {
      asm("nop");
    }
  }
  
  // 일정 시간 지나면 피에조 부저를 끔
  DDRD &= ~(1 << BUZZERPWM);
  PORTD &= ~(1 << BUZZERPWM);
}
