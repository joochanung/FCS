// FCS MASTER BOARD

#include <SoftwareSerial.h>

// #define BUZZERPIN A0
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
  PORTB &= ~(1 << SS);

  // high & low bytes 변환
  SPDR = highByte;
  while (!(SPSR & (1 << SPIF))) ; // 다른 변환 완료까지 대기
  SPDR = lowByte;
  while (!(SPSR & (1 << SPIF))) ; // 다른 변환 완료까지 대기
  
  // Deselect slave
  PORTB |= (1 << SS);

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
}

void DCmotorON() {
  // 모터 A 제어
  PORTB &= ~(1 << IN1); // in1Pin = LOW
  PORTD |= (1 << IN2);  // in2Pin = HIGH
  OCR1A = 511;          // enaPin = 255 (최대 속도)

  // 모터 B 제어 (in3Pin = HIGH, in4Pin = LOW, enbPin = 255)
  PORTD |= (1 << IN3);  // in3Pin = HIGH
  PORTD &= ~(1 << IN4); // in4Pin = LOW
  OCR0A = 100;          // enbPin = 255 (최대 속도)
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
  DDRD &= ~(1 << BUZZERPWM); // PD3 (피에조 부저 핀)을 출력으로 설정
  PORTD &= ~(1 << BUZZERPWM);

  EICRA |= (1 << ISC01); // 인터럽트 신호의 하강 에지에서 인터럽트 발생
  EIMSK |= (1 << INT0); // INT0 인터럽트 활성화

  // 타이머2 설정 - 피에조 부저 주파수 설정
  TCCR2A |= (1 << WGM21) | (1 << WGM20); // Fast PWM 모드
  TCCR2A |= (1 << COM2B1); // 비교 일치 시 비반전 출력
  TCCR2B |= (1 << CS21); // 분주비 8 설정
  OCR2A = (16000000 / (2 * 8 * frequency)) - 1; // 주파수 설정
  OCR2B = OCR2A / 10; // 520% 듀티 사이클 설정
}

// Intterupt 단계
ISR(INT0_vect) {
  // 인터럽트가 발생하면 피에조 부저를 켭니다
  DDRD |= (1 << BUZZERPWM);
  PORTD |= (1 << BUZZERPWM);

  for(uint32_t j = 0; j < 5; j++) {
    for (uint32_t i = 0; i < 64000; i++) {
      asm("nop");
    }
  }
  
  DDRD &= ~(1 << BUZZERPWM);
  PORTD &= ~(1 << BUZZERPWM);
}
