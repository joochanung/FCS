// 코드를 하나로 합치는 과정에서 반드시 필요한 코드 (아두이노 - 스마트폰 간의 통신 과정)
// HC-05가 잘 작동하는지 확인하기 위해 사용한 코드. 그 뿐만 아니라 스마트폰와의 연결, 스마트폰에서의 입력과 아두이노에서의 출력 결과가 잘 이루어져 있는지 확인하는 과정.
// SPI와 모터에 대한 라이브러리를 이용하지 않고 코드를 구현함.
// 오늘 Master에서는 피에조 부저를 구현하면 아두이노 Master 부분은 완성. Slave 부분에 집중해야 함.

#include <SoftwareSerial.h>

#define BTRXD A2
#define BTTXD A3

// Arduino RX connected to Bluetooth TX, Arduino TX connected to Bluetooth RX
SoftwareSerial btserial(BTTXD, BTRXD); 

// 소리 출력 및 멈춤 시간을 밀리초 단위로 설정
const int soundDuration = 100;  // 소리가 나는 시간 (100ms)
const int silenceDuration = 2000;  // 소리가 안 나는 시간 (2000ms)

// 주파수 설정 (392Hz)
const int frequency = 392;

// 타이머 인터럽트에 사용될 변수
volatile bool soundOn = false;  // 현재 소리 상태를 저장
volatile unsigned long lastToggleTime = 0;

int status;

void setup() {
  // SPI 설정
  // SS, MOSI, SCK 핀을 출력 모드로 설정
  DDRB |= (1 << PB2) | (1 << PB3) | (1 << PB5);

  // SPI 설정
  // SPCR |= (1 << SPE) | (1 << MSTR) | (1 << SPR0); // 초기에는 SPI 활성화하지 않음

  PORTB |= (1 << DDB2);
  btserial.begin(9600);
  Serial.begin(9600);

  // 핀을 출력 모드로 설정
  DDRB |= (1 << PB1) | (1 << PB0); // enaPin (PB1), in1Pin (PB0)
  DDRD |= (1 << PD7) | (1 << PD6) | (1 << PD5) | (1 << PD4); // in2Pin (PD7), enbPin (PD6), in3Pin (PD5), in4Pin (PD4)

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

  // 레이저 모듈
  DDRD |= (1 << PD2);

  // 피에조 부저 모듈 (테스트 필요함)
  DDRD |= (1 << PD3);

  // 타이머 2 설정
  cli(); // 모든 인터럽트를 비활성화

  // 타이머 2를 CTC 모드로 설정
  TCCR2A = (1 << WGM21);
  TCCR2B = (1 << CS22) | (1 << CS20); // 분주율 128

  // 비교 일치 값 설정 (392Hz)
  OCR2A = (F_CPU / (128 * 2 * frequency)) - 1;

  // 비교 일치 인터럽트 허용
  TIMSK2 = (1 << OCIE2A);

  sei(); // 모든 인터럽트를 활성화
}

ISR(TIMER2_COMPA_vect) {
  // 타이머 비교 일치 인터럽트 서비스 루틴
  if (status == 2) {
    if (soundOn) {
      PORTD |= (1 << PD3); // 피에조 부저 켜기
    } else {
      PORTD &= ~(1 << PD3); // 피에조 부저 끄기
    }
  } else {
    PORTD &= ~(1 << PD3); // 피에조 부저 끄기 (다른 상태일 때)
  }
}

void loop() {
  if (btserial.available()) { // Check if there is any data available from Bluetooth
    char cmd = (char)btserial.read(); // Read the data and convert it from char to int
    status = cmd - '0';

    Serial.println(status); // Print received data to the Serial Monitor

    // SPI 활성화
    SPCR |= (1 << SPE) | (1 << MSTR) | (1 << SPR0);

    if (status == 1) { // 전진
      sendDataToSlave(status);
      // 레이저 비활성화
      PORTD &= ~(1 << PD2);
      // 피에조 부저 비활성화
      PORTD &= ~(1 << PD3);

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
      unsigned long currentMillis = millis();

      // 레이저 모듈 활성화
      PORTD |= (1 << PD2);

      // 모터 A 제어 (in1Pin = LOW, in2Pin = HIGH, enaPin = 511)
      PORTB &= ~(1 << PB0); // in1Pin = LOW
      PORTD |= (1 << PD7);  // in2Pin = HIGH
      OCR1A = 511;          // enaPin = 255 (최대 속도)

      // 모터 B 제어 (in3Pin = HIGH, in4Pin = LOW, enbPin = 255)
      PORTD |= (1 << PD5);  // in3Pin = HIGH
      PORTD &= ~(1 << PD4); // in4Pin = LOW
      OCR0A = 255;          // enbPin = 255 (최대 속도)

      // 소리 출력 상태에서 일정 시간이 지나면 소리 끄기
      if (soundOn && (currentMillis - lastToggleTime >= soundDuration)) {
        soundOn = false;
        lastToggleTime = currentMillis;
      }

      // 소리 꺼짐 상태에서 일정 시간이 지나면 소리 켜기
      if (!soundOn && (currentMillis - lastToggleTime >= silenceDuration)) {
        soundOn = true;
        lastToggleTime = currentMillis;
      }

    }
    
    else if(status == 0){ // 정지
      sendDataToSlave(status);
      // 레이저 비활성화
      PORTD &= ~(1 << PD2);
      // 피에조 부저 비활성화
      PORTD &= ~(1 << PD3);

      // 모터 A 제어 (in1Pin = LOW, in2Pin = HIGH, enaPin = 0)
      PORTB &= ~(1 << PB0);  // in1Pin = LOW
      PORTD &= ~(1 << PD7);  // in2Pin = HIGH
      OCR1A = 0;          

      // 모터 B 제어 (in3Pin = HIGH, in4Pin = LOW, enbPin = 0)
      PORTD &= ~(1 << PD5);  // in3Pin = HIGH
      PORTD &= ~(1 << PD4);  // in4Pin = LOW
      OCR0A = 0;          
    }

    // SPI 비활성화
    SPCR &= ~(1 << SPE);
  }
}

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
