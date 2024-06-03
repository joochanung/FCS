#include <SoftwareSerial.h>

#define BTRXD A2
#define BTTXD A3

const int buzzerPin = A0;
const int inputPin = 2;

// Arduino RX connected to Bluetooth TX, Arduino TX connected to Bluetooth RX
SoftwareSerial btserial(BTTXD, BTRXD); 

/*
// 소리 출력 및 멈춤 시간을 밀리초 단위로 설정
const int soundDuration = 100;  // 소리가 나는 시간 (100ms)
const int silenceDuration = 2000;  // 소리가 안 나는 시간 (2000ms)

// 주파수 설정 (392Hz)
const int frequency = 392;

// 타이머 인터럽트에 사용될 변수
volatile bool soundOn = false;  // 현재 소리 상태를 저장
volatile unsigned long lastToggleTime = 0;
volatile bool maintainStatus2 = false;  // 상태 2 유지 상태를 저장
*/

void setup() {

  pinMode(buzzerPin, OUTPUT);
  pinMode(inputPin, INPUT);

  // 외부 인터럽트 설정
  EICRA |= (1 << ISC01);  // 상승 에지에서 인터럽트 발생 (ISC01 = 1, ISC00 = 0)
  EIMSK |= (1 << INT0);   // INT0 인터럽트 활성화

  // SPI 설정
  // SS, MOSI, SCK 핀을 출력 모드로 설정
  DDRB |= (1 << PB2) | (1 << PB3) | (1 << PB5);

  // SPI 설정
  SPCR |= (1 << SPE) | (1 << MSTR) | (1 << SPR0); // SPI 활성화, 마스터 모드, F_CPU/16 속도

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

  // 레이저 모듈 (테스트 필요함)
  DDRC |= (1 << PC0);
  /*
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
  */
}

/*
ISR(TIMER2_COMPA_vect) {
  static bool pinState = false;
  if (soundOn) {
    pinState = !pinState;
    if (pinState) {
      PORTD |= (1 << PD3);
    } 
    else {
      PORTD &= ~(1 << PD3);
    }
  } 
  else {
    PORTD &= ~(1 << PD3);
  }
}
*/
/*
// 핀 체인지 인터럽트 서비스 루틴
ISR(INT0_vect) {
  if (digitalRead(inputPin) == HIGH) {
    PORTD |= (1<<PD3);
    for (uint16_t j=0; j < 250; j++){
      for (uint16_t i=0; i<64000; i++){
        asm("nop");
      }
    }
    PORTD &= ~(1<<PD3);
  }
}
*/

void loop() {
  if (btserial.available()) { // Check if there is any data available from Bluetooth
    char cmd = (char)btserial.read(); // Read the data and convert it from char to int
    int status = cmd - '0';
    Serial.println(status); // Print received data to the Serial Monitor

    if (status == 1) { // 전진
      // maintainStatus2 = false; // 상태 2 유지 상태를 false로 설정
      sendDataToSlave(status);
      // 레이저 비활성화
      PORTC &= ~(1 << PC0);
      
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
      // maintainStatus2 = true; // 상태 2 유지 상태를 true로 설정
      sendDataToSlave(status);
      
      // lastToggleTime = millis(); // 초기화 필요
      // 피에조 부저 활성화
      // soundOn = true;
      
      // 레이저 모듈 활성화
      PORTC |= (1 << PC0);

      // 모터 A 제어 (in1Pin = LOW, in2Pin = HIGH, enaPin = 511)
      PORTB &= ~(1 << PB0); // in1Pin = LOW
      PORTD |= (1 << PD7);  // in2Pin = HIGH
      OCR1A = 511;          // enaPin = 255 (최대 속도)

      // 모터 B 제어 (in3Pin = HIGH, in4Pin = LOW, enbPin = 255)
      PORTD |= (1 << PD5);  // in3Pin = HIGH
      PORTD &= ~(1 << PD4); // in4Pin = LOW
      OCR0A = 255;          // enbPin = 255 (최대 속도)
    }
    else if(status == 0){ // 정지
      // maintainStatus2 = false; // 상태 2 유지 상태를 false로 설정
      sendDataToSlave(status);
      // 레이저 비활성화
      PORTC &= ~(1 << PC0);
      
      // 피에조 부저 비활성화
      PORTD &= ~(1 << PD3);
      // soundOn = false;
      
      // 모터 A 제어 (in1Pin = LOW, in2Pin = HIGH, enaPin = 0)
      PORTB &= ~(1 << PB0);  // in1Pin = LOW
      PORTD &= ~(1 << PD7);  // in2Pin = HIGH
      OCR1A = 0;          

      // 모터 B 제어 (in3Pin = HIGH, in4Pin = LOW, enbPin = 0)
      PORTD &= ~(1 << PD5);  // in3Pin = HIGH
      PORTD &= ~(1 << PD4);  // in4Pin = LOW
      OCR0A = 0;          
    }
  }

  /*
  // 상태 2를 유지하면서 피에조 부저의 소리를 주기적으로 켜고 끄기
  if (maintainStatus2) {
    unsigned long currentMillis = millis();

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
  */
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
