// 피에조 부너 : PD3, 레이저 모듈: PC0
#include <SoftwareSerial.h>

#define BTRXD A2
#define BTTXD A3

// Arduino RX connected to Bluetooth TX, Arduino TX connected to Bluetooth RX
SoftwareSerial btserial(BTTXD, BTRXD); 

int status;

const int buzzerPin = 3;
const int inputPin = A0;

void setup() {
  // SPI 설정
  // SS, MOSI, SCK 핀을 출력 모드로 설정
  DDRB |= (1 << PB2) | (1 << PB3) | (1 << PB5);
  PORTB |= (1 << PB2);

  btserial.begin(9600);
  Serial.begin(9600);

  pinMode(buzzerPin, OUTPUT);
  pinMode(inputPin, INPUT);

  DC_Motor_init();

  // 레이저 모듈
  DDRC |= (1 << PC0);

}

void loop() {
  int sensorValue = analogRead(inputPin);

  if (btserial.available()) { // Check if there is any data available from Bluetooth
    char cmd = (char)btserial.read(); // Read the data and convert it from char to int
    status = cmd - '0';

    Serial.println(status); // Print received data to the Serial Monitor

    // SPI 활성화
    SPCR |= (1 << SPE) | (1 << MSTR) | (1 << SPR0);

    if (status == 1) { // 전진
      sendDataToSlave(status);

      // 레이저 비활성화
      PORTC &= ~(1 << PC0);

      digitalWrite(buzzerPin, LOW);

      run();
    }

    else if(status == 2){ // 기동간 사격
      sendDataToSlave(status);

      // 레이저 활성화
      PORTC |= (1 << PC0);

      if (sensorValue > 512) { // 일정 전압 이상이 들어오면 피에조 부저 울림
        digitalWrite(buzzerPin, HIGH); 
      } 
      else {
        digitalWrite(buzzerPin, LOW); 
      }

      run();
    }
    
    else if(status == 0){ // 정지
      sendDataToSlave(status);

      // 레이저 비활성화
      PORTC &= ~(1 << PC0);

      digitalWrite(buzzerPin, LOW);

      stop();       
    }

    // SPI 비활성화
    SPCR &= ~(1 << SPE);
  }
}

void DC_Motor_init(){
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
}

void run(){
  // 모터 A 제어 (in1Pin = LOW, in2Pin = HIGH, enaPin = 511)
  PORTB &= ~(1 << PB0); // in1Pin = LOW
  PORTD |= (1 << PD7);  // in2Pin = HIGH
  OCR1A = 511;          // enaPin = 255 (최대 속도)

  // 모터 B 제어 (in3Pin = HIGH, in4Pin = LOW, enbPin = 255)
  PORTD |= (1 << PD5);  // in3Pin = HIGH
  PORTD &= ~(1 << PD4); // in4Pin = LOW
  OCR0A = 255;          // enbPin = 255 (최대 속도)
}

void stop(){
  // 모터 A 제어 (in1Pin = LOW, in2Pin = HIGH, enaPin = 0)
  PORTB &= ~(1 << PB0);  // in1Pin = LOW
  PORTD &= ~(1 << PD7);  // in2Pin = HIGH
  OCR1A = 0;          

  // 모터 B 제어 (in3Pin = HIGH, in4Pin = LOW, enbPin = 0)
  PORTD &= ~(1 << PD5);  // in3Pin = HIGH
  PORTD &= ~(1 << PD4);  // in4Pin = LOW
  OCR0A = 0;   
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