// 모터 A: 탱크 하부 2개의 DC 모터
// 모터 B: 탱크 하부 너모지 2개의 DC 모터

// 핀 정의
const int enaPin = 3; // 모터 A 속도 제어 핀 (PWM)
const int in1Pin = 4; // 모터 A 방향 제어 핀 1
const int in2Pin = 5;  // 모터 A 방향 제어 핀 2

const int in3Pin = 7;  // 모터 B 방향 제어 핀 1
const int in4Pin = 8;  // 모터 B 방향 제어 핀 2
const int enbPin = 9;  // 모터 B 속도 제어 핀 (PWM)

uint8_t pwm = 0;

ISR(TIMER2_COMPA_vect){
  OCR2A = pwm;
}

void setup() {

  cli();

  // 핀 모드 설정
  // pinMode(enaPin, OUTPUT);
  // pinMode(in1Pin, OUTPUT);
  // pinMode(in2Pin, OUTPUT);
  // DDRB |= 0xB8;
  DDRB |= (1 << PB0) | (1 << PB1) | (1 << PB2) | (1 << PB7);

  // pinMode(enbPin, OUTPUT);
  // pinMode(in3Pin, OUTPUT);
  // pinMode(in4Pin, OUTPUT);
  // DDRD |= 0x03;
  DDRD |= (1 << PD3) | (1 << PD4) | (1 << PD5) | (1 << PD6);
  
  // 초기 상태 설정 (모터 정지)
  // analogWrite(enaPin, 0);
  // digitalWrite(in1Pin, LOW);
  // digitalWrite(in2Pin, LOW);
  // PORTB &= ~0x00;
  PORTB &= ~(1 << PB0) & ~(1 << PB7);

  // analogWrite(enbPin, 0);
  // digitalWrite(in3Pin, LOW);
  // digitalWrite(in4Pin, LOW);
  // PORTD &= ~0x00;
  PORTD &= ~(1 << PD4) & ~(1 << PD5);

  // Fast PWM
  TCCR2A |= (1 << WGM21) | (1 << WGM20);

  // No prescaling
  TCCR2B |= (0 << CS22) | (0 << CS21) | (1 << CS20);

  // Setting for OC2A: clear OC2A on compare match
  TCCR2A |= (1 << COM2A1) | (0 << COM2A0);

  // Enable interrupt
  TIMSK2 |= (1 << OCIE2A);

  // Output 2A
  OCR2A = pwm;

  sei();

  Serial.begin(9600);
}

void loop() {
  // digitalWrite(in1Pin, LOW);
  // digitalWrite(in2Pin, HIGH);
  // analogWrite(enaPin, 255); //모터 A 속도 조절
  // digitalWrite(in3Pin, HIGH);
  // digitalWrite(in4Pin, LOW);
  // analogWrite(enbPin, 255); //모터 B 속도 조절
  if (Serial.available()){
    pwm = 200;
    // PORTB |= 0xA0; // Pin 5, 7 HIGH
    // 모터 A 제어
    PORTD &= ~(1 << PD4); // in1Pin LOW
    PORTD |= (1 << PD5); // in2Pin HIGH
    OCR2A = pwm;

    // 모터 B 제어
    PORTB |= (1 << PB0); // in3Pin LOW
    PORTB &= ~(1 << PB7); // in4Pin HIGH
    OCR2B = pwm;
  }
}
