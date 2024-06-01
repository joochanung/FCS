// 코드를 하나로 합치는 과정에서 반드시 필요한 코드 (아두이노 - 스마트폰 간의 통신 과정)
// HC-05가 잘 작동하는지 확인하기 위해 사용한 코드. 그 뿐만 아니라 스마트폰와의 연결, 스마트폰에서의 입력과 아두이노에서의 출력 결과가 잘 이루어져 있는지 확인하는 과정.

#include<SoftwareSerial.h> 

SoftwareSerial btserial(3,2);
//(아두이노의 RX는 블루투스의 TX에, 아두이노의 TX는 블루투스의 RX에 연결

// 핀 정의
const int enaPin = 6; // 모터 A 속도 제어 핀 (PWM)
const int in1Pin = 5; // 모터 A 방향 제어 핀 1
const int in2Pin = 4;  // 모터 A 방향 제어 핀 2

const int in3Pin = 7;  // 모터 B 방향 제어 핀 1
const int in4Pin = 8;  // 모터 B 방향 제어 핀 2
const int enbPin = 9;  // 모터 B 속도 제어 핀 (PWM)

void setup() {
  btserial.begin(9600); //아두이노와 블루투스끼리의 통신속도를 9600으로 지정
  Serial.begin(9600); //아두이노와 컴퓨터와의 통신속도도 9600으로 지정

  // // 핀 모드 설정
  // pinMode(enaPin, OUTPUT);
  // pinMode(in1Pin, OUTPUT);
  // pinMode(in2Pin, OUTPUT);
  
  // pinMode(enbPin, OUTPUT);
  // pinMode(in3Pin, OUTPUT);
  // pinMode(in4Pin, OUTPUT);

  DDRD |= (1 << PD6) | (1 << PD5) | (1 << PD4); // enaPin, in1Pin, in2Pin
  DDRB |= (1 << PB1) | (1 << PB0) | (1 << PB3); // enbPin, in3Pin, in4Pin
  
  // // 초기 상태 설정 (모터 정지)
  // analogWrite(enaPin, 0);
  // digitalWrite(in1Pin, LOW);
  // digitalWrite(in2Pin, LOW);
  
  // analogWrite(enbPin, 0);
  // digitalWrite(in3Pin, LOW);
  // digitalWrite(in4Pin, LOW);
  
  PORTD &= ~(1 << PD6); // enaPin LOW
  PORTD &= ~(1 << PD5); // in1Pin LOW
  PORTD &= ~(1 << PD4); // in2Pin LOW
  
  PORTB &= ~(1 << PB1); // enbPin LOW
  PORTB &= ~(1 << PB0); // in3Pin LOW
  PORTB &= ~(1 << PB3); // in4Pin LOW

  // Timer 0 for PWM (enaPin, enbPin)
  TCCR0A |= (1 << WGM00) | (1 << WGM01); // Fast PWM
  TCCR0A |= (1 << COM0A1); // Clear OC0A on compare match (PB7/PD6)
  TCCR0A |= (1 << COM0B1); // Clear OC0B on compare match (PD5)
  TCCR0B |= (1 << CS00); // No prescaling

  // Timer 2 for PWM (enaPin, enbPin)
  TCCR2A |= (1 << WGM20) | (1 << WGM21); // Fast PWM
  TCCR2A |= (1 << COM2A1); // Clear OC2A on compare match (PD3)
  TCCR2A |= (1 << COM2B1); // Clear OC2B on compare match (PD3)
  TCCR2B |= (1 << CS20); // No prescaling
}

void loop() {
  //만약 블루투스가 통신가능한 상태라면 아래 코드들을 실행, 아니라면 아무것도 하지 않음
  if(btserial.available()){     
    //문자 형식의 cmd라는 변수를 생성하고 블루투스로부터 들어오는 값을 저장
    char cmd = (char)btserial.read(); 
    //블루투스로부터 들어오는 값을 시리얼모니터에 출력
    Serial.println(cmd); 
    if(cmd == 'a'){ // 가동
      // digitalWrite(in1Pin, HIGH);
      // digitalWrite(in2Pin, LOW);
      // analogWrite(enaPin, 255); // 모터 A 최대 속도
      PORTD |= (1 << PD5); // in1Pin HIGH
      PORTD &= ~(1 << PD4); // in2Pin LOW
      OCR0A = 255;
  
      // digitalWrite(in3Pin, HIGH);
      // digitalWrite(in4Pin, LOW);
      // analogWrite(enbPin, 255); // 모터 B 최대 속도
      PORTB |= (1 << PB0); // in3Pin HIGH
      PORTB &= ~(1 << PB3); // in4Pin LOW
      OCR0B = 255;
    }    
    else if(cmd == 'b'){ // 정지
      // analogWrite(enaPin, 0);
      // digitalWrite(in1Pin, LOW);
      // digitalWrite(in2Pin, LOW);
      OCR0A = 0;
      PORTD &= ~(1 << PD5); // in1Pin LOW
      PORTD &= ~(1 << PD4); // in2Pin LOW
  
      // analogWrite(enbPin, 0);
      // digitalWrite(in3Pin, LOW);
      // digitalWrite(in4Pin, LOW);
      OCR0B = 0;
      PORTB &= ~(1 << PB0); // in3Pin LOW
      PORTB &= ~(1 << PB3); // in4Pin LOW
    }
  }
}
