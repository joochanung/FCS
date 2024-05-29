// 모터 A: 탱크 하부 2개의 DC 모터
// 모터 B: 탱크 하부 너모지 2개의 DC 모터

// 핀 정의
const int enaPin = 3; // 모터 A 속도 제어 핀 (PWM)
const int in1Pin = 4; // 모터 A 방향 제어 핀 1
const int in2Pin = 5;  // 모터 A 방향 제어 핀 2

const int in3Pin = 7;  // 모터 B 방향 제어 핀 1
const int in4Pin = 8;  // 모터 B 방향 제어 핀 2
const int enbPin = 9;  // 모터 B 속도 제어 핀 (PWM)

void setup() {
  // 핀 모드 설정
  pinMode(enaPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);

  pinMode(enbPin, OUTPUT);
  pinMode(in3Pin, OUTPUT);
  pinMode(in4Pin, OUTPUT);
  
  // 초기 상태 설정 (모터 정지)
  analogWrite(enaPin, 0);
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, LOW);

  analogWrite(enbPin, 0);
  digitalWrite(in3Pin, LOW);
  digitalWrite(in4Pin, LOW);
}

void loop() {
  digitalWrite(in1Pin, LOW);
  digitalWrite(in2Pin, HIGH);
  analogWrite(enaPin, 255); //모터 A 속도 조절
  digitalWrite(in3Pin, HIGH);
  digitalWrite(in4Pin, LOW);
  analogWrite(enbPin, 255); //모터 B 속도 조절
}

// PS: 레지스터 비트로 변환 필요
