// 피에조 부저가 연결된 핀 번호
const int piezoPin = 3;

// 소리 출력 및 멈춤 시간을 밀리초 단위로 설정
const int soundDuration = 100;  // 소리가 나는 시간 (500ms)
const int silenceDuration = 2000;  // 소리가 안 나는 시간 (1000ms)

// 주파수 설정 (392Hz)
const int frequency = 392;

// 타이머 인터럽트에 사용될 변수
volatile bool soundOn = false;  // 현재 소리 상태를 저장
volatile unsigned long lastToggleTime = 0;

void setup() {
  // 피에조 부저 핀을 출력 모드로 설정
  DDRD |= (1 << DDD3);

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
  static bool pinState = false;
  if (soundOn) {
    PIND = (1 << piezoPin);
  } 
  else {
    // digitalWrite(piezoPin, LOW); // 소리 끄기
    PORTD &= ~(1 << piezoPin);
  }
}

void loop() {
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
