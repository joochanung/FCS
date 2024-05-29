// SPI + 서보 모터 테스트 코드
// 서보 모터가 회전한 각도를 Master 아두이노가 값을 받아와 Serial 모니터에 출력 & Slave에 값 전달

#include <SPI.h>
#include <Servo.h>

Servo servo;

// 서보 모터 핀 
const int slaveSelectPin = 10;

uint8_t transfer_SPI(uint8_t data){
  // Activate slave
  PORTB &= ~(1 << SS);

  SPDR = data;
  while (!(SPSR & (1 << SPIF)));

  // Deactivate slave
  PORTB |= (1 << SS);

  return SPDR;
}

void setup() {
  // pinMode(slaveSelectPin, OUTPUT);
  DDRB |= (1 << DDB2);
  // digitalWrite(slaveSelectPin, HIGH);
  PORTB |= (1 << SS); // SS를 비활성화 상태로 시작
  // SPI.begin();
  SPCR |= (1 << SPE) | (1 << MSTR) | (0 << SPR1) | (1 << SPR0);
  Serial.begin(9600);
  // servo.attach(9);
  DDRB |= (1 << DDB1);
  // servo.write(0);
}

void loop() {
  // 90도라는 값 전달
  int motorAngle = getMotorAngle(90); 
  // 슬레이브에게 각도 데이터를 전송
  sendAngleToSlave(motorAngle);
  delay(1000); // 1초에 한 번씩 전송

  // 0도라는 값 전달
  motorAngle = getMotorAngle(0);
  sendAngleToSlave(motorAngle);
  delay(1000);
}

int getMotorAngle(int angle) {
  // 모터 각도를 측정하는 코드를 여기에 작성합니다.
  // servo.write(angle);
  // return angle;
  int pulseWidth = map(angle, 0, 180, 1000, 2000);
  // digitalWrite(servoPin, HIGH);
  PORTB |= (1 << PB1);
  delayMicroseconds(pulseWidth);
  // digitalWrite(servoPin, LOW);
  PORTB &= ~(1 << PB1);
  delayMicroseconds(20000 - pulseWidth);

  return angle;
}

void sendAngleToSlave(int angle) {
  byte highByte = (angle >> 8) & 0xFF; // 상위 8비트
  byte lowByte = angle & 0xFF;         // 하위 8비트

  // digitalWrite(slaveSelectPin, LOW);
  PORTB &= ~(1 << PB2);   // 슬레이브 선택
  // SPI.transfer(highByte);              // 상위 바이트 전송
  transfer_SPI(highByte);
  transfer_SPI(lowByte);
  // SPI.transfer(lowByte);               // 하위 바이트 전송
  // digitalWrite(slaveSelectPin, HIGH);
  PORTB |= (1 << PB2);  // 슬레이브 비선택

  Serial.print("Sent angle: ");
  Serial.println(angle);
}
