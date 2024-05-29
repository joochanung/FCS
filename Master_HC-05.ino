// SPI + 서보 모터 테스트 코드
// 서보 모터가 회전한 각도를 Master 아두이노가 값을 받아와 Serial 모니터에 출력 & Slave에 값 전달

#include<SoftwareSerial.h> 
#include <SPI.h>

SoftwareSerial btserial(3,2);
//(아두이노의 RX는 블루투스의 TX에, 아두이노의 TX는 블루투스의 RX에 연결

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

void init_Serial(){
  // pinMode(slaveSelectPin, OUTPUT);
  DDRB |= (1 << DDB2);
  // servo.attach(9);
  DDRB |= (1 << DDB1);
  // digitalWrite(slaveSelectPin, HIGH);
  PORTB |= (1 << SS); // SS를 비활성화 상태로 시작
  // SPI.begin();
  SPCR |= (1 << SPE) | (1 << MSTR) | (0 << SPR1) | (1 << SPR0);
  Serial.begin(9600);
  btserial.begin(9600);
  // servo.write(0);
}
void setup() {
  init_Serial();
}

void loop() {
  //만약 블루투스가 통신가능한 상태라면 아래 코드들을 실행, 아니라면 아무것도 하지 않음
  if(btserial.available()){     
    //문자 형식의 cmd라는 변수를 생성하고 블루투스로부터 들어오는 값을 저장
    char cmd = (char)btserial.read();
    int status = cmd - '0';
    //블루투스로부터 들어오는 값을 시리얼모니터에 출력
    Serial.println(cmd); 
    if(status == 1){ 
      sendDataToSlave(status);
    }    
    else if(status == 2){ 
      sendDataToSlave(status);
    }
    else if(status == 3){
      sendDataToSlave(status);
    }
  }
}

void sendDataToSlave(int data){
  byte highByte = (data >> 8) & 0xFF; // 상위 8비트
  byte lowByte = data & 0xFF;         // 하위 8비트

  // digitalWrite(slaveSelectPin, LOW);
  PORTB &= ~(1 << PB2);   // 슬레이브 선택
  // SPI.transfer(highByte);              // 상위 바이트 전송
  transfer_SPI(highByte);
  transfer_SPI(lowByte);
  // SPI.transfer(lowByte);               // 하위 바이트 전송
  // digitalWrite(slaveSelectPin, HIGH);
  PORTB |= (1 << PB2);  // 슬레이브 비선택

  Serial.print("Sent data: ");
  Serial.println(data);
}
