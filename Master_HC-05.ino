#include<SoftwareSerial.h> 
#include <SPI.h>

SoftwareSerial btserial(3,2);
//(아두이노의 RX는 블루투스의 TX에, 아두이노의 TX는 블루투스의 RX에 연결

const int slaveSelectPin = 10;

void setup() {
  pinMode(slaveSelectPin, OUTPUT);
  digitalWrite(slaveSelectPin, HIGH); // SS를 비활성화 상태로 시작
  SPI.begin();
  btserial.begin(9600); //아두이노와 블루투스끼리의 통신속도를 9600으로 지정
  Serial.begin(9600); //아두이노와 컴퓨터와의 통신속도도 9600으로 지정
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

  digitalWrite(slaveSelectPin, LOW);   // 슬레이브 선택
  SPI.transfer(highByte);              // 상위 바이트 전송
  SPI.transfer(lowByte);               // 하위 바이트 전송
  digitalWrite(slaveSelectPin, HIGH);  // 슬레이브 비선택

  Serial.print("Sent data: ");
  Serial.println(data);
}
