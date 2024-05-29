// 코드를 하나로 합치는 과정에서 반드시 필요한 코드 (아두이노 - 스마트폰 간의 통신 과정)
// HC-05가 잘 작동하는지 확인하기 위해 사용한 코드. 그 뿐만 아니라 스마트폰와의 연결, 스마트폰에서의 입력과 아두이노에서의 출력 결과가 잘 이루어져 있는지 확인하는 과정.

#include<SoftwareSerial.h> 

SoftwareSerial btserial(3,2);
// 아두이노의 RX는 블루투스의 TX에, 아두이노의 TX는 블루투스의 RX에 연결
// 아두이노 3번 - 블루투스의 TX, 아두이노 2번 - 블루투스 RX

#define led 7 
#define led2 6

void setup() {
  btserial.begin(9600); //아두이노와 블루투스끼리의 통신속도를 9600으로 지정
  Serial.begin(9600);   //아두이노와 컴퓨터와의 통신속도도 9600으로 지정
  pinMode(led,OUTPUT); 
  pinMode(led2, OUTPUT);
}

void loop() {
  //만약 블루투스가 통신가능한 상태라면 아래 코드들을 실행, 아니라면 아무것도 하지 않음
  if(btserial.available()){     
    //문자 형식의 cmd라는 변수를 생성하고 블루투스로부터 들어오는 값을 저장
    char cmd = (char)btserial.read(); 
    //블루투스로부터 들어오는 값을 시리얼모니터에 출력
    Serial.println(cmd); 
    if(cmd == 'a'){
      // a를 입력하면 led ON
      digitalWrite(led,HIGH);
    }    
    else if(cmd == 'b'){ 
      // b를 입력하면 led2 ON
      digitalWrite(led2,HIGH); 
    }
    else if(cmd == 'c'){
      // c를 입력하면 모든 led OFF
      digitalWrite(led, LOW);
      digitalWrite(led2, LOW);
    }
  }
}
