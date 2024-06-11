// Master 기본 뼈대 코드 (SPI, 블루투스 설정)

#include <SoftwareSerial.h> 
#include <SPI.h>

#define BTRXD 2;
#define BTTXD 3;
#define IN2 4;
#define IN1 5;
#define ENA 6;
#define IN3 7;
#define IN4 8;
#define ENB 9;
#define SS 10;
#define MOSI 11;
#define MISO 12;
#define SCK 13;

#define FOSC 1843200
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD - 1

//(아두이노의 RX는 블루투스의 TX에, 아두이노의 TX는 블루투스의 RX에 연결
SoftwareSerial btserial(BTTXD, BTRXD);

int status = 0;

uint8_t transfer_SPI(uint8_t data){
  // Activate slave
  PORTB &= ~(1 << PB2);

  SPDR = data;
  while (!(SPSR & (1 << SPIF)));

  // Deactivate slave
  PORTB |= (1 << PB2);

  return SPDR;
}

void USART_init(unsigned int ubrr){
  ubrr = FOSC/16/BAUD - 1;

  // baud rate 설정
  UBRR0H = (unsigned char)(ubrr >> 8);
  UBRR0L = (unsigned char)ubrr;

  // RX와 TX 활성화
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);

  //프레임 format 설정: 8 bit data, 2 stop bit
  UCSR0C = (1 << USBS0) | (3 << UCSZ00);
}

void USART_transmit(unsigned char data){
  while(!(UCSR0A & (1 << UDRE0)));
  UDR0 = data;
}

unsigned char USART_receive(void){
  //문자 형식의 cmd라는 변수를 생성하고 블루투스로부터 들어오는 값을 저장
  while (!(UCSR0A & (1 << RXC0))); 
  char cmd = UDR0;
  //블루투스로부터 들어오는 값을 시리얼모니터에 출력
  Serial.println(cmd); 
  return cmd - '0';
}

void Serial_init(){
  // pinMode(slaveSelectPin, OUTPUT);
  DDRB |= (1 << DDB2);
  // digitalWrite(slaveSelectPin, HIGH);
  PORTB |= (1 << PB2); // SS를 비활성화 상태로 시작
  // SPI.begin();
  SPCR |= (1 << SPE) | (1 << MSTR) | (0 << SPR1) | (1 << SPR0);
  // servo.write(0);
}

void setup() {
  btserial.begin(9600);
  // Serial.begin(9600);
  Serial_init();
  USART_init(MYUBRR);
}

void loop() {
  //만약 블루투스가 통신가능한 상태라면 아래 코드들을 실행, 아니라면 아무것도 하지 않음
  if(UCSR0A & (1 << RXC0)){  // btserial   
    status = USART_receive();
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
