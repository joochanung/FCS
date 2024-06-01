// 피에조 부저는 수정이 필요함. (잠깐 소리가 나다가 몇 초간 안 나야 하는데, 지금은 계속해서 소리가 남.)
// ISR 부분을 수정하거나 loop문 내에서 코드를 작성해 작동하다가 안 하게 만들어야 할 듯.

#define OC2B 0x08

ISR(INT0_vect) {
  float freq_target = 392.00;

  OCR2A = F_CPU / 256 / freq_target - 1;
  OCR2B = OCR2A / 2;

  // Enable Timer/Counter2 Overflow Interrupt
  TIMSK2 |= (1 << TOIE2);
  
  for (uint16_t j = 0; j < 50; j++) {
    for (uint16_t i = 0; i < 1000; i++) {
      asm("nop");
    }
  }
  delayMicroseconds(3000);
}

void setup() {
  cli();

  EICRA |= 0x03;
  EIMSK |= 0x01;

  DDRD |= OC2B;

  TCCR2A |= (1 << WGM21) | (1 << WGM20);
  TCCR2B |= (1 << WGM22);
  TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);

  TCCR2A |= (1 << COM2B1);

  float freq_target = 392.00;

  OCR2A = F_CPU / 256 / freq_target - 1;
  OCR2B = OCR2A / 2;

  sei();
  delay(3000);
}

void loop() {
  // main code here, to run repeatedly

}
