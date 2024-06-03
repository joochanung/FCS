목표 : Pixy2를 라이브러리 없이 사용해보자

현재 알고 있는 정보 : 
1. 우리가 사용한 라이브러리 에서는 Pixy2를 아두이노 우노의 ICSP 핀을 이용하여 SPI 통신을 통해 데이터를 주고 받는다
2. ICSP 커넥터 핀 이용하여 연결했을때 SS pin을 따로 지정해 줄 필요가 없다

알아야 할 정보 :
1. pixy.init()

template <class LinkType> int8_t TPixy2<LinkType>::init(uint32_t arg)
{
  uint32_t t0;
  int8_t res;
  
  res = m_link.open(arg);
  if (res<0)
    return res;
  
  // wait for pixy to be ready -- that is, Pixy takes a second or 2 boot up
  // getVersion is an effective "ping".  We timeout after 5s.
  for(t0=millis(); millis()-t0<5000; )
  {
    if (getVersion()>=0) // successful version get -> pixy is ready
	{
      getResolution(); // get resolution so we have it
      return PIXY_RESULT_OK;
    }	  
    delayMicroseconds(5000); // delay for sync
  }
  // timeout
  return PIXY_RESULT_TIMEOUT;
}

2. pixy.ccc.getBlocks()
3. pixy.ccc.numBlocks
4. pixy.ccc.blocks[i].m_x
5. pixy.ccc.blocks[i].m_y
6. pixy.frameWidth
