목표 : Pixy2를 라이브러리 없이 사용해보자

현재 알고 있는 정보 : 
1. 우리가 사용한 라이브러리 에서는 Pixy2를 아두이노 우노의 ICSP 핀을 이용하여 SPI 통신을 통해 데이터를 주고 받는다
2. ICSP 커넥터 핀 이용하여 연결했을때 SS pin을 따로 지정해 줄 필요가 없다

알아야 할 정보 :
1. pixy.init()
2. pixy.ccc.getBlocks()
3. pixy.ccc.numBlocks
4. pixy.ccc.blocks[i].m_x
5. pixy.ccc.blocks[i].m_y
6. pixy.frameWidth
