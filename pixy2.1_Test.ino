// 라이브러리를 이용한 pixy2.1 물체 추적 코드

#include <Pixy2.h>
#include <PIDLoop.h>
#include <Servo.h>

Pixy2 pixy;
PIDLoop panLoop(300, 0, 400, true);  // 조정된 PID 파라미터
PIDLoop tiltLoop(300, 0, 400, true); // 조정된 PID 파라미터

Servo panServo;
Servo tiltServo;

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");

  // We need to initialize the pixy object 
  pixy.init();
  // Use color connected components program for the pan tilt to track 
  pixy.changeProg("color_connected_components");

  // Attach servos to pins
  panServo.attach(5);
  tiltServo.attach(6);
  panServo.write(0);  // Center position (중앙으로 초기화)
  tiltServo.write(180); // Center position (중앙으로 초기화)
}

void loop()
{  
  int32_t panOffset, tiltOffset;

  // get active blocks from Pixy
  pixy.ccc.getBlocks();

  if (pixy.ccc.numBlocks)
  {        
    // calculate pan and tilt "errors" with respect to first object (blocks[0]), 
    // which is the biggest object (they are sorted by size).  
    panOffset = (int32_t)pixy.frameWidth / 2 - (int32_t)pixy.ccc.blocks[0].m_x;
    tiltOffset = (int32_t)pixy.ccc.blocks[0].m_y - (int32_t)pixy.frameHeight / 2;  

    // update loops
    panLoop.update(panOffset);
    tiltLoop.update(tiltOffset);

    // get pan and tilt command from PID loops
    int panCommand = panLoop.m_command;
    int tiltCommand = tiltLoop.m_command;

    // limit the command values to the expected range
    panCommand = constrain(panCommand, -400, 400);
    tiltCommand = constrain(tiltCommand, -400, 400);

    // map the command values to servo angle range
    int panAngle = map(panCommand, -400, 400, -180, 180);
    int tiltAngle = map(tiltCommand, -400, 400, 0, 180);

    // set pan and tilt servos  
    panServo.write(constrain(panAngle, -180, 180)); // 각도를 제한하여 서보의 물리적 한계 내에서 움직이게 함
    tiltServo.write(constrain(tiltAngle, 0, 180)); // 각도를 제한하여 서보의 물리적 한계 내에서 움직이게 함
  }
  else{
  }
}
