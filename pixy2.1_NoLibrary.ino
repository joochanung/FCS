#include <Pixy2.h>
#include <Servo.h>

// This is the main Pixy object 
Pixy2 pixy;

// Create servo objects
Servo servoX;
Servo servoY;

// Define pin numbers for servos
const int servoXPin = 5;
const int servoYPin = 6;

// Servo positions
int servoXPos = 180;
int servoYPos = 180;

// Screen center coordinates
const int screenWidth = 316;
const int screenHeight = 208;
const int centerX = screenWidth / 2;
const int centerY = screenHeight / 2;

// Servo movement increment
const int stepSize = 1;

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");
  
  pixy.init();

  // Attach the servos
  servoX.attach(servoXPin);
  servoY.attach(servoYPin);

  // Initialize servo positions
  servoX.write(servoXPos);
  servoY.write(servoYPos);
}

void loop()
{ 
  int i; 
  // Grab blocks!
  pixy.ccc.getBlocks();
  
  // If there are detected blocks, process them!
  if (pixy.ccc.numBlocks)
  {
    Serial.print("Detected ");
    Serial.println(pixy.ccc.numBlocks);
    for (i=0; i<pixy.ccc.numBlocks; i++)
    { 
      /*
      Serial.print("  block ");
      Serial.print(i);
      Serial.print(": ");
      pixy.ccc.blocks[i].print();
      */

      int blockX = pixy.ccc.blocks[i].m_x;
      int blockY = pixy.ccc.blocks[i].m_y;

      // Calculate the error between block position and screen center
      int errorX = blockX - centerX;
      int errorY = blockY - centerY;

      // Adjust servo positions to reduce the error
      if (abs(errorX) > 10) // Tolerance to avoid shaking
      {
        if (errorX > 0)
          servoXPos -= stepSize;
        else
          servoXPos += stepSize;
      }

      if (abs(errorY) > 10) // Tolerance to avoid shaking
      {
        if (errorY > 0)
          servoYPos += stepSize;
        else
          servoYPos -= stepSize;
      }

      // Ensure servo positions are within valid range
      servoXPos = constrain(servoXPos, 0, 180);
      servoYPos = constrain(servoYPos, 0, 180);

      // Write new positions to servos
      servoX.write(servoXPos);
      servoY.write(servoYPos);

      Serial.print("Pan Angle: ");
      Serial.println(servoXPos);
      Serial.print("Tilt Angle: ");
      Serial.println(servoYPos);

      // Short delay to allow servo to move
      delay(20);
    }
  }  
}
