//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//

#include <SPI.h>  
#include <Pixy.h>

#define X_CENTER    160L
#define Y_CENTER    100L
#define RCS_MIN_POS     0L
#define RCS_MAX_POS     1000L
#define RCS_CENTER_POS  ((RCS_MAX_POS-RCS_MIN_POS)/2)

class ServoLoop{
public:
  ServoLoop(int32_t pgain, int32_t dgain);

  void update(int32_t error);
   
  int32_t m_pos;
  int32_t m_prevError;
  int32_t m_pgain;
  int32_t m_dgain;
};


ServoLoop panLoop(200, 200);  // Servo loop for pan
ServoLoop tiltLoop(150, 200); // Servo loop for tilt


ServoLoop::ServoLoop(int32_t pgain, int32_t dgain){
  m_pos = RCS_CENTER_POS;
  m_pgain = pgain;
  m_dgain = dgain;
  m_prevError = 0x80000000L;
}

void ServoLoop::update(int32_t error){
  long int vel;
  char buf[32];
  if (m_prevError!=0x80000000)
  { 
    vel = (error*m_pgain + (error - m_prevError)*m_dgain)>>10;
    //sprintf(buf, "%ld\n", vel);
   // Serial.print(buf);
    m_pos += vel;
    if (m_pos>RCS_MAX_POS) 
      m_pos = RCS_MAX_POS; 
    else if (m_pos<RCS_MIN_POS) 
      m_pos = RCS_MIN_POS;
    //cprintf("%d %d %d\n", m_axis, m_pos, vel);
  }
  m_prevError = error;
}

class DCMOTORS{
  private:
    int leftSpeed;
    int rigthSpeed;
    int leftDir;
    int rightDir;
  public:
    DCMOTORS(){
      /*leftSpeed = 9;
      rigthSpeed = 10;
      leftDir = 7;
      rightDir = 8;
      */
      leftSpeed = 5;
      rigthSpeed = 6;
      leftDir = 10;
      rightDir = 11;       
      pinMode(leftDir, OUTPUT);
      pinMode(rightDir, OUTPUT);
      pinMode(leftSpeed, OUTPUT);
      pinMode(rigthSpeed, OUTPUT);
    }//end DCMOTORS
    int setLeftSpeed(int speed){
      if(speed > 0){//forward
          digitalWrite(leftDir, HIGH);
          analogWrite(leftSpeed, speed);
        }else{//backward
          digitalWrite(leftDir, LOW);
          analogWrite(leftSpeed, speed);
        }
    }//end set
    int setRightSpeed(int speed){
       if(speed > 0){
          digitalWrite(rightDir, HIGH);
          analogWrite(rigthSpeed, speed);
        }else{
          digitalWrite(rightDir, LOW);
          analogWrite(rigthSpeed, speed);
        }
    }//end set
/*
    void leftRotate(int speed){
      digitalWrite(leftDir, LOW);
      digitalWrite(rightDir, HIGH);
      analogWrite(leftSpeed, speed);
      analogWrite(rigthSpeed, speed);
    }
    void rightRotate(int speed){
      digitalWrite(leftDir, HIGH);
      digitalWrite(rightDir, LOW);
      analogWrite(leftSpeed, speed);
      analogWrite(rigthSpeed, speed);
    }    

    void forward(int speed){
      digitalWrite(leftDir, HIGH);
      digitalWrite(rightDir, HIGH);
      analogWrite(leftSpeed, speed);
      analogWrite(rigthSpeed, speed);
    }
    void reverse(int speed){
      digitalWrite(leftDir, LOW);
      digitalWrite(rightDir, LOW);
      analogWrite(leftSpeed, speed);
      analogWrite(rigthSpeed, speed);
    }
*/

};


Pixy pixy;

void setup()
{
  //Serial.begin(9600);
  //Serial.print("Starting...\n");
  
  pixy.init();
}

uint32_t lastBlockTime = 0;

DCMOTORS motors;

//---------------------------------------
// Main loop - runs continuously after setup
//---------------------------------------
void loop()
{ 
  uint16_t blocks;
  blocks = pixy.getBlocks();
 
  // If we have blocks in sight, track and follow them
  if (blocks)
  {
    int trackedBlock = TrackBlock(blocks);
    FollowBlock(trackedBlock);
    lastBlockTime = millis();
  }  
  else if (millis() - lastBlockTime > 100)
  {
    motors.setLeftSpeed(0);
    motors.setRightSpeed(0);
    ScanForBlocks();
  }
}
 
int oldX, oldY, oldSignature;
 
//---------------------------------------
// Track blocks via the Pixy pan/tilt mech
// (based in part on Pixy CMUcam5 pantilt example)
//---------------------------------------
int TrackBlock(int blockCount)
{
  int trackedBlock = 0;
  long maxSize = 0;
 
 // Serial.print("blocks =");
  //Serial.println(blockCount);
 
  for (int i = 0; i < blockCount; i++)
  {
    if ((oldSignature == 0) || (pixy.blocks[i].signature == oldSignature))
    {
      long newSize = pixy.blocks[i].height * pixy.blocks[i].width;
      if (newSize > maxSize)
      {
        trackedBlock = i;
        maxSize = newSize;
      }
    }
  }
 
  int32_t panError = X_CENTER - pixy.blocks[trackedBlock].x;
  int32_t tiltError = pixy.blocks[trackedBlock].y - Y_CENTER;
 
  panLoop.update(panError);
  tiltLoop.update(tiltError);
 
  pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
 
  oldX = pixy.blocks[trackedBlock].x;
  oldY = pixy.blocks[trackedBlock].y;
  oldSignature = pixy.blocks[trackedBlock].signature;
  return trackedBlock;
}
 
//---------------------------------------
// Follow blocks via the Zumo robot drive
//
// This code makes the robot base turn 
// and move to follow the pan/tilt tracking
// of the head.
//---------------------------------------
int32_t size = 400;
void FollowBlock(int trackedBlock)
{
  int32_t followError = RCS_CENTER_POS - panLoop.m_pos;  // How far off-center are we looking now?
 
  // Size is the area of the object.
  // We keep a running average of the last 8.
  size += pixy.blocks[trackedBlock].width * pixy.blocks[trackedBlock].height; 
  size -= size >> 3;
 
  // Forward speed decreases as we approach the object (size is larger)
  int forwardSpeed = constrain(400 - (size/256), -100, 400);  
 
  // Steering differential is proportional to the error times the forward speed
  int32_t differential = (followError + (followError * forwardSpeed))>>8;
 
  // Adjust the left and right speeds by the steering differential.
  int leftSpeed = constrain(forwardSpeed + differential, -400, 400);
  int rightSpeed = constrain(forwardSpeed - differential, -400, 400);
 
  // And set the motor speeds
  motors.setLeftSpeed(leftSpeed);
  motors.setRightSpeed(rightSpeed);
}
 
//---------------------------------------
// Random search for blocks
//
// This code pans back and forth at random
// until a block is detected
//---------------------------------------
int scanIncrement = (RCS_MAX_POS - RCS_MIN_POS) / 150;
uint32_t lastMove = 0;
 
void ScanForBlocks()
{
  if (millis() - lastMove > 20)
  {
    lastMove = millis();
    panLoop.m_pos += scanIncrement;
    if ((panLoop.m_pos >= RCS_MAX_POS)||(panLoop.m_pos <= RCS_MIN_POS))
    {
      tiltLoop.m_pos = random(RCS_MAX_POS * 0.6, RCS_MAX_POS);
      scanIncrement = -scanIncrement;
      if (scanIncrement < 0)
      {
        motors.setLeftSpeed(-175);
        motors.setRightSpeed(175);
      }
      else
      {
        motors.setLeftSpeed(+90);
        motors.setRightSpeed(-90);
      }
      delay(random(175, 250));
    }
 
    pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
  }
}
