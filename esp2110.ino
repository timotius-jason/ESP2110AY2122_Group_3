#include <Pixy2.h>
#include <PIDLoop.h>
#include <ZumoMotors.h>
#include <ZumoBuzzer.h>

// Zumo speeds, maximum allowed is 400
#define ZUMO_FAST         285
#define ZUMO_SLOW         200
#define ZUMO_SPIN         300
#define X_CENTER          (pixy.frameWidth/2)

// Pixy camera angle
#define PIXY_X            500            
#define PIXY_Y            740

// Constants for when no vector is found
#define SPIN_MAX_LOOP_NUMBER 75

Pixy2 pixy;
ZumoMotors motors;
ZumoBuzzer buzzer;
int8_t spin_count;
bool need_spin;

PIDLoop headingLoop(8500, 0.01, 520, false);

void setup() 
{
  Serial.begin(115200);
  Serial.print("Starting...\n");
  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);

  pixy.init();
  // Turn on both lamps, upper and lower for maximum exposure
  pixy.setLamp(1, 1);
  // change to the line_tracking program.  Note, changeProg can use partial strings, so for example,
  // you can change to the line_tracking program by calling changeProg("line") instead of the whole
  // string changeProg("line_tracking")
  pixy.changeProg("line");

  // look straight and down
  pixy.setServos(PIXY_X, PIXY_Y);

  spin_count = 0;
  need_spin = true;
}


void loop()
{
  int8_t res;
  int32_t error; 
  int left, right;

  // Get latest data from Pixy, including main vector, new intersections and new barcodes.
  res = pixy.line.getMainFeatures();

  // If error or nothing detected, spin around to find a vector
  if (res<=0) 
  {
    if(!need_spin)
    {
      return;
    }
    if(spin_count >= SPIN_MAX_LOOP_NUMBER)
    {
      spin_count = 0;
      left = 0;
      right = 0;
      need_spin = false;
    }
    else
    {
      spin_count += 1;
      left = ZUMO_SPIN;
      right = -ZUMO_SPIN;  
    }
    motors.setLeftSpeed(left);
    motors.setRightSpeed(right);
    buzzer.playFrequency(500, 50, 15);
    Serial.print("stop ");
    Serial.println(res);
    return;
  }

  // We found the vector...
  if (res&LINE_VECTOR)
  {
    // Calculate heading error with respect to m_x1, which is the far-end of the vector,
    // the part of the vector we're heading toward.
    error = (int32_t)pixy.line.vectors->m_x1 - (int32_t)X_CENTER;

    // pixy.line.vectors->print();

    // Perform PID calcs on heading error.
    headingLoop.update(error);

    // separate heading into left and right wheel velocities.
    left = headingLoop.m_command;
    right = -headingLoop.m_command;

    // If vector is heading away from us (arrow pointing up), things are normal.
    if (pixy.line.vectors->m_y0 > pixy.line.vectors->m_y1)
    {
      // ... but slow down a little if intersection is present, so we don't miss it.
      if (pixy.line.vectors->m_flags&LINE_FLAG_INTERSECTION_PRESENT)
      {
        left += ZUMO_SLOW;
        right += ZUMO_SLOW;
      }
      else // otherwise, pedal to the metal!
      {
        left += ZUMO_FAST;
        right += ZUMO_FAST;
      }    
    }
    else  // If the vector is pointing down, or down-ish, reverse the vector.
    {
      pixy.line.reverseVector();  
    } 
    motors.setLeftSpeed(left);
    motors.setRightSpeed(right);
    need_spin = true;
    spin_count = 0;
  }

}
