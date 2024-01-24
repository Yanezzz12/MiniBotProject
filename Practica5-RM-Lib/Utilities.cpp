/*
 * Utilities.h has some useful functions
 */

#include "Utilities.h"
#include <Arduino.h>

float AngleCorrection(float angle)
{  
  while(angle > 180)
    angle -= 360;
  while(angle < -180)
    angle += 360;
  return angle;
}

long DistanceToTicks(float distance)
{
  long ticks = 22.206f * distance;
  return ticks;
}

long AngleToTicks(float angle)
{
  angle = AngleCorrection(angle);
  long ticks = (2.35 * (angle / 360)) * 300;
  return ticks;
}

short Map(short value) 
{
  //Input: Range[-127, 127], Output: Range[0, 255]
  value += 127;
  return value;
}

float ErrorToPWM(long maxError, float currentError)
{
  /* This function transforms an e(t) to a F(s) signal able to get in the plant (Motor)
   * The mathematic model is linear, described by m = (maxPWMValue - minPWMValue) / (maxError - minError)
   * m = (60 - 20) / (maxError - 0) => m = 66/target => F(s) = [66*e(t)]/target + minPWMValue
  */
  currentError = (40.0f * currentError)/maxError + 20.0f;
  return currentError;
}
