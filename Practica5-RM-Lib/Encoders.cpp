/*
 * Encoders.h library sends encoders data
 */

#include "Encoders.h"
#include <Arduino.h>

const byte rightEncoderA = 2; //Interruption 1
const byte leftEncoderA = 3; //Interruption 2
const byte leftEncoderB = 4;
const byte rightEncoderB = 7;

volatile long rightCount = 0;
volatile long leftCount = 0;

void LeftEncoderInterrupt()
{
  if(digitalRead(leftEncoderA) == HIGH)
    if(digitalRead(leftEncoderB) == LOW)
      leftCount--;
    else
      leftCount++;
  else
    if(digitalRead(leftEncoderB) == LOW)
      leftCount++;
    else
      leftCount--;
}

void RightEncoderInterrupt()
{
  if(digitalRead(rightEncoderA) == HIGH)
    if(digitalRead(rightEncoderB) == LOW)
      rightCount++;
    else
      rightCount--;
  else
    if(digitalRead(rightEncoderB) == LOW)
      rightCount--;
    else
      rightCount++;
}

volatile long LeftCount()
{
  return leftCount;
}

volatile long RightCount()
{
  return rightCount;
}

void EncoderSetup(){
	pinMode(leftEncoderA, INPUT);
  pinMode(rightEncoderA, INPUT);
  pinMode(leftEncoderB, INPUT);
  pinMode(rightEncoderB, INPUT);

	attachInterrupt(digitalPinToInterrupt(leftEncoderA), LeftEncoderInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(rightEncoderA), RightEncoderInterrupt, RISING);
}
