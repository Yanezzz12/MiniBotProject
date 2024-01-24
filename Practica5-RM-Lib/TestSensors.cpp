/*
 * TestSensors.h sends information about any sensor in the robot
 */

#include "TestSensors.h"
#include "Encoders.h"
#include <Arduino.h>

//Sensors
const byte leftContactSens = 13; 
const byte rightContactSens = 8; 
const byte currentBridge = 12;
const byte leftInfraSens = 19;   
const byte rightInfraSens = 9;
//LDR sensors
const byte LDR1 = 14; // Yellow   (A0)
const byte LDR2 = 15; // White    (A1)
const byte LDR3 = 16; // Orange   (A2)
const byte LDR4 = 17; // Green    (A3)
const byte LDR5 = 18; // Purple   (A4)

void SensorSetup()
{
  //Sensors
  pinMode(currentBridge, OUTPUT);
  pinMode(leftInfraSens, INPUT); 
  pinMode(rightInfraSens, INPUT);
  pinMode(leftContactSens, INPUT_PULLUP);
  pinMode(rightContactSens, INPUT_PULLUP);
  //LDR sensors
  pinMode(LDR1, INPUT);
  pinMode(LDR2, INPUT);
  pinMode(LDR3, INPUT);
  pinMode(LDR4, INPUT);
  pinMode(LDR5, INPUT);
}

void TestEncoders()
{
  Serial.print("Left: ");
  Serial.print(LeftCount());
  Serial.print(", Right: ");
  Serial.println(RightCount());
  delay(100);
}

bool InfraredSensor(char sensor) 
{
  //Reads an infrared sensor and returns a boolean value if exceeds sensitivity
  float infraValue, sensitivity = 400.0f;
  
  if(sensor == 'L')
    infraValue = analogRead(leftInfraSens);
  else if(sensor == 'R')
    infraValue = analogRead(rightInfraSens);

  if(infraValue > sensitivity)
    return 0;
  else 
    return 1;
}

bool ContactSensor(char sensor)
{
  //Reads a contact sensor and returns a boolean value
  bool registeredValue;

  if(sensor == 'L')
    registeredValue = digitalRead(leftContactSens);
  else if(sensor == 'R')
    registeredValue = digitalRead(rightContactSens);
  return registeredValue; 
}

void TestContact()
{
  Serial.print("Left: ");   Serial.print(ContactSensor('L'));
  Serial.print(", Right: ");  Serial.println(ContactSensor('R'));
  delay(1000);
}

void TestInfrared()
{
  Serial.print("Lectura Izq: "); 
  Serial.print(analogRead(leftInfraSens)); 

  Serial.print(", Lectura Derecha: "); 
  Serial.println(analogRead(rightInfraSens)); 

  delay(600);
}
 
void TestArrayLDR()
{
  const byte sensorQuantity = 5;
  unsigned int LDRvalue[sensorQuantity];
  
  for(byte i = 0; i < sensorQuantity; i++)
    LDRvalue[i] = analogRead(i + 14);
  
  Serial.print("Valores: ");
  for(byte i = 0; i < sensorQuantity; i++)
  {
    Serial.print(LDRvalue[i]);
    Serial.print(", ");
  }
  Serial.println(" ");
  delay(1000);
}
