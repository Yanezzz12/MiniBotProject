/*
Fifth practice by:
GONZALEZ VELASCO OSCAR EDUARDO
PEREZ YANEZ MIGUEL ANGEL
*/

//Class libraries
#include "Encoders.h"
#include "Utilities.h"
#include "TestSensors.h"

//Predetermined values
const float baseDistance = 10.0f;
const float baseAngle = 45.0f;
//Constants
#define MOTION ScrollPID(0.0f)
#define FORWARD ScrollPID(baseDistance)
#define BACK ScrollPID(-baseDistance)
#define TURNLEFT RotatePID(baseAngle)
#define TURNRIGHT RotatePID(-baseAngle)
#define STOP MotorMovement("OFF", 0);
#define MSG Serial.println("Hola mundo!")
#define INF 1000000

//Mathematic values
/* WheelDiameter = 4.3f cm
 * WheelPerimeter = 13.51f cm
 * DistanceBetweenWheels = 8.9f cm
 */
const float wheelPerimeter = 13.51f; //cm
const byte ticksPerRevolution = 300;
const float distanceAxis = 8.9f; // [cm]
//Pin setup
const byte leftMotor = 5;  //~ (Goes to 2)
const byte rightMotor = 6; //~ (Goes to 15)
const byte vel1 = 10;      //~ (Goes to 1)
const byte vel2 = 11;      //~ (Goes to 9)
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
//PID Control

//Strings
String commandStr;
String command1;
String command2;
String command3;
  
void setup() 
{ 
  Serial.begin(9600);  
  /*===PIN SETUP===*/
  //Motors
  pinMode(leftMotor, OUTPUT);
  pinMode(rightMotor, OUTPUT);
  pinMode(vel1, OUTPUT);
  pinMode(vel2, OUTPUT);
  /*===INITIAL VALUES===*/
  //Turns off motors
  digitalWrite(vel1, LOW);
  digitalWrite(vel2, LOW);
  digitalWrite(leftMotor, LOW);
  digitalWrite(rightMotor, LOW);  
  //Activates logic
  digitalWrite(currentBridge, HIGH);  
  //Sensors
  SensorSetup();
  //Encoder setup & interruptions
  SetEncodersInput();
}

void loop()
{
  //TestInfrared();
  //TestArrayLDR();
  //TestContact();
  //TestEncoders();
  
  //SerialTestMotors();
  //MaxLightValue();

  //MotorMovement("L", 60);
  //MotorMovement("R", 60);

  //ScrollPID(10.0f);
  //RotatePID(45.0f);
  
  //while(true){} //*/
  
  //MotorMovement("R", 40);
  //ObstacleAvoidance();

  StraightMovement(500); 

  //FORWARD;
  //while(true){}
  
  /*/ 
  Scroll(0.0f); RotateRobot(90.0f);
  while(true) {} 
  // */
}

//==========PID functions=========
long LPError = 0, RPError = 0;
long previousTime = 0;
long Iedt = 0;

long PID(long destiny, long currentPosition, long previousError, int index, float Kp, float Kd, float Ki)
{  
  //Variable declaration
  long currentTime;
  double dt;
  long error;
  double Dedt;
  long signal;
  
  //Control boundaries
  const float tolerance = 4.0f;

  currentTime = micros();
  dt = ((double)(currentTime - previousTime))/(1.0e6);
  previousTime = currentTime;
  error = destiny - currentPosition; //Check*

  //Derivative & Integral
  Dedt = (error - previousError)/(dt);
  Iedt = Iedt + (error * dt);

  if(index == 0)
    LPError = error;
  if(index == 1)
    RPError = error;

  //Control u(t) signal
  signal = (Kp * error) + (Kd * Dedt) + (Ki * Iedt);

  return signal;
}

int SignalProcessing(long signal, int minVelocity, int maxVelocity)
{



  if(uFunction[motor] > limitVelocity[motor])
    uFunction[motor] = limitVelocity[motor];
  else if((uFunction[motor] < minVelocity[motor]) && (uFunction[motor] >= 0))
    uFunction[motor] = minVelocity[motor];
  else if((uFunction[motor] < 0) && (uFunction[motor] > -minVelocity[motor])) 
    uFunction[motor] = -minVelocity[motor];
  else if(uFunction[motor] < -limitVelocity[motor])
    uFunction[motor] = -limitVelocity[motor];




  //return processedSignal;
}

void StraightMovement(long destiny)
{
  //Control constants
  const float LKp = 6.0f;
  const float LKd = 0.004;
  const float LKi = 0.025f;
  const float RKp = 6.0f;
  const float RKd = 0.004;
  const float RKi = 0.025f;

  //Variables
  long leftSignal, rightSignal;
  
  //Left movement
  leftSignal = PID(destiny, LeftCount(), LPError, 0, LKp, LKd, LKi);
  //Right movement
  //rightSignal, RPError = PID(destiny, RightCount(), RPError, 0, LKp, LKd, LKi);

  Serial.print("Left signal: ");
  Serial.println(leftSignal);
  delay(2000);
}

void TurnMovement()
{
  Serial.print("Nothing");
}

//==========PID functions=========

void PlotPID(long ticks)
{
  Serial.print("Target:");      Serial.print(ticks);         Serial.print(",");
  Serial.print("LeftWheel:");   Serial.print(LeftCount());   Serial.print(",");
  Serial.print("RightWheel:");  Serial.print(RightCount());  Serial.println(","); 
}

void ScrollPID(float distance) //Inactive funtion
{
  /*
  long ticks = DistanceToTicks(distance);
  unsigned long startTime;
  unsigned long currTime = 0;
  unsigned long settlingTime = 1000; //1 segundo

  while(true)
  {
    MotorPID(ticks, 'L');
    MotorPID(ticks, 'R');
    if(MotorPID(ticks, 'L') && MotorPID(ticks, 'R'))
    {
      startTime = millis();
      while(currTime < startTime + settlingTime)
      {
        currTime = millis();
        MotorPID(ticks, 'L');
        MotorPID(ticks, 'R');
        PlotPID(ticks);
      }
      MotorMovement("LOFF", 0);
      MotorMovement("ROFF", 0); 
      break; 
    }
    PlotPID(ticks);
  } // */
}

void RotatePID(float angle) //Inactive function
{
  /*
  long ticks = AngleToTicks(angle);
  unsigned long startTime;
  unsigned long currTime = 0;
  unsigned long settlingTime = 1000; //1 segundo

  while(true)
  {
    MotorPID(-ticks, 'L');
    MotorPID(ticks, 'R');
    if(MotorPID(-ticks, 'L') && MotorPID(ticks, 'R'))
    {
      startTime = millis();
      while(currTime < startTime + settlingTime)
      {
        currTime = millis();
        MotorPID(-ticks, 'L');
        MotorPID(ticks, 'R');
        PlotPID(ticks);
      }
      break; 
    }
    PlotPID(ticks);
  } // */
}

void LightFollowerAlgorithm()
{
  byte direction = MaxLightIndex();
  int MaxLightValue = 400; //Bias 

  switch(direction) //Modificar casos
  {
    case 0: //Luz enfrente
      //ScrollPID(); //Move indefinetely
      break;
    case 1: //Luz a la izquierda
      //RotatePID(); //Hasta que el mayor valor sea el del LDR de enfrente
      break;
    case 2: //Luz a la derecha
      //RotatePID(); //Hasta que el mayor valor sea el del LDR de enfrente
      break;
    case 3: //Luz detrás
      //RotatePID(); //Hasta que el mayor valor sea el del LDR de enfrente
      break; 
  }

  //if(LDR[enfrente] > umbralLuzMaximo) -> MiniBot se detiene
  //if(LDRvalue[direction] > MaxLightValue) 
  //  STOP;
}

void Scroll(float distance)     //Test function
{
  //This parameters shows a 10 [cm] scroll
  const float baseDistance = 10.0f;
  const float baseTime = 700.0f;
  const int deviation = 50;
  const int leftVel = 45;
  const int rightVel = 38; //54
  
  float redefine = abs(distance) / baseDistance;

  if(distance > 999)
  {
    MotorMovement("L", rightVel);
    MotorMovement("R", -leftVel);
  }
  else if(distance > 0)
  {
    MotorMovement("L", leftVel);
    MotorMovement("R", -rightVel);
    delay(int(baseTime * redefine) + deviation);
    MotorMovement("OFF", 0);
  }
  else if(distance < 0) 
  {
    MotorMovement("L", -leftVel);
    MotorMovement("R", rightVel);
    delay(int(baseTime * redefine) + deviation);
    MotorMovement("OFF", 0);
  }
  else if(distance == 0)
  {
    MotorMovement("OFF", 0);
  }
  else { }
}

void RotateRobot(float angle)   //Test function
{ 
  //This parameters show a 90 degree rotation
  const float baseRotation = 90.0f;
  const float baseTime = 800.0f;
  const int deviation = 50;
  const int leftVel = 40;
  const int rightVel = 40;
  
  float redefine = abs(angle) / baseRotation;

  if(angle > 0)
  {
    MotorMovement("L", rightVel);
    MotorMovement("R", leftVel);
    delay(int(baseTime * redefine) + deviation);
    MotorMovement("OFF", 0);
  }
  else if(angle < 0)
  {
    MotorMovement("L", -rightVel);
    MotorMovement("R", -leftVel);
    delay(int(baseTime * redefine) + deviation);
    MotorMovement("OFF", 0);
  }
  else{}
}

void MotorMovement(String command, int speedPWM) 
{
  //Allows motor movement, SpeedPWM: Range[-127, 127]
  /*
  Serial.print("Command: ");
  Serial.print(command);
  Serial.print(", ");
  Serial.println(speedPWM); // */

  if(command == "L" || command == "R") //Can be tweaked yet
  {
    if(-10 < speedPWM && speedPWM < 10)
    {
      if(command == "L") 
        MotorMovement("LOFF", 0);
      else if(command == "R")
        MotorMovement("ROFF", 0);
      return;
    }  
  }
  
  speedPWM = Map(speedPWM);

  if(command == "L")
  {
    digitalWrite(vel1, HIGH);
    analogWrite(leftMotor, -speedPWM);  
  }
  else if(command == "R")
  {
    digitalWrite(vel2, HIGH);
    analogWrite(rightMotor, speedPWM);
  }
  else if(command == "LOFF")
  {
    digitalWrite(leftMotor, LOW);
    digitalWrite(vel1, LOW); 
  }
  else if(command == "ROFF")
  {
    digitalWrite(rightMotor, LOW);
    digitalWrite(vel2, LOW); 
  }
  else if(command == "OFF")
  {
    digitalWrite(leftMotor, LOW);
    digitalWrite(vel1, LOW);
    digitalWrite(rightMotor, LOW);
    digitalWrite(vel2, LOW); 
  }
  else
    Serial.println("Unknown command!");
}

String ReadCommands()
{
  while (Serial.available() == 0) {} 
  commandStr = Serial.readString();
  commandStr.trim();  

  command1 = "";
  command2 = "";

  int wordCount = 0;
  for(int i = 0; i < commandStr.length(); i++)
  {
    if(commandStr[i] == ' ')      
      wordCount++;
    else if(wordCount == 0)
      command1 += commandStr[i];
    else if(wordCount == 1)
      command2 += commandStr[i];
  }
  return command1, command2;  
}

byte MaxLightIndex()
{
  //Detects LDR with higher value and returns an index
  const byte sensorQuantity = 5;
  byte maxLightIndex = 0;
  int LDRvalue[sensorQuantity];
  
  for(byte i = 0; i < sensorQuantity; i++) //Checks all LDR values
    LDRvalue[i] = analogRead(i + 14);
    
  for(byte i = 1; i < sensorQuantity; i++)
    if(LDRvalue[maxLightIndex] < LDRvalue[i])
      maxLightIndex = i;

  Serial.print("Max value: ");
  Serial.println(maxLightIndex); // */
  
  return maxLightIndex;
}

int LDRArray()
{
  //Returns the array of values of light sensors
  const byte sensorQuantity = 5;
  int LDRvalue[sensorQuantity];

  for(byte i = 0; i < sensorQuantity; i++) //Checks all LDR values
    LDRvalue[i] = analogRead(i + 14);

  return LDRvalue;
}

void ObstacleAvoidance() //Obstacle avoidance algorithm 
{
  /*  COMMANDS
  ContactSensor('L', 'R');
  FORWARD
  BACK
  TURNLEFT
  TURNRIGHT
  STOP
  */

  int state = 0;

  state = 3 - (ContactSensor('L') * 2 + ContactSensor('R') * 1);

  /*
  if(state == 0)
    state = 3 - (InfraredSensor('L') * 2 + InfraredSensor('R') * 1);*/

  switch(state)
  {
    case 0: //No obstacle in front
      LightFollowerAlgorithm();
      break;
    case 1: //Obstacle to the right
      STOP;
      BACK;
      TURNLEFT;
      break;
    case 2: //Obstacle to the left
      STOP;
      BACK;
      TURNRIGHT;
      break;
    case 3: //Obstacle in front of the robot
      STOP;
      BACK;
      TURNLEFT;
      TURNLEFT;
      FORWARD;
      TURNRIGHT;
      break;
 }
}

void SerialTestMotors()
{
  command1, command2 = ReadCommands();
  MotorMovement(command1, command2.toInt());
}