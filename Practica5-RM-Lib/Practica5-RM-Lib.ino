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
const byte currentBridge = 12;
const byte leftMotor = 5;  //~ (Goes to 2)
const byte rightMotor = 6; //~ (Goes to 15)
const byte pinPWM1 = 10;      //~ (Goes to 1)
const byte pinPWM2 = 11;      //~ (Goes to 9)
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
  //Motor setup
  pinMode(rightMotor, OUTPUT);
  pinMode(leftMotor, OUTPUT);
  pinMode(pinPWM1, OUTPUT);
  pinMode(pinPWM2, OUTPUT);
  pinMode(currentBridge, OUTPUT);
  //Pin setup
  SensorSetup();
  EncoderSetup();
  /*===INITIAL VALUES===*/
  //Turns off motors
  digitalWrite(pinPWM1, LOW);
  digitalWrite(pinPWM2, LOW);
  digitalWrite(leftMotor, LOW);
  digitalWrite(rightMotor, LOW);  
  //Activates logic
  digitalWrite(currentBridge, HIGH);  
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
  
  //MotorMovement("R", 45);
  //ObstacleAvoidance();

  StraightMovement(8.0f); 

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
long previousError = 0;

long PID(long destiny, long currentPosition, long previousError, int index, float Kp, float Kd, float Ki)
{  
  //Variable declaration
  long currentTime;
  double dt;
  long error;
  double Dedt;
  long signal;

  //Calculus operations
  currentTime = micros();
  dt = ((double)(currentTime - previousTime))/(1.0e6);
  previousTime = currentTime;
  error = destiny - currentPosition; //Check*

  //Derivative & Integral
  Dedt = (error - previousError)/dt;
  Iedt = Iedt + (error * dt);

  //Control u(t) signal
  signal = (Kp * error) + (Kd * Dedt) + (Ki * Iedt);

  if(index == 0)
    LPError = error;
  if(index == 1)
    RPError	 = error;

  return signal;
}

int SignalProcessing(long signal, int minVelocity, int maxVelocity)
{
  if(signal > maxVelocity)
    signal = maxVelocity;
  else if((signal < minVelocity) && (signal >= 0))
    signal = minVelocity;
  else if((signal < 0) && (signal > -minVelocity))
    signal = -minVelocity;
  else if(signal < -maxVelocity)
    signal = -maxVelocity;

  return signal;
}

//Function variables
long startTime = 0;

void StraightMovement(float distance)
{
  //Variables
  long leftSignal, rightSignal, destiny;

  //Control constants
  const float LKp = 1.0f;
  const float LKd = 0.00;
  const float LKi = 0.0f;

  const float RKp = 1.0f;
  const float RKd = 0.00;
  const float RKi = 0.0f;

  //Velocities
  int leftVel[2]  = {15, 60};
  int rightVel[2] = {15, 50};

  //Inputs
  const long tolerance = 5;
  long settlingTime = 1000; //1 segundo

  destiny = DistanceToTicks(distance);

  //PID 
  leftSignal = PID(destiny, LeftCount(), LPError, 0, LKp, LKd, LKi);
  rightSignal = PID(destiny, RightCount(), RPError, 1, RKp, RKd, RKi);
  //Signal processing
  leftSignal = SignalProcessing(leftSignal, leftVel[0], leftVel[1]);
  rightSignal = SignalProcessing(rightSignal, rightVel[0], rightVel[1]);

  //if(destiny - tolerance < currentPosition < destiny + tolerance)
  bool leftGoal = (destiny - tolerance < LeftCount()) && (LeftCount() < destiny + tolerance);
  bool rightGoal = (destiny - tolerance < RightCount()) && (RightCount() < destiny + tolerance);

  if(leftGoal || rightGoal) //Aún falta corregir
    MotorMovement("OFF", 0);
  else
  {
    MotorMovement("L", leftSignal);
    MotorMovement("R", rightSignal);
  }
  
  PlotPID(destiny, LeftCount(), RightCount());
}

void TurnMovement()
{
  Serial.print("Nothing");
}


void PlotPID(long goal, long value1, long value2)
{
  Serial.print("Target:");      Serial.print(goal);         Serial.print(",");
  Serial.print("LeftWheel:");   Serial.print(value1);   Serial.print(",");
  Serial.print("RightWheel:");  Serial.print(value2);  Serial.println(","); 
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

//==========PID functions=========

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

  if(command == "L" || command == "R") //Can be tweaked
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
    digitalWrite(pinPWM1, HIGH);
    analogWrite(leftMotor, -speedPWM);  
  }
  else if(command == "R")
  {
    digitalWrite(pinPWM2, HIGH);
    analogWrite(rightMotor, speedPWM);
  }
  else if(command == "LOFF")
  {
    digitalWrite(leftMotor, LOW);
    digitalWrite(pinPWM1, LOW); 
  }
  else if(command == "ROFF")
  {
    digitalWrite(rightMotor, LOW);
    digitalWrite(pinPWM2, LOW); 
  }
  else if(command == "OFF")
  {
    digitalWrite(leftMotor, LOW);
    digitalWrite(pinPWM1, LOW);
    digitalWrite(rightMotor, LOW);
    digitalWrite(pinPWM2, LOW); 
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