#include <Wire.h>
#include "gyro_accel.h"
#include <Servo.h>
// Defining constants
#define dt 20                       // time difference in milli seconds
#define rad2degree 57.3              // Radian to degree conversion
#define Filter_gain 0.95             // e.g.  angle = angle_gyro*Filter_gain + angle_accel*(1-Filter_gain)

// wheels
#define pinLB 2
#define pinLF 4
#define pinRB 7
#define pinRF 8

// adjusting speed
#define Lpwm_pin 5
#define Rpwm_pin 10
#define Lpwm_val 50
#define Rpwm_val 50

// ultrasonic sensor
#define inputPin A0
#define outputPin A1

// speed by directions
#define Fspeed 0
#define Rspeed 0
#define Lspeed 0

#define delay_time 250

// directions
#define Fgo 8
#define Rgo 6
#define Lgo 4
#define Bgo 2
// *********************************************************************
//    Global Variables
// *********************************************************************
unsigned long t=0; // Time Variables
float angle_x_gyro=0,angle_y_gyro=0,angle_z_gyro=0,angle_x_accel=0,angle_y_accel=0,angle_z_accel=0,angle_x=0,angle_y=0,angle_z=0;
float integral=0, t0, correction_factor;
bool f;

Servo servo;
int directionn = 0; //front=8, back=2, left=4, right=6
// *********************************************************************
// Main Code
void setup(){
  Serial.begin(9600);
  Wire.begin();

  // Calibration
  MPU6050_ResetWake();
  MPU6050_SetGains(0,1);// Setting the lows scale
  MPU6050_SetDLPF(0); // Setting the DLPF to inf Bandwidth for calibration
  MPU6050_OffsetCal();
  MPU6050_SetDLPF(6); // Setting the DLPF to lowest Bandwidth

  // motors, servo, ultrasonic
  pinMode(pinLB, OUTPUT);
  pinMode(pinLF, OUTPUT);
  pinMode(pinRB, OUTPUT);
  pinMode(pinRF, OUTPUT);

  pinMode(inputPin, INPUT);
  pinMode(outputPin, OUTPUT);

  servo.attach(3);
    
  t=millis(); 
  f = true;

  float z0 = gyro_z_scalled;
  while((millis()-t) < 10 * dt){ // Making sure the cycle time is equal to dt  // И это тоже
  // Do nothing
  }
  correction_factor = (gyro_z_scalled - z0) / 10;
}
void loop(){
  t=millis(); 
  
  MPU6050_ReadData();
 
  //Serial.print(gyro_z_scalled);
  //Serial.print("\t");
  integral += (gyro_z_scalled + correction_factor) * dt / 1000;
  Serial.print(integral);
  Serial.print("\t");

  Serial.println(((float)(millis()-t)/(float)dt)*100);  // Не знаю, что это, но без этого не работает. Магия!

  if (f)
  {
    turn(left, 90);
    f = false;
  }
  else
    forward();
  
  while((millis()-t) < dt){ // Making sure the cycle time is equal to dt  // И это тоже
  // Do nothing
  }
}

void forward()
{
  digitalWrite(pinLB, LOW);
  digitalWrite(pinRB, LOW);
  digitalWrite(pinLF, HIGH);
  digitalWrite(pinRF, HIGH);
}

void left()
{
  digitalWrite(pinLB, LOW);
  digitalWrite(pinRB, HIGH);
  digitalWrite(pinLF, HIGH);
  digitalWrite(pinRF, LOW);
}

void stopp()
{
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinRB, LOW);
  digitalWrite(pinLF, HIGH);
  digitalWrite(pinRF, LOW);
}

void right()
{
  digitalWrite(pinLB, HIGH);
  digitalWrite(pinRB, LOW);
  digitalWrite(pinLF, LOW);
  digitalWrite(pinRF, HIGH);
}

void turn(void (*dir)(), int angle)
{
  int start = integral;
  stopp();
  while(abs(integral - start) < angle)
  {
    readData();
    (*dir)();
  }
  stopp();
}

void readData()
{
  MPU6050_ReadData();
  integral += (gyro_z_scalled + correction_factor) * dt / 1000;
  Serial.println(integral);
  while((millis()-t) < dt){ // Making sure the cycle time is equal to dt  // И это тоже
  // Do nothing
  }
}
