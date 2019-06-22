#include <Servo2.h>
#include <StaticThreadController.h>
#include <Thread.h>
#include <ThreadController.h>
#include <Wire.h>
#include "gyro_accel.h"

// Defining constants
#define dt 80                       // time difference in milli seconds; должен соответсововать хотя бы примерно времени работы loop
#define rad2degree 57.3              // Radian to degree conversion
#define Filter_gain 0.95             // e.g.  angle = angle_gyro*Filter_gain + angle_accel*(1-Filter_gain)
#define correction_factor 0.9
#define rotation_factor 1.667
#define speed_factor 150

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
#define S 5

#define minDist 15
// *********************************************************************
//    Global Variables
// *********************************************************************
unsigned long t=0; // Time Variables
float angle_x_gyro=0,angle_y_gyro=0,angle_z_gyro=0,angle_x_accel=0,angle_y_accel=0,angle_z_accel=0,angle_x=0,angle_y=0,angle_z=0;
float integral=0, t0, distL, distF, distR;
bool f;
int mode = -1; // 0 - авто, 1 - ручной
int dir = S; // направление движения
int val = -1;

Servo2 servo;

Thread angleThread = Thread();
Thread corrThread = Thread();
Thread detectionThread = Thread();
// *********************************************************************
void turn(void (*dir)(), int angle, bool corr = false)
{
  int start = integral;
  stopp();
  while(abs(integral - start) < angle)
  {
    if (angleThread.shouldRun())
      angleThread.run();
    (*dir)();
  }
  stopp();
  if (!corr)  // Если мы именно поворачиваем, а не корректируем снос вправо
    integral = 0;
}

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
  MPU6050_ReadData();

  // motors, servo, ultrasonic
  pinMode(pinLB, OUTPUT);
  pinMode(pinLF, OUTPUT);
  pinMode(pinRB, OUTPUT);
  pinMode(pinRF, OUTPUT);
  pinMode(Lpwm_pin, INPUT);
  pinMode(Rpwm_pin, INPUT);

  pinMode(inputPin, INPUT);
  pinMode(outputPin, OUTPUT);

  servo.attach(3);

  angleThread.onRun(readData);
  angleThread.setInterval(dt);

  corrThread.onRun(correction);
  corrThread.setInterval(dt);

  detectionThread.onRun(detection);
  detectionThread.setInterval(750);

  analogWrite(Lpwm_pin, speed_factor);
  analogWrite(Rpwm_pin, speed_factor);
}

void loop(){
  if (angleThread.shouldRun())
    angleThread.run();

  ReadCommand();
  if (mode == 1)
    ManualModeGo();
  else if (mode == 0)
  {
    if (detectionThread.shouldRun())
      detectionThread.run();
      
    if (dir != S)
      AutoModeGo();
    else
      stopp();
  }
}

void forward()
{
  if (corrThread.shouldRun())
    corrThread.run();
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

void readData()
{
  MPU6050_ReadData();
  integral += (int)(gyro_z_scalled * 10.0) / 10.0 * dt / 1000 / correction_factor;  // Угол поворота считаем как интеграл gyro_z_scalled по времени
}

void correction() // Исправляет снос вправо
{
  if (integral < -10)
    turn(left, abs(integral), true);
}

void detection()   // n - угол поворота сервы (0 - право, 180 - лево, 90 - прямо)
{
  servo.write(0);
  delay(30);
  digitalWrite(outputPin, LOW); // ultrasonic launching low voltage at 2μs
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH); // ultrasonic launching high voltage at 10μs，at least at10μs
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW); // keeping ultrasonic launching low voltage
  distL = pulseIn(inputPin, HIGH); // time of error reading
  distL = distL/5.8/10; // converting time into distance（unit：cm）

  delay(250);

  servo.write(90);
  delay(30);
  digitalWrite(outputPin, LOW); 
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW); 
  distF = pulseIn(inputPin, HIGH);
  distF = distF/5.8/10;

  delay(250);

  servo.write(180);
  delay(30);
  digitalWrite(outputPin, LOW); 
  delayMicroseconds(2);
  digitalWrite(outputPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(outputPin, LOW); 
  distR = pulseIn(inputPin, HIGH);
  distR = distR/5.8/10;
}

void ReadCommand()
{
  int val = Serial.read();
  if (val == '0')
  {
    if (mode == 1)  // При смене режима (не волнуйтесь, товарищ майор, это не то) было бы неплохо остановить устройство
      stopp();
    mode = 0;
  }
  else if (val == '1')
  {
    if (mode == 0)
      stopp();
    mode = 1;
  }
  else if (val == '4' && mode == 1)
    dir = Lgo;
  else if (val == '6' && mode == 1)
    dir = Rgo;
  else if (val == '8' && mode == 1)
    dir = Fgo;
  else if (val == '5')
    dir = S;
}

void ManualModeGo()
{
  if (dir == S)
    stopp();
  else if (dir == Fgo)
    forward();
  else if (dir == Rgo)
    turn(right, 5);
  else if (dir == Lgo)
    turn(left, 5);
}

void AutoModeGo()
{
  
}
