#include <Wire.h>
//PWM library
#include <Adafruit_PWMServoDriver.h>
//compass sensor library
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

//matrix and vector implementation
#include "Matrix.h"
//MPU gyro sensor implementation
#include "Gyro.h"

/************************* variables ****************************/

const int servoCount = 6;
//constants for pulse widths at 12 bit PWM
const int servoPulseMin = 128;
const int servoPulseMax = 640;
const int servoPulseWidth = servoPulseMax - servoPulseMin;

//analog normatization values
int analogMin = 0;
const int analogMax = 1023;
//the from 0 to 0.99999. the higher the smoother the movements but also more delayed
const float attenuation = 0.8;

//reference compass vector
Vector compass0;

//servo ranges [min, max] from 0 to 1. This prevents that the arm hits itself and the servos overheat
float servoRange[servoCount][2] = {
  {0.2, 0.9}, 
  {0.35, 0.95}, 
  {0.0, 1}, 
  {0.25, 1}, 
  {0, 0.75}, 
  {0.15, 0.4}};

Vector sensors[4];  //vectors of the sensors
float analog = 0; //last analog reading of the hall effect
float angles[servoCount] = {-1};  //angle values of the joints
float servoValue[servoCount] = {0.5, 0.4, 0.2, 0.25, 0.0, 0.2};   //values the servos will be set to [0; 1]
int reversedRotation[servoCount] = {0, 1, 0, 1, 0, 0};  //needed to reverse rotation for servos that are flipped
float offsetValue[servoCount] = {0.5, 0.4, 0.2, 0.25, 0.0, 0.2};  //offset for the angle 0

//sensor class initializations
Gyro accel1(0, 1, 0x68);
Gyro accel2(0, 1, 0x69);
Adafruit_LSM303_Accel_Unified accel3 = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag1 = Adafruit_LSM303_Mag_Unified(12345);

//pwm driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

///reads all sensor values and normalizes all except for the magnetometer
void readSensors()
{
  //read the sensor vlaues
  accel1.poll();
  accel2.poll();  
  sensors_event_t accel3e;
  accel3.getEvent(&accel3e);  
  sensors_event_t mag1e;
  mag1.getEvent(&mag1e);

  //set the vectors and normalize
  sensors[0] = Vector(accel1.positionA[0], accel1.positionA[1], accel1.positionA[2]);
  sensors[0].normalize();

  sensors[1] = Vector(accel2.positionA[0], accel2.positionA[1], accel2.positionA[2]);
  sensors[1].normalize();

  sensors[2] = Vector(accel3e.acceleration.x, accel3e.acceleration.y, accel3e.acceleration.z);
  sensors[2].normalize();
  sensors[3] = Vector(mag1e.magnetic.x, mag1e.magnetic.y, mag1e.magnetic.z);
  //sensors[3].normalize();

  //scale the analog reading to [0; 1]
  analog = float(analogRead(A0) - analogMin) / (analogMax - analogMin);
}

///initial calibration, reference values for the gripper and the yaw are read
void calibrate()
{
  //get minimal value when gipper is open
  analogMin = analogRead(A0);
  //read the sensors
  readSensors();
  //calculate the absolute compass vector
  float a1 = -atan2(-sensors[1][1], -sensors[1][0]);
  Matrix r0 = Matrix::rotation(-a1, 0, 1, 0);
  float a2 = atan2(sensors[2][1], sensors[2][2]);
  Matrix r1 = Matrix::rotation(angles[4], 1, 0, 0);
  Vector v = r0 * r1 * sensors[3];
  v[2] = 0;
  v.normalize();
  compass0 = v;
}

void setup() 
{
  //setup serial
  Serial.begin(115200);
  Serial.println("Setup");
  //enable gyro sensors
  accel1.begin();
  accel2.begin();
  if(!accel3.begin())
    Serial.println("LSM303 accelerometer not detected");
  //enable magnetometer
  if(!mag1.begin())
    Serial.println("LSM303 magnetometer not detected");
  //enable pwm board and set the frequency to 50Hz
  pwm.begin();
  pwm.setPWMFreq(50);
  for(int i = 0; i < servoCount; i++)
    pwm.setPWM(i, 4096, 0);
  //get initial readings
  calibrate();
}

///calculate the servo values form the angles
void setServoAngle(int servo, float angle)
{
  //convert to [0; 1] scale and clip to min max range
  float v = max(servoRange[servo][0], min(servoRange[servo][1], angle / float(M_PI) + offsetValue[servo]));
  //attenuate if not forst values
  if(servoValue[servo] >= 0)
      servoValue[servo] = servoValue[servo] * attenuation + v * (1 - attenuation);
  else 
    servoValue[servo] = v;
}

///writes the servo values to the servos
void updateServos()
{
  for(int i = 0; i < servoCount; i++)
    if(servoValue[i] < 0)
      pwm.setPWM(i, 4096, 0);
    else
      pwm.setPWM(i, 0, int(servoPulseMin + (reversedRotation[i] ? 1 - servoValue[i]: servoValue[i]) * servoPulseWidth)); 
}

///turn off a servo
void disableServo(int servo)
{
  servoValue[servo] = -1;
}

///outputs all measured sensor values
void printSensors()
{
  for(int i = 0; i < 4; i++)
  {
    for(int j = 0; j < 3; j++)
    {
      Serial.print(sensors[i][j]); Serial.print(' ');
    }
    Serial.print("| ");
  }
  Serial.print(analog);
  Serial.print("   ");
}

///calculates the angles from the sensor values
void calculateAngles()
{
  //shoulder roll
  angles[1] = -atan2(-sensors[0][2], -sensors[0][1]);
  //shoulder pitch
  angles[2] = atan2(-sensors[0][0], -sensors[0][1]);
  float elbowPitch = atan2(-sensors[1][0], -sensors[1][1]);
  //setting relative pitch to shoulder
  angles[3] = elbowPitch - angles[2];
  //wrist roll
  angles[4] = atan2(sensors[2][1], sensors[2][2]);
  //gripper depending on analog value. We take the the square toot to get a linear movement. (point sources attenuate quadratically)
  angles[5] = (sqrt(max(analog, 0.f)) * (servoRange[5][1] - servoRange[5][0])) * M_PI;

  //calculating reverse tansformation for lower arm pitch
  float a1 = -atan2(-sensors[1][1], -sensors[1][0]);
  Matrix r0 = Matrix::rotation(-a1, 0, 1, 0);
  
  //calculating reverse transformation of wrist roll
  float a2 = atan2(sensors[2][1], sensors[2][2]);
  Matrix r1 = Matrix::rotation(angles[4], 1, 0, 0);
  
  //transforming the magnetic field vector back
  Vector v = r0 * r1 * sensors[3];
  //switching to flat earth compass
  v[2] = 0;
  v.normalize();
  //calculating angle between startup and current compass value
  Vector a = v.cross(compass0);
  angles[0] = -asin(a[2]);

  //setting the values
  setServoAngle(0, angles[0]);
  setServoAngle(1, angles[1]);
  setServoAngle(2, angles[2]);
  setServoAngle(3, angles[3]);
  setServoAngle(4, angles[4]);
  setServoAngle(5, angles[5]);
}

void printAngles()
{
  for(int i = 0; i < servoCount; i++)
  {
    Serial.print(angles[i]); Serial.print(' ');
  }
  Serial.println();
}

///main loop
void loop()
{
  static int lastServoUpdate = 0;
  //get current time
  int t = millis();
  static int time = 0;
  int dt = t - time;
  if (time == 0) dt = 0;
  time = t;

  //read sensors, calculate angles and set servos
  //servo PWM cant be updated more often than every 20ms
  if(time - lastServoUpdate > 20)
  {
    lastServoUpdate = time;
    readSensors();
    printSensors();
    calculateAngles();
    printAngles();
    updateServos();
  }
}


