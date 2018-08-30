//by bitluni
//for PWM extension board
//servo 0 is moved back forth (not smooth.. timing is messed up)
//servo 1 is 0°
//servo 2 is 180°
//servo 3 is 90°

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

const int servoPulseMin = 128;
const int servoPulseMax = 640;
const int servoPulseWidth = servoPulseMax - servoPulseMin;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

void setServo(int servo, float angle)
{
  pwm.setPWM(servo, 0, int(servoPulseMin + angle / 180.f * servoPulseWidth)); 
}

void setup() 
{
  pwm.begin();
  pwm.setPWMFreq(50);
  for(int i = 0; i < 16; i++)
    pwm.setPWM(i, 4096, 0);
  setServo(1, 0);
  setServo(2, 180);
  setServo(3, 90);
}

void loop() 
{
  static int pos = 0;
  for (pos = 0; pos <= 180; pos += 1) 
  {
    setServo(0, pos);
    delay(20);
  }
  for(; pos >= 0; pos -= 1) 
  {
    setServo(0, pos);
    delay(20);
  }
}
