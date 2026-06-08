#include <Servo.h>
int R_SPEED = 300;//-500 to 500 (0=STOP)
int L_SPEED = 300;//-500 to 500 (0=STOP)
Servo L_SERVO;
Servo R_SERVO;

void setup()
{
  L_SERVO.attach(14,1000,2000);//PORT(A0),MIN,MAX
  R_SERVO.attach(15,1000,2000);//PORT(A1),MIN,MAX
}

void loop()
{
  L_SERVO.writeMicroseconds(1500 + L_SPEED);
  R_SERVO.writeMicroseconds(1500 - R_SPEED);
  while(1);
}