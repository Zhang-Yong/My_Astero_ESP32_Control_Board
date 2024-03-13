#include<Arduino.h>
#include<motor.h>
float left_motor_angle, left_motor_speed, right_motor_angle,right_motor_speed;

// left use "-" to keep same direction for 2 motors
void set_motor_speed(float left_speed,float right_speed)
{
  left_speed = -left_speed;
  Serial1.print("T");
  Serial1.println(-left_speed);

  Serial2.print("T");
  Serial2.println(right_speed);
}


void read_left_motor()
{
  // Serial1.println("A");
  // while(Serial1.available() == 0){}
  // left_motor_angle = -Serial1.parseFloat();
  // Serial.print("left");
  // Serial.print(left_motor_angle);
  // Serial.print("    ");

  Serial1.println("S");
  while(Serial1.available() == 0){}
  //add "-" on left motor speed for pid_vec cal, make it positive
  left_motor_speed = -Serial1.parseFloat();
  // Serial.print("left:");
  // Serial.println(left_motor_speed);

    // problem code below, show no data,
  // Serial1.println("S");
  // if (Serial1.available() > 0) {
  //   // read the incoming byte:
  // left_motor_speed = -Serial1.parseFloat();
  // Serial.print("left:");
  // Serial.println(left_motor_speed);
  // }
  // else {
  //   Serial.println("left motor no data");
  // }

  
}


void read_right_motor()
{
  // Serial2.println("A");
  // while(Serial2.available() == 0){}
  // right_motor_angle = Serial2.parseFloat();
  // Serial.print("right");
  // Serial.print(right_motor_angle);
  // Serial.print("    ");

  Serial2.println("S");
  while(Serial2.available() == 0){}
  right_motor_speed = Serial2.parseFloat();
  // Serial.print("Right:");
  // Serial.println(right_motor_speed);

  // Serial2.println("S");
  // //while(Serial1.available() == 0){}
  // if (Serial2.available() > 0) {
  //   // read the incoming byte:
  // right_motor_speed = -Serial2.parseFloat();
  // Serial.print("right:");
  // Serial.println(right_motor_speed);
  //}

}

