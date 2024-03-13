// Run this Program in ESP Lionlite  to control BLDC driver through UART1. BLDC Drvier ESP32 should uploaded with full control firmware
//  Setup:
//  1) Connect ESP Lionlite UART (TX1) to BLDC UART1(RX) 2) ESP Lionlite connect to PC 3) Send motor cmd eg" T 10" to serial port 4)BLDC motor should move

#include <Arduino.h>
#include <Wire.h>
// #include <servo.h>
// ESP32Servo Lib
// #include <Servo.h>
#include <motor.h>
#include <main.h>
// #include <blink_ble.h>
#include <SimpleFOC.h>
#include <MPU6050_tockn.h>

// define MPU6050
 #define I2C_SDA 23
 #define I2C_SCL 19
 TwoWire I2C_mpu = TwoWire(0);
 MPU6050 mpu6050(I2C_mpu);




// MPU6050 PID setup, p = -0.6, P=(0.1~0.6) with max 6V control vol,
// Tune P, set I,D =0, check P +-.
// Tune D, set P, I =0, check +-, D is degree/s,  increase until motor shake
PIDController pid_stb{.P = -0.6, .I = 0.0, .D = -0.01, .ramp = 100000, .limit = 6};
// velocity pid, full voltage 6v, max 20/s , P*20 = 6, P = 0.3
PIDController pid_vel{.P = 0.6, .I = 0, .D = 0.003, .ramp = 10000, .limit = 6};

LowPassFilter lpf_mpu6050{.Tf = 0.07};
// velocity control filtering 速度控制滤波，滤波时间常数为0.07
LowPassFilter lpf_pitch_cmd{.Tf = 0.07};
LowPassFilter lpf_control_voltage_cmd{.Tf = 0.07};


// Servo Setup
static const int servosPins[2] = {33, 32};

// Servo servos[2];

// void setServos(int degrees) {
//     for(int i = 0; i < 2; ++i) {
//         servos[i].write((degrees + (35 * i)) % 180);
//     }
// }

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  // Setup BLDC Driver UART ports
  //V1 Control Board
  //Serial1.begin(115200, SERIAL_8N1, 0, 2);   // Serial_motor_left, uart1 in schematic，close to corner in layout
  //Serial2.begin(115200, SERIAL_8N1, 15, 13); // Serial_motor_right, uart0 in schematic, RX FIRST THEN TX
  
  //For V2 Control Board, serial1 test working,swap motor control serial port solve  speed diff issue,why??? 
  //PS: type C接口有方向，有一面无法识别！！！
  //电源线接触问题会导致motor serial 没有输出，可以通过扭矩判断，没有扭矩就是没有上电
  // Motor driver set to 9600 to avoid transfer error
  Serial1.begin(9600, SERIAL_8N1, 13, 15);   // Serial_motor_left, uart1 in schematic，close to corner in layout
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // Serial_motor_right, uart0 in schematic, RX FIRST THEN TX

 // Serial.print("Controller Program is running... : ");

  // Servo attach
  //  for(int i = 0; i < 2; ++i) {
  //        if(!servos[i].attach(servosPins[i])) {
  //            Serial.print("Servo ");
  //            Serial.print(i);
  //            Serial.println("attach error");
  //        }
  //    }

  //stop motor when mpu calibration
 // set_motor_speed(0,0);   //  1st is right hand wheel, swap motor uart port solve wheel speed diff problem，why??? 

   I2C_mpu.begin(I2C_SDA,I2C_SCL, 400000UL);
   mpu6050.begin();
   mpu6050.calcGyroOffsets(true);
  // _delay(1000);
}

void loop()
{
  // put your main code here, to run repeatedly:
  // Get MPU6050 data
  

//remove comments here; left motor speed is 12 less than right in 6V mode， why???
 read_left_motor(); // read_left_motor_speed  angel,serial 
 read_right_motor(); // read_right_motor_speed  angel, serial 2 
////////////////////
 
  Serial.print("left: ");
  Serial.print(left_motor_speed);
  Serial.print("\tright: ");
  Serial.println(right_motor_speed);
  //delay(500);

  mpu6050.update();
  // Serial.print("angleX : ");
  // Serial.print(mpu6050.getAngleX());
  // Serial.print("\tangleY : ");
  // Serial.print(mpu6050.getAngleY());
  // Serial.print("\tangleZ : ");
  // Serial.println(mpu6050.getAngleZ());

  //PID to set balance, car detla angle, car angle vel, P*delta_angle + D*motor_vec + i*?
  // float target_angle = 0;
  // float measure_angle = lpf_mpu6050(mpu6050.getAngleZ());
   float measure_angle = mpu6050.getAngleX();
   // half circle per sec
   float target_speed = 0.0;
   float target_angle = lpf_pitch_cmd(pid_vel((left_motor_speed+right_motor_speed)/2 - target_speed));
  // seperate speed control mode
  // float control_voltage = lpf_pitch_cmd(pid_vel((left_motor_speed+right_motor_speed)/2 - target_speed));
   float control_voltage = pid_stb( 1.5 + target_angle - measure_angle );
   
   
  // Serial.print("--> control voltage : ");
  // Serial.println(control_voltage);

  

  // Servo Move
  //  for(int posDegrees = 0; posDegrees <= 180; posDegrees++) {
  //          setServos(posDegrees);
  //          Serial.println(posDegrees);
  //          delay(20);
  //      }

  //     for(int posDegrees = 180; posDegrees >= 0; posDegrees--) {
  //         setServos(posDegrees);
  //         Serial.println(posDegrees);
  //         delay(20);
  //     }

  // //send "T X " cmd to motor through serial port 1/2, actually it's voltage setting
  //  set_motor_speed(-6,-6);
  //  delay(5000);
    
    // set same 6,6 voltage. left speed is -4.5, right is 19 ?? Why?
  // set_motor_speed(control_voltage,control_voltage);   //  1st is right hand wheel, swap motor uart port solve wheel speed diff problem，why??? 
   
  // use cmd below cannot make motor move, while set_motor_speed works, why???
  // use cmd below get left(serial1) speed 5, right(serial2) 18, same direction, cw. Serial1 -> L
  // Debug Process:
  // step 1, run-write motor driver on left motor, same
  // step 2, exchange left/right uart port, same
  // step 3, serial1 write -6, same
  //step 4, exchange  left/right power port,same
  // suspect left power line weak connection, change , same
    //  Serial1.print("T");
    //  Serial1.println(-6.0);
    //  Serial2.print("T");
    //  Serial2.println(8.0);


    // to be optimized to use a better  way waiting for data out instead of fix delay
    //remove comments below to enable progrma
    //  delay(100); 
    //  while (Serial1.available() == 0) {}
     
    //  while (Serial1.available() > 0 ) {
    //  float dummy_Byte = Serial1.read();
    //  }


    //  while (Serial2.available() == 0) {
    // //  Serial.print("Waiting for motor serial output...");
    //   }
     
    //  while (Serial2.available() > 0 ) {
    //  float dummy_Byte2 = Serial2.read();
    //  }
////////////////////////////////////////////


  

  /* just print to pc of what you just send on serial1/2 */
  // if (Serial1.available()) {
  //   int inByte = Serial1.read();
  //   Serial.write(inByte);
  // }
  // if (Serial2.available()) {
  //   int inByte = Serial2.read();
  //   Serial.write(inByte);
  // }

  // Manually set motor  Speed:
  //  Setup:
  //  1) Connect ESP Lionlite UART (TX1) to BLDC UART1(RX) 2) ESP Lionlite connect to PC
  // 2) Send motor cmd eg" T 10" to serial port 4)BLDC motor should move
  // read from PC port 0, send to BLDC Driver UART1, connect
  //  if (Serial.available()) {
  //    int inByte = Serial.read();
  //    Serial.println("Send ok !");
  //    Serial1.write(inByte);
  //   // }
}