/*
 Example sketch for the Xbox 360 USB library - developed by Kristian Lauszus
 For more information visit my blog: http://blog.tkjelectronics.dk/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */

#include <XBOXUSB.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

USB Usb;
XBOXUSB Xbox(&Usb);

#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

SoftwareSerial SWSerial1(NOT_A_PIN, 7);
SabertoothSimplified ST1(SWSerial1);
SoftwareSerial SWSerial2(NOT_A_PIN, 5);
SabertoothSimplified ST2(SWSerial2);

//IMU
#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu;

unsigned long timer = 0;
float timeStep = 0.01;
float yaw = 0;

//other variables
float M1 = 0;
float M2 = 0;

float M3 = 0;
float M4 = 0;
float theta, M, setpoint = 0, pre_error = 0, proportional, integral = 0, derivative, error, a_M, b_M, c_M;
float kp = 0, ki = 0, kd = 0, pid = 0;
float result = 0;
float motor_A = 0;
float motor_B = 0;
float motor_C = 0;
int limit_switch = 0;
float speed_arm = 0;
int g = 0, hh;
int a_ref = 0;
int counter = 0;
int theta_1;
int yaw_1;
int counter_ref = 0, counter_ref1 = 0, counter_ref2 = 0;
;
int x_ref = 0;
int counter_1 = 0;
int y_ref = 0;
int counter_2 = 0;
bool solenoid_state = LOW;
int solenoid_base = 24;
int Bx_ref1 = 0;
int U, W;
void setup() {
  Serial.begin(115200);
#if !defined(MIPSEL)
  while (!Serial)
    ;  // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1)
      ;  //halt
  }
  Serial.print(F("\r\nXBOX USB Library Started"));

  //Sabertooth
  SWSerial1.begin(9600);
  SWSerial2.begin(9600);

  pinMode(44, OUTPUT);
  pinMode(45, OUTPUT);
  pinMode(46, OUTPUT);
  pinMode(47, OUTPUT);
  pinMode(48, OUTPUT);
  pinMode(49, OUTPUT);
  pinMode(29, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(solenoid_base, OUTPUT);
  //IMU
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(1);
}
void loop() {
  Usb.Task();
  if (Xbox.Xbox360Connected) {
    Vector norm = mpu.readNormalizeGyro();
    yaw = yaw + norm.ZAxis * timeStep;
    yaw_1 = yaw * 1.73;
    digitalWrite(29, HIGH);
    digitalWrite(31, LOW);
    int rawLX = Xbox.getAnalogHat(LeftHatX);
    int rawLY = Xbox.getAnalogHat(LeftHatY);
    int rawRX = Xbox.getAnalogHat(RightHatX);
    int rawRY = Xbox.getAnalogHat(RightHatY);



    // Map values from -32768~32767 to 0~127
    int LX = map(rawLX, -32768, 32767, -127, 127);
    int LY = map(rawLY, -32768, 32767, -127, 127);
    int RX = map(rawRX, -32768, 32767, -40, 40);
    int RY = map(rawRY, -32768, 32767, -40, 40);

    if (LX > -2 && LX < 2) LX = 0;
    if (LY > -2 && LY < 2) LY = 0;
    //if (RX > -22 && RX < 2) RX = 0;
    //if (RY > -23 && RY < 2) RY = 0;

    // if (RY >= 2) RY = RY - 2;
    // if (RY <= -2) RY = RY + 2;
    // if (RX >= 2) RX = RX - 2;
    // if (RX <= -2) RX = RX + 2;

    if (LY >= 2) LY = LY - 2; 
    if (LY <= -2) LY = LY + 2;
    if (LX >= 2) LX = LX - 2; 
    if (LX <= -2) LX = LX + 2;



    U = LX + RX;
    W = LY + RY;

    // Button states
    int a = Xbox.getButtonPress(A);
    int b = Xbox.getButtonPress(B);
    int x = Xbox.getButtonPress(X);
    int y = Xbox.getButtonPress(Y);
    int lb = Xbox.getButtonPress(LB);
    int rb = Xbox.getButtonPress(RB);
    int lt = Xbox.getButtonPress(LT);
    int rt = Xbox.getButtonPress(RT);
    int start = Xbox.getButtonPress(START);
    int back = Xbox.getButtonPress(BACK);
    int up = Xbox.getButtonPress(UP);
    int down = Xbox.getButtonPress(DOWN);
    int left = Xbox.getButtonPress(LEFT);
    int right = Xbox.getButtonPress(RIGHT);

    if (rt > 0) {
      setpoint -= 0.20;
    }
    if (lt > 0) {
      setpoint += 0.20;
    }




    if (b == 1 && Bx_ref1 == 0) {
      solenoid_state = !solenoid_state;
      digitalWrite(solenoid_base, solenoid_state);
      Bx_ref1 = 1;
    }
    if (b == 0 && Bx_ref1 == 1) {
      Bx_ref1 = 0;
    }


    digitalWrite(44, rb);
    digitalWrite(47, lb);
    digitalWrite(46, left);
    digitalWrite(45, right);
    digitalWrite(48, b);
    digitalWrite(49, start);

    kp = 11.5;  //10 //17  //48    //20      //55
    kd = 82;    //74 //70;  //65.5   // 80  //130
    ki = 0;     // 0.001

    result = sqrt((U * U) + (W * W));
    //result = result / 1.5;
    theta = atan2(W, U) * 180 / 3.142;
    theta = theta - (yaw * 1.73);

    
    // bracket checking



    c_M = result * cos(theta * (3.14 / 180));
    b_M = result * sin((theta - 30) * (3.14 / 180));
    a_M = result * sin((theta + 30) * (3.14 / 180));

    // PID control adjustments
    error = setpoint - yaw;
    proportional = error;
    integral += error;
    derivative = error - pre_error;
    pre_error = error;
    pid = kp * proportional + ki * integral + kd * derivative;

    motor_A = a_M - pid;
    motor_B = b_M + pid;
    motor_C = c_M + pid;

    motor_A = constrain(motor_A, -127, 127);
    motor_B = constrain(motor_B, -127, 127);
    motor_C = constrain(motor_C, -127, 127);

    //motor_C=100;

    // M1 = motor_A ;
    // M2 = motor_B;
    // M3 = motor_C;

    M1 = map(motor_A, -127, 127, -90, 90);
    M2 = map(motor_B, -127, 127, -90, 90);
    M3 = map(motor_C, -127, 127, -90, 90);


    ST1.motor(1, M1);
    ST1.motor(2, -M2);
    ST2.motor(1, M3);
    // ST2.motor(2, speed_arm);

    Serial.print("yaw_1: ");
    Serial.print(yaw_1);
    Serial.print(" error: ");
    Serial.print(error);

    Serial.print("  M1: ");
    Serial.print(M1);
    Serial.print("  M2: ");
    Serial.print(M2);
    Serial.print("  M3: ");
    Serial.print(M3);
    Serial.print("  pid: ");
    Serial.print(pid);

    Serial.print("  RX: ");
    Serial.print(RX);
    Serial.print("  RY: ");
    Serial.print(RY);
    Serial.print("  LX: ");
    Serial.print(LX);
    Serial.print("  LY: ");
    Serial.print(LY);

    Serial.println(" ");

  }

  else {
    digitalWrite(31, HIGH);
    ST1.motor(1, 0);
    ST1.motor(2, -0);
    ST2.motor(1, 0);
    ST2.motor(2, 0);
  }
  // delay(1);
}