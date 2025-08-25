#include <XBOXUSB.h>

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

float M1 = 0, M2 = 0, M3 = 0;
float theta, M, setpoint = 0, pre_error = 0, proportional, integral = 0, derivative, error, a_M, b_M, c_M;
float kp = 0, ki = 0, kd = 0, pid = 0;
float result = 0;
float motor_A = 0, motor_B = 0, motor_C = 0;

int solenoid_base = 24;
bool solenoid_state = LOW;
int Bx_ref1 = 0;

int yaw_1, W, U;
int previousValue, currentValue;
float previousTheta;

//  A Button speed toggle
bool A_ref = false;
bool joystick_speed_state = 0;  // 0 = low (20), 1 = high (80)
float joystick_speed_multiplier = 0.25;  // initial: 20%

void setup() {
  Serial.begin(115200);
#if !defined(_MIPSEL_)
  while (!Serial);
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1);
  }
  Serial.print(F("\r\nXBOX USB Library Started"));

  SWSerial1.begin(9600);
  SWSerial2.begin(9600);

  pinMode(44, OUTPUT);
  pinMode(45, OUTPUT);
  pinMode(46, OUTPUT);
  pinMode(47, OUTPUT);
  pinMode(48, OUTPUT);
  pinMode(49, OUTPUT);
  pinMode(29, OUTPUT);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(solenoid_base, OUTPUT);

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

    int rawLX = Xbox.getAnalogHat(LeftHatX);
    int rawLY = Xbox.getAnalogHat(LeftHatY);
    int rawRX = Xbox.getAnalogHat(RightHatX);
    int rawRY = Xbox.getAnalogHat(RightHatY);

    int LX = map(rawLX, -32768, 32767, -127, 127);
    int LY = map(rawLY, -32768, 32767, -127, 127);
    int RX = map(rawRX, -32768, 32767, -40, 40);
    int RY = map(rawRY, -32768, 32767, -40, 40);

    if (LX > -2 && LX < 2) LX = 0;
    if (LY > -2 && LY < 2) LY = 0;
    if (RX > -2 && RX < 2) RX = 0;
    if (RY > -2 && RY < 2) RY = 0;

    if (RY >= 2) RY = RY - 2;
    if (RY <= -2) RY = RY + 2;
    if (RX >= 2) RX = RX - 2;
    if (RX <= -2) RX = RX + 2;

    if (LY >= 2) LY = LY - 2;
    if (LY <= -2) LY = LY + 2;
    if (LX >= 2) LX = LX - 2;
    if (LX <= -2) LX = LX + 2;

    W = LX + RX;
    U = LY + RY;

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

    //  Toggle speed on A button
    if (a == 0 && !A_ref) {
      joystick_speed_state = !joystick_speed_state;
      joystick_speed_multiplier = joystick_speed_state ? 1.0 : 0.40;
      A_ref = true;
    }
    
    if (a == 1 && A_ref) {
      A_ref = false;
    }

    if (rt > 0) setpoint -= 0.25;
    if (lt > 0) setpoint += 0.25;

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

    kp = 11;
    kd = 80;
    ki = 0;

    result = sqrt((W * W) + (U * U));
    result = result * joystick_speed_multiplier;

    theta = atan2(U, W) * 180 / 3.142;
    theta = theta - (yaw * 1.73);

    c_M = result * cos(theta * (3.14 / 180));
    b_M = result * sin((theta - 30) * (3.14 / 180));
    a_M = result * sin((theta + 30) * (3.14 / 180));

    error = setpoint - yaw;
    proportional = error;
    integral += error;
    derivative = error - pre_error;
    pre_error = error;
    pid = kp * proportional + ki * integral + kd * derivative;

    motor_A = a_M - pid;
    motor_B = b_M + pid;
    motor_C = c_M + pid;

    motor_A = constrain(motor_A, -115, 115);
    motor_B = constrain(motor_B, -115, 115);
    motor_C = constrain(motor_C, -115, 115);

    M1 = map(motor_A, -115, 115, -80, 80);
    M2 = map(motor_B, -115, 115, -80, 80);
    M3 = map(motor_C, -115, 115, -80, 80);

    ST1.motor(1, M1);
    ST1.motor(2, -M2);
    //ST2.motor(1, M3);
    ST1.motor(1, -M3);

    Serial.print("yaw_1: ");
    Serial.print(yaw_1);
    Serial.print(" setpoint: ");
    Serial.print(setpoint);
    Serial.print("  LX: ");
    Serial.print(LX);
    Serial.print("  LY: ");
    Serial.print(LY);
    Serial.print("  RX: ");
    Serial.print(RX);
    Serial.print("  RY: ");
    Serial.print(RY);
    Serial.print("  M1: ");
    Serial.print(M1);
    Serial.print("  M2: ");
    Serial.print(M2);
    Serial.print("  M3: ");
    Serial.println(M3);
  } else {
    digitalWrite(29, LOW);
    ST1.motor(1, 0);
    ST1.motor(2, 0);
    ST2.motor(1, 0);
    ST2.motor(2, 0);
  }
  delay(1);
}