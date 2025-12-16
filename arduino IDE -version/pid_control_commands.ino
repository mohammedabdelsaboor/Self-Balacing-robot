#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#include "Wire.h"

#define MIN_ABS_SPEED 50

// ================= MPU6050 =================
MPU6050 mpu;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

// ================= PID =================
double input, output;
double setpoint = 0;

double Kp = 200;
double Ki = 0.05;
double Kd = 170;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// ================= MOTOR FACTORS =================
double motorSpeedFactorLeft  = 1.0;
double motorSpeedFactorRight = 0.35;

// ================= L298 PINS =================
int ENA = 11;
int IN1 = 6;
int IN2 = 7;

int ENB = 10;
int IN3 = 4;
int IN4 = 5;

LMotorController motorController(
  ENA, IN1, IN2,
  ENB, IN3, IN4,
  motorSpeedFactorLeft,
  motorSpeedFactorRight
);

// ================= KEYBOARD =================
char cmd = 'x';
double moveOffset = 0.0;   // forward / backward tilt
int turnOffset = 0;        // steering PWM offset

// ================= INTERRUPT =================
volatile bool mpuInterrupt = false;
void dmpDataReady()
{
  mpuInterrupt = true;
}

// ================= SETUP =================
void setup()
{
  Wire.begin();
  Serial.begin(9600);

  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setZAccelOffset(0);

  if (devStatus == 0)
  {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();

    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
  }
}

// ================= LOOP =================
void loop()
{
  if (!dmpReady) return;

  // ===== READ KEYBOARD =====
  if (Serial.available())
  {
    cmd = Serial.read();

    if (cmd == 'w') moveOffset = 3.0;
    else if (cmd == 's') moveOffset = -3.0;
    else if (cmd == 'x') moveOffset = 0.0;

    if (cmd == 'a') turnOffset = -40;
    else if (cmd == 'd') turnOffset = 40;
    else if (cmd == 'x') turnOffset = 0;
  }

  if (!mpuInterrupt) return;
  mpuInterrupt = false;

  fifoCount = mpu.getFIFOCount();
  if (fifoCount < packetSize) return;

  mpu.getFIFOBytes(fifoBuffer, packetSize);
  fifoCount -= packetSize;

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  input = ypr[1] * 180 / M_PI;

  // ===== APPLY MOVE OFFSET =====
  setpoint = moveOffset;

  pid.Compute();

  // ===== APPLY TURN OFFSET =====
  int leftPWM  = output + turnOffset;
  int rightPWM = output - turnOffset;

  motorController.move(leftPWM, rightPWM, MIN_ABS_SPEED);

  Serial.println(input);
}
