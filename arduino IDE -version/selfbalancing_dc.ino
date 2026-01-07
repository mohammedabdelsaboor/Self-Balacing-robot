// first install required libraries in .zip in  the folder Arduino IDE version 
#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 #include "Wire.h"
#endif
#define MIN_ABS_SPEED 50
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

double originalSetpoint = .6;
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;
double Kp = 60;   
double Kd = 2.2;
double Ki = 270;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.7;
double motorSpeedFactorRight = 0.7;

int ENA = 11;
int IN1 = 7;
int IN2 = 6;
int IN3 = 5;
int IN4 = 4;
int ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);
volatile bool mpuInterrupt = false; 
void dmpDataReady()
{
 mpuInterrupt = true;
}

void setup()
{
 #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
 Wire.begin();
 TWBR = 24;
 #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
 Fastwire::setup(400, true);
 #endif
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
 attachInterrupt(0, dmpDataReady, RISING);
 mpuIntStatus = mpu.getIntStatus();
 dmpReady = true;
 packetSize = mpu.dmpGetFIFOPacketSize();
 pid.SetMode(AUTOMATIC);
 pid.SetSampleTime(10);
 pid.SetOutputLimits(-255, 255); 
 }
 else
 {
 Serial.print(F("DMP Initialization failed (code "));
 Serial.print(devStatus);
 Serial.println(F(")"));
 }
}

void loop()
{ 
 if (!dmpReady) return;
 while (!mpuInterrupt && fifoCount < packetSize)
 {
 pid.Compute();
 motorController.move(output, MIN_ABS_SPEED);
 }
 mpuInterrupt = false;
 mpuIntStatus = mpu.getIntStatus();
 fifoCount = mpu.getFIFOCount();

 if ((mpuIntStatus & 0x10) || fifoCount == 1024)
 {
 mpu.resetFIFO();
 Serial.println(F("FIFO overflow!"));
 }
 else if (mpuIntStatus & 0x02)
 {
 while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
 mpu.getFIFOBytes(fifoBuffer, packetSize);
 fifoCount -= packetSize;
 Serial.println(ypr[1] * 180/M_PI);
 mpu.dmpGetQuaternion(&q, fifoBuffer);
 mpu.dmpGetGravity(&gravity, &q);
 mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
 input = ypr[1] * 180 / M_PI;
 setpoint = 0;   
 }
}
