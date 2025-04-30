#include <Wire.h>
#include <MPU6050.h>
#include <AccelStepper.h>

MPU6050 mpu;
AccelStepper stepper1(AccelStepper::DRIVER, 2, 3);
AccelStepper stepper2(AccelStepper::DRIVER, 4, 5);

float Kp = 15.0;
float Ki = 0.0;
float Kd = 0.5;

float setPoint = 0.0;
float input = 0.0;
float output = 0.0;
float error = 0.0, previous_error = 0.0, integral = 0.0;
unsigned long lastTime = 0;
float deltaTime = 0.01;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.initialize();
  stepper1.setMaxSpeed(300);
  stepper2.setMaxSpeed(300);
  stepper1.setAcceleration(200);
  stepper2.setAcceleration(200);
}

void loop() {
  unsigned long now = millis();
  deltaTime = (now - lastTime) / 1000.0;
  if (deltaTime <= 0) deltaTime = 0.01;
  lastTime = now;

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float angleAcc = atan2(ay, az) * 180 / PI;
  static float angle = angleAcc;
  float gyroRate = gx / 131.0;
  angle = 0.98 * (angle + gyroRate * deltaTime) + 0.02 * angleAcc;
  input = angle;

  error = setPoint - input;
  integral += error * deltaTime;
  float derivative = (error - previous_error) / deltaTime;
  output = Kp * error + Ki * integral + Kd * derivative;
  previous_error = error;
  output = constrain(output, -300, 300);

  stepper1.setSpeed(output);
  stepper2.setSpeed(-output);
  stepper1.runSpeed();
  stepper2.runSpeed();

  Serial.print(angle);
  Serial.print(" ");
  Serial.println(output);
}
