#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"




// LQR version >>> state-feedback control is implemented here using an ESP32 (240 MHz CPU)
// ================= MOTOR PINS =================
#define ENA 25
#define IN1 26
#define IN2 27
#define ENB 14
#define IN3 12
#define IN4 13

// ================= ENCODER PINS =================
#define L_ENC_A 34
#define L_ENC_B 35
#define R_ENC_A 32
#define R_ENC_B 33

// ================= PWM =================
#define PWM_FREQ 20000
#define PWM_RES  8
#define CH_L 0
#define CH_R 1

// ================= ENCODER PARAMS =================
#define ENCODER_PPR 11
#define GEAR_RATIO  30
#define TICKS_PER_REV (ENCODER_PPR * 4 * GEAR_RATIO)

// ================= LQR GAINS (HIGH CoM – STABLE) =================
float K_theta    = 200.0;   // angle
float K_thetaDot = 0.4;    // angular rate damping
float K_xDot     = 2.2;    // wheel speed damping

// ================= SETPOINT =================
float theta_ref = 0.6;     // VERY SMALL (deg)

// ================= VARIABLES =================
volatile long leftTicks  = 0;
volatile long rightTicks = 0;

long lastLeft  = 0;
long lastRight = 0;

float wheelVel = 0;
float wheelVelFilt = 0;

unsigned long lastTimeMicros = 0;

// ================= IMU =================
MPU6050 mpu;
bool dmpReady = false;
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

volatile bool mpuInterrupt = false;
void IRAM_ATTR dmpDataReady() {
  mpuInterrupt = true;
}

// ================= ENCODER ISR =================
void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(L_ENC_B)) leftTicks++;
  else leftTicks--;
}

void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(R_ENC_B)) rightTicks++;
  else rightTicks--;
}

// ================= MOTOR CONTROL =================
void drive(int pwm) {
  pwm = constrain(pwm, -255, 255);

  // ---- minimal deadzone (avoid bang-bang) ----
  int minPWM = 22;
  if (pwm > 0 && pwm < minPWM) pwm = minPWM;
  if (pwm < 0 && pwm > -minPWM) pwm = -minPWM;

  digitalWrite(IN1, pwm > 0);
  digitalWrite(IN2, pwm < 0);
  digitalWrite(IN3, pwm > 0);
  digitalWrite(IN4, pwm < 0);

  ledcWrite(CH_L, abs(pwm));
  ledcWrite(CH_R, abs(pwm));
}

void stopMotors() {
  ledcWrite(CH_L, 0);
  ledcWrite(CH_R, 0);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcSetup(CH_L, PWM_FREQ, PWM_RES);
  ledcSetup(CH_R, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, CH_L);
  ledcAttachPin(ENB, CH_R);

  pinMode(L_ENC_A, INPUT_PULLUP);
  pinMode(L_ENC_B, INPUT_PULLUP);
  pinMode(R_ENC_A, INPUT_PULLUP);
  pinMode(R_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(L_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(R_ENC_A), rightEncoderISR, RISING);

  // ===== IMU =====
  mpu.initialize();
  if (mpu.dmpInitialize() == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(23), dmpDataReady, RISING);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
    Serial.println(" DMP READY");
  } else {
    Serial.println(" DMP INIT FAILED");
    while (1);
  }

  lastTimeMicros = micros();
}

// ================= LOOP =================
void loop() {
  if (!dmpReady || !mpuInterrupt) return;
  mpuInterrupt = false;

  if (mpu.getFIFOCount() < packetSize) return;
  mpu.getFIFOBytes(fifoBuffer, packetSize);

  // ===== IMU =====
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  VectorInt16 gyro;
  mpu.dmpGetGyro(&gyro, fifoBuffer);

  float thetaRaw = ypr[1] * 180.0 / M_PI;
  float theta    = thetaRaw - theta_ref;
  float thetaDot = gyro.x / 131.0;

  // ===== TIME =====
  unsigned long now = micros();
  float dt = (now - lastTimeMicros) * 1e-6;
  if (dt <= 0 || dt > 0.015) dt = 0.01;
  lastTimeMicros = now;

  // ===== ENCODER SPEED =====
  long l, r;
  noInterrupts();
  l = leftTicks;
  r = rightTicks;
  interrupts();

  long dL = l - lastLeft;
  long dR = r - lastRight;
  lastLeft = l;
  lastRight = r;

  wheelVel = ((dL + dR) / 2.0) * 60.0 / (TICKS_PER_REV * dt);

  // ---- strong damping filter ----
  wheelVelFilt = 0.7 * wheelVelFilt + 0.3 * wheelVel;
  wheelVel = wheelVelFilt;

  // ===== CLEAN LQR (NO KICKS) =====
  float u =
    - K_theta    * theta
    - K_thetaDot * thetaDot
    - K_xDot     * wheelVel;

  // ===== SAFETY =====
  if (abs(thetaRaw) > 35) {
    stopMotors();
    return;
  }

  drive(u);

  // ===== DEBUG =====
  Serial.print("θ: "); Serial.print(thetaRaw);
  Serial.print(" | θ̇: "); Serial.print(thetaDot);
  Serial.print(" | RPM: "); Serial.print(wheelVel);
  Serial.print(" | U: "); Serial.println(u);
}
