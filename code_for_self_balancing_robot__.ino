#include <Wire.h>

#define MPU 0x68

// ======================================================
// MOTOR PINS
// ======================================================

#define IN1 27
#define IN2 26
#define IN3 25
#define IN4 33

// ======================================================
// MPU VARIABLES
// ======================================================

float AccAngle = 0;
float GyroRate = 0;
float Angle = 0;

// ======================================================
// FILTERED GYRO
// ======================================================

float filteredGyro = 0;

// ======================================================
// GYRO OFFSET
// ======================================================

float GyroErrorX = 0;

// ======================================================
// CONTROL VALUES
// ======================================================

float Kp = 55.0;
float Kd = 2.5;

// ======================================================
// OUTPUT
// ======================================================

float output = 0;

// ======================================================
// TIMING
// ======================================================

unsigned long previousTime = 0;
float dt = 0;

void setup() {

  Serial.begin(115200);

  // ======================================================
  // MOTOR SETUP
  // ======================================================

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  analogWrite(IN1, 0);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 0);

  // ======================================================
  // MPU6050 SETUP
  // ======================================================

  Wire.begin(21, 22);

  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  delay(1000);

  // ======================================================
  // GYRO CALIBRATION
  // KEEP ROBOT STILL
  // ======================================================

  Serial.println("KEEP ROBOT STILL");

  for (int i = 0; i < 800; i++) {

    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);

    Wire.requestFrom(MPU, 2, true);

    int16_t GyX = Wire.read() << 8 | Wire.read();

    GyroErrorX += (GyX / 131.0);

    delay(2);
  }

  GyroErrorX /= 800;

  Serial.print("GYRO OFFSET: ");
  Serial.println(GyroErrorX);

  previousTime = micros();

  Serial.println("BALANCING STARTED");
}

void loop() {

  // ======================================================
  // TIME
  // ======================================================

  unsigned long currentTime = micros();

  dt = (currentTime - previousTime) / 1000000.0;

  previousTime = currentTime;

  if (dt <= 0) {
    dt = 0.001;
  }

  // ======================================================
  // MPU READ
  // ======================================================

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU, 14, true);

  int16_t AcX = Wire.read() << 8 | Wire.read();
  int16_t AcY = Wire.read() << 8 | Wire.read();
  int16_t AcZ = Wire.read() << 8 | Wire.read();

  // Skip temperature bytes
  Wire.read();
  Wire.read();

  int16_t GyX = Wire.read() << 8 | Wire.read();

  // ======================================================
  // ANGLE CALCULATION
  // ======================================================

  AccAngle = atan2(AcX, AcZ) * 180 / PI;

  GyroRate = (GyX / 131.0) - GyroErrorX;

  // ======================================================
  // FASTER GYRO RESPONSE
  // ======================================================

  filteredGyro =
      0.4 * filteredGyro
    + 0.6 * GyroRate;

  // ======================================================
  // COMPLEMENTARY FILTER
  // TRUST GYRO MORE FOR FASTER PREDICTION
  // ======================================================

  Angle =
      0.992 * (Angle + filteredGyro * dt)
    + 0.008 * AccAngle;

  // ======================================================
  // PREDICTIVE CONTROL
  // ======================================================

  output =
      (Kp * Angle)
    + (Kd * filteredGyro);

  output = constrain(output, -255, 255);

  // ======================================================
  // NO RANDOM JITTER
  // ======================================================

  if (abs(Angle) < 0.25 &&
      abs(filteredGyro) < 0.8) {

    output = 0;
  }

  // ======================================================
  // FAST INITIAL KICK
  // FOR HEAVY ROBOT INERTIA
  // ======================================================

  if (abs(Angle) > 1.2 &&
      abs(Angle) < 10) {

    if (output > 0)
      output += 85;

    if (output < 0)
      output -= 85;
  }

  // ======================================================
  // SPEED
  // ======================================================

  int speed = abs(output);

  speed = constrain(speed, 0, 255);

  // ======================================================
  // SAFETY STOP
  // ======================================================

  if (abs(Angle) > 45) {

    analogWrite(IN1, 0);
    analogWrite(IN2, 0);

    analogWrite(IN3, 0);
    analogWrite(IN4, 0);

    return;
  }

  // ======================================================
  // DEBUG
  // ======================================================

  Serial.print("Angle: ");
  Serial.print(Angle);

  Serial.print(" | Gyro: ");
  Serial.print(filteredGyro);

  Serial.print(" | Output: ");
  Serial.println(output);

  // ======================================================
  // MOTOR CONTROL
  // FALL FORWARD -> MOVE FORWARD
  // ======================================================

  if (output > 0) {

    // FORWARD

    analogWrite(IN1, 0);
    analogWrite(IN2, speed);

    analogWrite(IN3, 0);
    analogWrite(IN4, speed);

  }
  else {

    // BACKWARD

    analogWrite(IN1, speed);
    analogWrite(IN2, 0);

    analogWrite(IN3, speed);
    analogWrite(IN4, 0);
  }

  // ======================================================
  // LOOP SPEED
  // ======================================================

  delayMicroseconds(1000);
}