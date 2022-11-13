#include <Wire.h>
#include <MPU6050.h>
#include <KalmanFilter.h>
#include <Servo.h>
#include <avr/io.h>

MPU6050 mpu;

KalmanFilter kalmanX(0.001, 0.003, 0.003);
KalmanFilter kalmanY(0.001, 0.003, 0.03);
KalmanFilter kalmanVX(0.001, 0.003, 0.03);
KalmanFilter kalmanVY(0.001, 0.003, 0.03);

float accPitch = 0;
float accRoll = 0;

float kalPitch = 0;
float kalRoll = 0;

const int SERVO1_PIN = 9;   //x
const int SERVO2_PIN = 10;  //y

Servo servo1;
Servo servo2;
String inData;
float servo1_angle = 87;
float servo2_angle = 87;
float v0;
float v1;
float a0;
float a1;
float t0;
float a00;
float a10;

String valores;

void setup()
{
  Serial.begin(115200);
  // Initialize MPU6050
  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    delay(500);
  }
  
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();

  servo1.attach(SERVO1_PIN, 500, 2500);
  servo2.attach(SERVO2_PIN, 500, 2500);

  servo1.writeMicroseconds((servo1_angle * 1500) / 90); //X  78    7,955(138.603) e 7.974(41.396)  / 5.274(120.059) e 5.692(59.94) / 6.247(126.922) e 6.534(53.077) // 7.555(42.54) e 7.022(131.46)
  servo2.writeMicroseconds((servo2_angle * 1500) / 90);
  t0 = millis();
  a0 = 0;
  a1 = 0;
  a00 = 0;
  a10 = 0;
}

void loop()
{
  Vector acc = mpu.readNormalizeAccel();
  Vector gyr = mpu.readNormalizeGyro();

  // Calculate Pitch & Roll from accelerometer (deg)
  accPitch = -(atan2(acc.XAxis, sqrt(acc.YAxis * acc.YAxis + acc.ZAxis * acc.ZAxis)) * 180.0) / M_PI;
  accRoll  = (atan2(acc.YAxis, acc.ZAxis) * 180.0) / M_PI;

  // Kalman filter
  kalPitch = kalmanY.update(accPitch, gyr.YAxis);
  kalRoll = kalmanX.update(accRoll, gyr.XAxis);

  v0 = (kalRoll - a0) / ((millis() - t0) / 1000);
  v1 = (-kalPitch - a1) / ((millis() - t0) / 1000);
  v0 = kalmanVX.update( gyr.XAxis,v0);
  v1 = kalmanVY.update( gyr.YAxis,v1);

  t0 = millis();
  a0 = kalRoll;
  a1 = -kalPitch;
  a00=accRoll;
  a10=-accPitch;
  
  //Serial.flush();
  Serial.print((kalRoll -0.4), 2);
  Serial.print(F(","));
  Serial.print((-kalPitch - 1.74), 2);
  Serial.print(F(","));
  Serial.print(0, 1);
  Serial.print(F(","));
  Serial.print((0), 1);
  Serial.println(F(""));



}

void serialEvent() {
  if (Serial.available() > 0) {


    inData = Serial.readStringUntil(':');
    servo1_angle = inData.toInt();

    inData = Serial.readStringUntil('$');
    servo2_angle = inData.toInt();


    servo1.write(servo1_angle);
    servo2.write(servo2_angle);


  }
}
