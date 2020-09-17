#include "transducer_classes.h"
#include "Arduino.h"
#include "Wire.h" // For I2C communication

#define serialOut Serial

#if USE_DISTANCE_SENSE
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include "SparkFun_VL53L1X.h"
#endif

/**********************************
MOTOR CLASS
***********************************/
#if USE_MOTOR_CLASS

/* Constructor
 * e: enable pin
 * in1: first input pin (high = reverse)
 * in2: second input pin (high = forward)
*/
Motor::Motor(int e, int in1, int in2)
{
  enPin = e;
  in1Pin = in1;
  in2Pin = in2;

  inReverse = 0;
  shouldAccel = 0;
  currentSpeed = 0;
}

Motor::Motor(const Motor &m)
{
  enPin = m.enPin;
  in1Pin = m.in1Pin;
  in2Pin = m.in2Pin;

  inReverse = 0;
  shouldAccel = 0;
  currentSpeed = 0;
}

// Connects all the pins and initializes the motor with Arduino
void Motor::begin()
{
  pinMode(enPin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);

  setDir(1);

  analogWrite(enPin,0);

  serialOut.println(F("Motor Connected"));
}

/* Turns on the motor and sets direction
 * s: speed input (-255, 255)
*/
void Motor::drive(int s)
{
  // Check if the speed is too big
  if (s > 255)
  {
    serialOut.println(F("Tried to go too fast!"));
    return;
  }

  // Check if it is already set at this speed
  if (currentSpeed == s)
    return;

  currentSpeed = s;

  // Reverse direction accordingly based on sign of speed input
  if (s >= 0)
  {
    setDir(1);
  }
  else
  {
    setDir(0);
    s = 0 - s;
  }

  // Set the speed!
  analogWrite(enPin,s);
  shouldAccel = 1;
  // serialOut.println(s);
}

/* Changes the direction (private)
 * dir: new direction (1 = forward)
*/
void Motor::setDir(bool dir)
{
  if (dir)
  {
    digitalWrite(in1Pin, 0);
    digitalWrite(in2Pin, 1);
  }
  else
  {
    digitalWrite(in1Pin, 1);
    digitalWrite(in2Pin, 0);
  }
}

#endif

/**********************************
GY-521 Functions
***********************************/
#if USE_GY_521

// Runs IMU
void GYbegin()
{
  const int IMU_ADDR{ 0x68 }; // I2C address of IMU
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register for IMU
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

/* Reads the IMU data
 * accel[]: an array that stores XYZ accel data
 * printIMU: decides if data should be printed
*/
void readGY(int16_t accel[], bool printIMU)
{
  const int IMU_ADDR{ 0x68 };     // I2C address of IMU
  const int gravAccelZ{ 15763 };  // for calibration
  const int gravAccelX{ 16832 };
  const int gravAccelY{ 16246 };

  Wire.beginTransmission(IMU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(IMU_ADDR, 7*2, true); // request a total of 7*2=14 registers

  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  //int16_t accel[];
  for (int i{ 0 }; i < 3; i++)
  {
    accel[i] = Wire.read()<<8 | Wire.read();
  }
  int16_t temp = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  int16_t gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  int16_t gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  int16_t gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

  if (printIMU)
  {
    // print out data
    serialOut.print(F("aX: ")); serialOut.print(accel[0] / (float) gravAccelX); // note: xAccel is forward
    serialOut.print(F(" | aY: ")); serialOut.print(accel[1] / (float) gravAccelY);
    serialOut.print(F(" | aZ: ")); serialOut.print(accel[2] / (float) gravAccelZ);
    // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
    serialOut.print(F(" | temp: ")); serialOut.print(temp/340.00+36.53);
    serialOut.print(F(" | gX: ")); serialOut.print(gyro_x);
    serialOut.print(F(" | gY: ")); serialOut.print(gyro_y);
    serialOut.print(F(" | gZ: ")); serialOut.print(gyro_z);
    serialOut.println();
  }
}

#endif

/**********************************
Battery Voltage
***********************************/
#if USE_BAT_VOLT

float readBatVolt(int p)
{
  float v = analogRead(p);
  // 3.3 V is Vout at 1024. *3 corrects for volt. divider
  // For some reason 3.5 is the base voltage (no connection). No idea why.
  v = v * (3.3 / 1024.0) * 3 - 3.5;
  serialOut.print(F("Battery Voltage: ")); serialOut.println(v);
  return v;
}

#endif

/**********************************
Buffer Functions
***********************************/
#if USE_BUFFER_FNS

/* Deletes the last element of the array and places the new value at index 0
 * buf[]: the buffer variable
 * newVal: the new variable to be put in the buffer
 * bufSize: the number of elements in the buffer (size of the array)
*/
void writeBuf(int16_t buf[], int16_t newVal, int bufSize)
{
  for (unsigned int i = bufSize - 1; i > 0; i--)
  {
    buf[i+1] = buf[i];
  }
  buf[0] = newVal;
}
void writeBuf(int buf[], int newVal, int bufSize)
{
  for (unsigned int i = bufSize - 1; i > 0; i--)
  {
    buf[i+1] = buf[i];
  }
  buf[0] = newVal;
}

#endif

/**********************************
ICM Functions
***********************************/
#if USE_ICM_209

/* Sets up ICM
 * Input: the ICM object
*/
void senBegin(ICM_20948_I2C &myI)
{
  bool initialized = false;
  // Make sure initialization occurs
  while(!initialized)
  {
    // Change the 1 to a 0 if the ADR jumper is closed.
    myI.begin(Wire, 1);
    serialOut.print(F("IMU initialization returned: "));
    serialOut.println(myI.statusString());
    if (myI.status != ICM_20948_Stat_Ok)
    {
      serialOut.println(F("Trying again..."));
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }
}
/* Gets the current direction (1 float) based on calibration data.
Calibrate with ICM_calibrate function! It is big cool.
Sample calibration data and const values (magX,magY):
 * N: -51.75, 39.3 -- scaled to 0
 * E: -25.5, 16.8 -- scaled to 0.5
 * S: -46.95, -6.45 -- scaled to -1/1
 * W: -73.35, 16.05 -- scaled to -0.5
 * refX0 = -49.35
 * refX1dif = 23.93
 * refY0 = 16.43
 * refY1dif = 22.87
We actually use the scaled angle (degrees)
Note: there is a discontinuity at S between -1 and 1.
*/
float getDir(ICM_20948_I2C &myI)
{
  // Normalize values based on unit circle
  const float refX0{ -49.35 };     // Zero for x
  const float refX1dif{ 23.93 };  // Difference between 0 and 1 for x
  const float refY0{ 16.43 };    // Zero for y
  const float refY1dif{ 22.87 };  // Dif between 0 and 1 for y

  float dirX = (myI.magX() - refX0) / refX1dif;
  float dirY = (myI.magY() - refY0) / refY1dif;

  // Angle in radians
  // Divide by pi to get multiple, add 1 to make it between 0 and 2.
  return atan2(dirX, dirY) / 3.1415;
}

// Reads ICM data (very simple, but mostly here for modularity)
// Returns success/failure
bool senRead(ICM_20948_I2C &myI)
{
  if (myI.dataReady())
  {
    myI.getAGMT();
    return 1;
  }
  return 0;
}

/* To access ICM data (scaled based on startup conditions)
myI.accX()
myI.accY()
myI.accZ()
myI.gyrZ()
myI.gyrY()
myI.gryZ()
myI.magX()
myI.magY()
myI.magZ()
myI.temp()
*/

#endif

/**********************************
Distance Sensor Functions
***********************************/
#if USE_DISTANCE_SENSE

/* Overloaded distance setup
 * Input: distance sensor object
*/
void senBegin(SFEVL53L1X &dist)
{
  if (dist.begin() != 0)  // Good init returns 0
  {
    serialOut.println(F("Distance sensor failed to begin. Freezing..."));
    while(1)
      ;
  }

  // We only really need short range distance
  dist.setTimingBudgetInMs(50);
  dist.setIntermeasurementPeriod(50);

  serialOut.println(F("Distance sensor online."));
}

#endif

/**********************************
Servo Functions
***********************************/
#if USE_SERVOS

  // Constructor: adds pin, writevals to
  MyServo::MyServo(short p, int minVal, int maxVal)
  {
    pin = p;
    minWrite = minVal;
    maxWrite = maxVal;
    middle = minWrite + (minWrite + maxWrite) / 2;
    currentPos = 0;
    maxReached = 0;
    Servo();
  }

  MyServo::MyServo(short p, int minVal, int maxVal, int m)
  {
    pin = p;
    minWrite = minVal;
    maxWrite = maxVal;
    middle = minWrite + (minWrite + maxWrite) / 2;
    currentPos = 0;
    maxReached = 0;
    middle = m;
  }

  void MyServo::attach()
  {
    Servo::attach(pin);
    maxReached = 0;
    servoOn = 1;
  }

  void MyServo::detach()
  {
    Servo::detach();
    servoOn = 0;
  }

  bool MyServo::writeMicroseconds(int w)
  {
    if (!servoOn)
    {
      serialOut.println("Tried to write to servo but servo is off.");
      return 0;
    }
    Servo::writeMicroseconds(w);
    if (currentPos == w)
      return 0;
    else
    {
      #if USE_SERVO_TIME
      lastWriteTime = millis();
      const float scalingFac = 0.25;
      delayPeriod = static_cast<int>(scalingFac * abs(currentPos - w));
      #endif
      currentPos = w;
      return 1;
    }
  }

  void MyServo::setStep(int s)
  {
    stepSize = s;
    maxReached = 0;
  }

  // steps by the amount set in setStep() each time it is called.
  // When max/min is reached, starts moving in other direction
  // also returns 1 when max/min reached
  void MyServo::step()
  {
    int newPos = currentPos;
    if (maxReached)
      newPos -= stepSize;
    else
      newPos += stepSize;
    // Check if we've passed one of the bounds
    if (newPos <= minWrite)
    {
      newPos = minWrite;
      maxReached = 0;
    }
    if (newPos >= maxWrite)
    {
      newPos = maxWrite;
      maxReached = 1;
    }
    writeMicroseconds(newPos);
  }

  // Gets the angle in radians / pi that the thing is facing
  // 0 is middle, minDir is -0.5, maxDir is 0.5
  // Assumes distance between minWrite and middle is same as between maxDir and middle
  // TODO: add angle limits.
  float MyServo::getAngle(int a)
  {
    float angle = a - middle;
    angle /= static_cast<float>(middle - minWrite)*2;
    return angle;
  }
  float MyServo::getAngle()
  {
    return getAngle(currentPos);
  }

  void MyServo::writeDir(float d)
  {
    if (abs(d) > 0.5)
      return;
    writeMicroseconds(static_cast<int>(d * 2 * (middle - minWrite) + middle));
  }

#endif
