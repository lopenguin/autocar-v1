/* Contains the forward declarations of the following:
 * Motor class
 * readGY function (GY-521)
 * GYbegin function (GY-521)
 * readBatVolt function (voltage divider)
 * writeBuf function
 * getDir function (ICM-20948)
 * senBegin functions
 * senRead functions
*/
#ifndef TRANSDUCER_CLASSES_HEADER
#define TRANSDUCER_CLASSES_HEADER

#include "Arduino.h"

// Transducers being used:
#define USE_MOTOR_CLASS 1
#define USE_GY_521 0
#define USE_BAT_VOLT 1
#define USE_BUFFER_FNS 1
#define USE_ICM_209 1
#define USE_DISTANCE_SENSE 1
#define USE_DATA_LOGGER 1
#define USE_SERVOS 1
#define USE_SERVO_TIME 1

// Specific libraries
#if USE_ICM_209
#include "ICM_20948.h"
#endif

#if USE_DISTANCE_SENSE
#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include "SparkFun_VL53L1X.h"
#endif

#if USE_SERVOS
#include <Servo.h>
#endif

/**********************************
MOTOR CLASS
***********************************/
#if USE_MOTOR_CLASS

class Motor
{
  int enPin;
  int in1Pin;
  int in2Pin;

  bool inReverse;
  bool shouldAccel;

  public:
    int currentSpeed;

    // Constructor
    Motor(int e, int in1, int in2);

    // Copy Constructor
    Motor(const Motor &m);

    // Default Constructor
    Motor();

    // Initializes motor with Arduino
    void begin();

    // Writes a specific speed to the motor
    void drive(int s);

  private:
    // Changes the direction (forward or reverse)
    void setDir(bool dir);
};

#endif

/**********************************
GY-521 Functions
***********************************/
#if USE_GY_521

// Sets up GY
void GYbegin();

// Reads GY data
void readGY(int16_t accel[], bool printIMU);

#endif

/**********************************
Battery Voltage
***********************************/
#if USE_BAT_VOLT

// Reads battery voltage
float readBatVolt(int p);

#endif

/**********************************
Buffer Functions
***********************************/
#if USE_BUFFER_FNS

// Writes to a buffer
void writeBuf(int16_t buf[], int16_t newVal, int bufSize);
void writeBuf(int buf[], int newVal, int bufSize);

#endif

/**********************************
ICM Functions
***********************************/
#if USE_ICM_209

// Sets up the ICM
void senBegin(ICM_20948_I2C &myI);

// Gets current magnetometer direction
float getDir(ICM_20948_I2C &myI);

// Reads ICM data
bool senRead(ICM_20948_I2C &myI);

#endif

/**********************************
Distance Sensor Functions
***********************************/
#if USE_DISTANCE_SENSE

// Sets up the distance sensor
void senBegin(SFEVL53L1X &dist);

#endif

/**********************************
Servo Functions
***********************************/
#if USE_SERVOS

class MyServo : public Servo
{
  short pin;
  int currentPos;

  bool maxReached;
  int stepSize = 100;
  bool servoOn = 0;

  public:
    #if USE_SERVO_TIME
    unsigned long lastWriteTime = 0;
    int delayPeriod = 0;
    #endif
    int minWrite;
    int maxWrite;
    int middle;
    // Constructor
    MyServo(short p, int minVal, int maxVal);
    // Allows setting of middle
    MyServo(short p, int minVal, int maxVal, int m);

    // Uses the pin defintion and Servo.attach(pin) fn
    void attach();

    void detach();

    // Calls Servo.writeMicroseconds()
    // returns boolean that describes if movement occured
    bool writeMicroseconds(int w);

    // sets the step size (optional)
    void setStep(int s);

    // Steps by the set step amount
    // Defaults to 100
    void step();

    // returns the angle (using middle as 0) that the servo is pointing.
    // Angle units are radians / pi
    // optionally can input a writeMicroseconds to get angle out
    float getAngle(int a);
    float getAngle();

    // writes an angular direction (converts it to writeMicroseconds)
    void writeDir(float d);
};

#endif

#endif
