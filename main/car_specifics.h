/* Contains the forward declarations of the following:
 * Car class
*/
#ifndef CAR_SPECIFICS_HEADER
#define CAR_SPECIFICS_HEADER

#include "transducer_classes.h"
#include "Arduino.h"

/**********************************
CAR CLASS
***********************************/
class Car
{
  Motor leftMotors;
  Motor rightMotors;

  const int turnSpeed{ 150 };

  public:
    bool dirOverriden{ 0 };
    int speed;
    int batVolt;
    float dir;

    const int minSpeed{ 50 };

    // Constructor
    Car(Motor l, Motor r);

    // Initializes car and performs system checks
    void begin();

    // Updates the motor to move at the specified speed
    // Overloaded to also change direction
    bool drive(int s);
    bool drive(float d);
    bool drive(int s, float d);

    // Overrides direction control
    void overrideDir();

    // turns by the specified amount
    void turn(float d);

    // PID-esque loop to keep car pointing same direction
    bool keepDir(float currentDir);
};

#endif
