#include "car_specifics.h"
#include "Arduino.h"
#include "transducer_classes.h"

#define serialOut Serial

/**********************************
CAR CLASS
***********************************/
/* Constructor
 * l: two motors on the left
 * r: two motors on the right
 * also intializes the motors and the sensors used
*/
Car::Car(Motor l, Motor r) : leftMotors(l), rightMotors(r)
{
  speed = 0;
  dir = 1.0; // defaults to S because I lazy.
}

//Sets up motors and sensors
void Car::begin()
{
  leftMotors.begin();
  rightMotors.begin();
  serialOut.println(F("--------Car is on.--------"));
}

/* Moves the vehicle and checks for collisions
 * Overloaded for direction setting and both direction and speed
*/
// This just sets speed and keeps current direction
bool Car::drive(int s)
{
  // Make sure speed is valid (-255,255)
  if (abs(s) > 255 || (abs(s) < minSpeed && s != 0))
  {
    serialOut.print(F("Please enter a speed between "));
    serialOut.print(minSpeed);
    serialOut.println(F(" and 255 in magnitude"));
    return 0;
  }

  // Check if this is the same speed
  if (s == speed)
    return 1;
  leftMotors.drive(s);
  rightMotors.drive(s);
  speed = s;
  return 1;
}

// This keeps current speed and sets new direction (keep dir will need to be called to actually change direction)
bool Car::drive(float d)
{
  dirOverriden = 0;
  // Make sure direction is okay (-1,1)
  if (abs(d) > 1)
  {
    serialOut.println(d);
    serialOut.println(F("Please enter a direction between -1 and 1"));
    return 0;
  }
  if (d == -1)
    d = 1;
  dir = d;
  return 1;
}

// This changes speed and direction (just for ease of programming)
bool Car::drive(int s, float d)
{
  return drive(s) && drive(d);
}

// This introduces override direction
void Car::overrideDir()
{
  serialOut.println(F("Direction control overridden"));
  dirOverriden = 1;
}

/* Changes the direction by a specified amount
 * Turn left:   turn(-0.5);
 * Turn right:  turn(0.5);
 * Turn around: turn(1);
 */
void Car::turn(float d)
{
  // dir should be between -1 and 1, so range of dir + d is [-1.5,1.5]
  float newDir{dir + d};
  if (newDir <= -1)
    newDir += 2;
  if (newDir > 1)
    newDir -= 2;
  //drive(fmod(newDir, 2) - 1);
  drive(newDir);
}

/* Proportional control of direction by changing speed.
 * Intended to be called every time accel data is taken
 * Two modes: turn and adjust
    * TURN: speed == 0, just turn passively towards direction
    * ADJUST: speed != 0, proportionally control speed to change direction
  Returns 1 if close to target, 0 if not.
 */
bool Car::keepDir(float currentDir)
{
  // Check if dir is the override direction. This means we don't use directional control.
  if (dirOverriden)
    return 1;

  // Change currentDir so that dir = 0 (put dir at the center).
  currentDir = currentDir - dir;
  // if this results in an invalid currentDir (not between -1 and 1),
  // -/+ 2 to fix it
  if (currentDir > 1)
    currentDir = currentDir - 2;
  if (currentDir <= -1)
    currentDir = currentDir + 2;
  // THIS CREATES A BANG-BANG in the opposite direction. Need to fix.

  // Now, we will use 0 as the desired direction
  // CurrentDir is the distance from this desired direction.

  // MODE: TURN
  // If we are not moving, try to rotate in place
  // Currently does this at full speed (255)
  if (speed == 0)
  {
    // if we are sufficiently close, just stop.
    if (abs(currentDir) < 0.01)
    {
      leftMotors.drive(0);
      rightMotors.drive(0);
      return 1;
    }

    // Normally we get a bang-bang at the extremes (-1/1)
    // This checks if we are close to an extreme and if we are already moving
    // If both these conditions are true, it does not change the motion
    if (abs(currentDir) > 0.9
        && (leftMotors.currentSpeed != 0 || rightMotors.currentSpeed != 0))
      return 0;

    // We are to the left of our target: turn right.
    if (currentDir < 0)
    {
      leftMotors.drive(turnSpeed);
      rightMotors.drive(-turnSpeed);
      return 0;
    }

    // We are to the right of our target: turn left.
    if (currentDir > 0)
    {
      leftMotors.drive(-turnSpeed);
      rightMotors.drive(turnSpeed);
      return 0;
    }
    return 0;
  }

  // MODE: ADJUST
  // If the code gets here, speed != 0.

  // Account for bang-bang at extremes (-1/1)
  // This checks if we are at an extreme and if control is happening
  // (if we are currently turning).
  // If so, there is no need to alter that control.
  if (abs(currentDir) > 0.9
      && (leftMotors.currentSpeed != rightMotors.currentSpeed))
      return 0;

  // If we are facing the wrong direction, just stop and turn
  if (abs(currentDir) > 0.5)
  {
    // We are to the right of our target: turn left.
    if (currentDir > 0)
    {
      leftMotors.drive(-turnSpeed);
      rightMotors.drive(turnSpeed);
      return 0;
    }

    // We are to the left of our target: turn right.
    if (currentDir < 0)
    {
      leftMotors.drive(turnSpeed);
      rightMotors.drive(-turnSpeed);
      return 0;
    }
  }

  // Calculate the proportional term
  // Input is between -0.5 and 0.5
  // Output should be between minSpeed and speed
  //float newSpeed{ max(speed*(1-abs(currentDir)), minSpeed) };
  float newSpeed = max(abs(int(speed*(1 - pow(abs(currentDir) / 3, (float) 1/3)))), abs(minSpeed));
  newSpeed = speed / abs(speed) * newSpeed; // Just to establish sign

  // We are a little to the right of our target
  if (currentDir > 0)
  {
    // We need to reduce power to the the left motor
    leftMotors.drive(newSpeed);
    rightMotors.drive(speed);
  }

  // We are little to the left
  if (currentDir < 0)
  {
    // Reduce power to right motor
    leftMotors.drive(speed);
    rightMotors.drive(newSpeed);
  }

  // Check if we are close to target
  if (abs(currentDir) < 0.01)
    return 1;
  else
    return 0;
}
