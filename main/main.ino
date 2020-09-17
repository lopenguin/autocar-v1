/*
A versitle program for any robot that uses motors and sensor inputs.
Designed for any Arudino. Tested on the Sparkfun Redboard Artemis
Written by Lorenzo Shaikewitz

Notes
 * If low on memory, change Serial.println("...") to .println(F("..."))
 * Using floats instead of doubles for everything (should be fine)

More info to come soon
TODO:
 * Store sensor data in separate file?
 * Video?
 * GPS support (not entirely necessary)
 * add "follow me" mode
 * figure out bluetooth
 * voice activated machine learning?
 * Update this header
*/

// NEED TO TEST: Direction control with serial (command 'N');
// NEED TO FIX: AVOID TURNING

#define SENSORS 0
#define DEBUG_CAR

// libraries and custom files
#include "transducer_classes.h"   // For sensor connections
#include "car_specifics.h"        // Main car specifics
#include "Wire.h"                 // For I2C communication
#include <Wire.h>
#include "SparkFun_Qwiic_OpenLog_Arduino_Library.h"
#include <Servo.h>

OpenLog myOpenLog;
#ifdef DEBUG_CAR
#define serialOut Serial
#else
#define serialOut myOpenLog
#endif

// States
enum class States
{
  LOWBAT,
  DRIVE,
  AVOID,
  FOLLOW_WAIT,
  FOLLOW_DRIVE,
  IDLING,
};

// Avoid states
enum class AvoidStates
{
  AVOID_APPRAISE,
  AVOID_TURN_CHECK,
  AVOID_CHECK,
  AVOID_TURN,
  AVOID_DRIVE,
  AVOID_LAST_TURN,
};

// Subsystems in use (0 means no selection)
enum class Subsystems
{
  NONE,
  MOTOR_ID,
  ICM_ID,
  DIST_ID,
  DIRECTION_ID,
  SERVO_ID,
};

// Subsystem on/off helper variables
Subsystems subSysSelection{ Subsystems::NONE };
bool ICMon{ 1 };
bool distOn{ 1 };

// Global variables (sorry Michael)
States currentState{ States::DRIVE };
AvoidStates avoidState { AvoidStates::AVOID_APPRAISE };
bool switched{ 1 };
float oldDir{ 0.0 };
int oldSpeed{ 0 };
bool ICMdataReady{ 0 };
bool distDataReady{0};
bool newDistData{0};
int distance{0};
float direction{0.0};
bool noOldDir{1};
bool avoidCheckingFront{0};
bool avoidingLeft{0};
int runningDist{0};
float minAngle{0.0};
bool leftChecked{0};
bool distServoWritten{0};

// Const global variables (you're welcome Michael)
const int closeDist{ 450 };
const String fileName{ "Car_Data.txt" };
const int distBufSize = 4;

// Time variables (consider adding struct for sensor data + times)
unsigned long lastCheckTime{};
unsigned long lastICMtime{};
unsigned long switchedTime{};
unsigned long lastDistTime{};
unsigned long avoidDriveTime{};
unsigned long lastUpdateTime{};

// Buffer variables
int16_t accelBuf[3]{ 0 };
int distBuf[distBufSize]{ 0, 100, 1000, 50 };

// Pins
const int batPin{ A0 };
const int ctrlPin{ 2 };
const int distServoPin{3};

// Sensor declarations
ICM_20948_I2C myICM;
SFEVL53L1X distSen;
MyServo distServo(distServoPin, 600, 2100, 1350);

// Motor declarations
Motor leftMotor(10,9,8);
Motor rightMotor(7,6,5);
Car autoCar(leftMotor, rightMotor);

// Forward function declarations
void switchStateTo(States newState);

void setup()
{
  Serial.begin(57600);
  Wire.begin();
  senBegin(myICM);
  senBegin(distSen);
  distServo.attach();
  distServo.writeMicroseconds(distServo.middle);
  autoCar.begin();
  #ifndef DEBUG_CAR
  serialOut.begin();
  serialOut.append(fileName);
  #endif
  serialOut.println(F("-------CAR BEGIN-------"));

  pinMode(batPin, INPUT);       // For voltage sensor
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(ctrlPin, INPUT);      // On/off button

  // Wait for button press to start
  while(!digitalRead(ctrlPin))
    ;
  serialOut.println(F("Activated by button press."));

  if (digitalRead(ctrlPin))
    delay(200);
  readBatVolt(batPin);
  lastCheckTime = millis();
  lastICMtime = millis();
  switchedTime = millis();
  lastDistTime = millis();
  avoidDriveTime = millis();
  lastUpdateTime = millis();

  autoCar.drive(150);
}

/***************************
STATE FUNCTION
****************************/
void runState(unsigned long currentTime)
{
  // turn off after first execution
  if (ICMdataReady)
    ICMdataReady = 0;

  if (distDataReady)
    distDataReady = 0;

  // Read ICM data
  if (currentTime - lastICMtime > 50 && ICMon)
  {
    lastICMtime = millis();
    if (senRead(myICM))
    {
      ICMdataReady = 1;
      if (SENSORS)
        serialOut.println(myICM.accY(), 2);
      writeBuf(accelBuf, myICM.accY(), 3);

      // Get the current direction
      direction = getDir(myICM);
    }
  }

  // Read distance data
  if (currentTime - lastDistTime > 125 && distOn)
  {
    lastDistTime = millis();
    // Ranging takes 50 ms on current settings
    distSen.startRanging();
    newDistData = 1;
  }

  if (newDistData && (distSen.checkForDataReady()) && distOn)
  {
    newDistData = 0;
    distDataReady = 1;
    // Get the distance
    distance = distSen.getDistance();
    distSen.clearInterrupt();
    distSen.stopRanging();
    writeBuf(distBuf, distance, distBufSize);
    if (SENSORS)
      serialOut.println(distance);
  }

  // Check if the distservo is ready
  if (distServoWritten)
    distServoWritten = 0;
  if (currentTime - distServo.lastWriteTime > distServo.delayPeriod)
  {
    distServoWritten = 1;
  }

  switch(currentState)
  {
    // Turn off everything in the event of low battery
    case States::LOWBAT:
      if (switched)
        serialOut.println(F("In state: LOWBAT"));
      autoCar.drive(0);
      digitalWrite(LED_BUILTIN,1);
      // Note: there is no escape function (yet)
      break;

    // Looks for acceleration changes and obstacles while currently moving
    case States::DRIVE:
      // Main goal: prevent the large acceleration detection from misfiring
      if (switched)
      {
        switched = 0;
        serialOut.println(F("In state: DRIVE"));
        distServo.writeMicroseconds(distServo.middle);
      }

      // Use mag data to keep direction
      if (ICMdataReady)
      {
        autoCar.keepDir(direction);

        // Check to see if we hit something
        if (currentTime - switchedTime > 1000)
        {
          // Check for large change in acceleration (2 G, from empirical)
          if (abs(accelBuf[0] - accelBuf[1]) > 2000)
          {
            serialOut.println(F("Hit something (acceleration)"));
            switchStateTo(States::IDLING);
          }
        }
      }

      if (distDataReady)
      {
        // See if any objects are getting close (<300 mm)
        // Stop and redirect around obstacle (dist in mm)
        if (distance < closeDist)
        {
          serialOut.print(F("Avoiding an obstacle "));
          serialOut.print(distance);
          serialOut.println(F(" mm away"));
          switchStateTo(States::AVOID);
        }

        // Check to make sure the distance has changed
        // If it hasn't that's a good sign we are stuck!
        if (autoCar.speed != 0)
        {
          int k = 0;
          for (int i = 1; i < distBufSize; i++)
          {
            k += (abs(distBuf[0] - distBuf[i]) < 5) ? 1 : 0;
          }
          if (k == distBufSize)
          {
            serialOut.println(F("We cannot move (distance). Stopping..."));
            switchStateTo(States::IDLING);
          }
        }
      }
      break;

    // Handles obstacle avoidance system
    case States::AVOID:
      // Save the old direction, stop, and turn 90 deg right.
      if (switched)
      {
        serialOut.println(F("In state: AVOID"));
        switched = 0;
        oldSpeed = autoCar.speed;
        oldDir = autoCar.dir;
        autoCar.drive(0);
        autoCar.overrideDir();
        avoidState = AvoidStates::AVOID_APPRAISE;
        distServo.writeMicroseconds(distServo.minWrite);
        runningDist = distance + 100;
        distServo.setStep(100);
        if (autoCar.dirOverriden)
          noOldDir = 1;
        else
          noOldDir = 0;
        break;
      }

      if (!ICMon || !distOn)
      {
        serialOut.println(F("ICM/distance needed to avoid obstacles!"));
        switchStateTo(States::IDLING);
      }

      /* Uses distance sensor output to cautiously change avoid an obstacle
       * Intended to be called in whenever distance sensor data is taken
       * General algorithm:
          1) Use servo to find min distance and direction
          2) turn in place to face min distance
          3) Check right
          4) If dist is good, go. If not, check left
          5) If dist is good, go. If not, turn 90 deg right and repeat from 3
          6) Go for 1 s, turn 90 deg left and check distance
          7) If okay, proceed. If not, repeat from 1
          All physical direction changing is handled by keepDir (inside main)
       */
      switch (avoidState)
      {
        // Find the wall using minimum distance from servo. GOOD
        case AvoidStates::AVOID_APPRAISE:
          // Check distance, then step the servo (starts all the way right)
          if (distDataReady)
          {
            // check if this is the true min distance.
            if (distance < runningDist)
            {
              runningDist = distance;
              minAngle = distServo.getAngle();
            }
            distServo.step();
            // exit protocol: check if we are at maxWrite (will always happen)
            if (distServo.getAngle() > 0.47)
            {
              serialOut.println("Wall found. Turning to face it");
              float newDir = direction - minAngle;
              if (newDir > 1.0)
                newDir -= 2.0;
              if (newDir <= -1.0)
                newDir += 2.0;
              autoCar.drive(0, newDir);
              avoidState = AvoidStates::AVOID_TURN_CHECK;
            }
          }
          break;

        // Carries out the turning (actual turn directive is somewhere else) GOOD
        case AvoidStates::AVOID_TURN_CHECK:
          if (ICMdataReady)
          {
            // Exit and turn protocol
            if (autoCar.keepDir(direction))
            {
              serialOut.print(F("Checking distance... "));
              // move the servo right 90 deg to start checking
              distServo.writeMicroseconds(distServo.minWrite);
              leftChecked = 0;
              avoidState = AvoidStates::AVOID_CHECK;
            }
          }
          break;

        // Check the right distance. If good, move right. Otherwise, check left
        // if left is bad, turn right 90 degrees and try again.
        case AvoidStates::AVOID_CHECK:
          if(distServoWritten)
          {
            if (distDataReady)
            {
              serialOut.println(distance);
              if (distance < closeDist)
              {
                // obstacle is too close
                if (!leftChecked)
                {
                  distServo.writeMicroseconds(distServo.maxWrite);
                  leftChecked = 1;
                  serialOut.println("Checking left");
                }
                else
                {
                  // statements will only execute if already facing left
                  // This means we're stuck; it's time to turn 90 degrees right and try again
                  autoCar.turn(0.5);
                  distServo.writeMicroseconds(distServo.minWrite);
                  serialOut.print(F("Obstacles on both left and right. "));
                  serialOut.println(F("Turning right"));
                  avoidState = AvoidStates::AVOID_TURN_CHECK;
                }
              }
              else
              {
                // The coast is clear! Let's turn towards the good direction!
                serialOut.print("Turning ");
                // Check the direction of the one that succeeded
                if (distServo.getAngle() > 0)
                {
                  serialOut.println("left to avoid obstacle.");
                  autoCar.turn(-0.5);
                  distServo.writeMicroseconds(distServo.minWrite);
                }
                else
                {
                  serialOut.println("right to avoid obstacle.");
                  autoCar.turn(0.5);
                  distServo.writeMicroseconds(distServo.maxWrite);
                }
                avoidState = AvoidStates::AVOID_TURN;
              }
            }
          }
          break;

        case AvoidStates::AVOID_TURN:
          if (ICMdataReady)
          {
            // Exit and turn protocol
            if (autoCar.keepDir(direction))
            {
              // If we are in roam mode, just switch back to drive
              if (noOldDir)
              {
                serialOut.print(F("Obstacle avoided!"));
                autoCar.drive(oldSpeed);
                autoCar.overrideDir();
                switchStateTo(States::DRIVE);
                return;
              }
              serialOut.println(F("Proceeding cautiously "));
              autoCar.drive(autoCar.minSpeed * 3);
              avoidDriveTime = currentTime;
              avoidCheckingFront = 0;
              avoidState = AvoidStates::AVOID_DRIVE;
            }
          }
          break;

        // Drives straight slowly
        /* If entered here: servo is facing the wall, we are moving parallel.
        What we need to do:
        1) IF wall disppears:
          A) IF we are within 90 deg of oldDir:
            * turn servo to face oldDir, if distance is good go! (FREE)
            * else: step B
          B) ELSE: turn towards where wall and
              make servo face direction you turned in (face old wall).
              Proceed slowly (repeat from 1).
        2) Check front every two(?) seconds.
        3) If too close, go to AVOID_APPRAISE */
        case AvoidStates::AVOID_DRIVE:
          if (ICMdataReady)
          {
            autoCar.keepDir(direction);
          }
          // make sure the servos have enough time to move
          if (distServoWritten)
          {
            if (distDataReady)
            {
              if (avoidCheckingFront)
              {
                if (distance < closeDist)
                {
                  // go back to avoid appraise, with initial conditions
                  serialOut.println("Obstacle in front. Appraising the situation.");
                  distServo.writeMicroseconds(distServo.minWrite);
                  autoCar.drive(0);
                  autoCar.overrideDir();
                  runningDist = 0;
                  distServo.setStep(100);
                  avoidState = AvoidStates::AVOID_APPRAISE;
                  return;
                }
                // no obstacle in front!
                serialOut.println("Front is good. Proceeding with avoidance");
                if (avoidingLeft)
                  distServo.writeMicroseconds(distServo.maxWrite);
                else
                  distServo.writeMicroseconds(distServo.minWrite);
                avoidCheckingFront = 0;
              }
              else
              {
                if (distance > closeDist)
                {
                  // Looks like the wall has disappeared
                  serialOut.println("Wall is gone.");
                  // If we're within 90 deg of oldDir, turn the servo to face it and check distance
                  if (abs(direction - oldDir) < 0.5)
                  {
                    // Check if the servo was already facing oldDir
                    if (abs(distServo.getAngle() - oldDir) < 0.1)
                    {
                      // SUCCESS! The obstacle in our old direction is gone
                      // Turn towards oldDir and then start driving.
                      serialOut.println(F("OldDir is good!"));
                      autoCar.drive(0, oldDir);
                      distServo.writeMicroseconds(distServo.middle);
                      avoidState = AvoidStates::AVOID_LAST_TURN;
                      return;
                    }
                    else
                    {
                      distServo.writeDir(oldDir);
                      serialOut.println(F("Checking oldDir..."));
                    }
                  }
                  else
                  {
                    // we're not near the target dir, but the obstacle is gone
                    // just turn 90 degrees towards where the wall was
                    if (distServo.getAngle() > 0)
                    {
                      // turn left
                      serialOut.println("Turning left");
                      autoCar.turn(-0.5);
                      distServo.writeMicroseconds(distServo.maxWrite);
                      autoCar.drive(0);
                      avoidState = AvoidStates::AVOID_TURN;
                      return;
                    }
                    else
                    {
                      // turn right
                      serialOut.println("Turning right");
                      autoCar.turn(0.5);
                      autoCar.drive(0);
                      distServo.writeMicroseconds(distServo.minWrite);
                      avoidState = AvoidStates::AVOID_TURN;
                      return;
                    }
                  }
                }
              }
            }
          }

          // check the front to make sure we don't run into anything
          if (currentTime - avoidDriveTime > 500)
          {
            serialOut.println("Checking out the front");
            avoidCheckingFront = 1;
            avoidDriveTime = currentTime;
            if (distServo.getAngle() > 0)
              avoidingLeft = 1;
            else
              avoidingLeft = 0;
            distServo.writeMicroseconds(distServo.middle);
          }
          break;

        case AvoidStates::AVOID_LAST_TURN:
          if (ICMdataReady)
          {
            if (autoCar.keepDir(direction))
            {
              serialOut.println("Obstacle is gone. Driving in original direction");
              autoCar.drive(oldSpeed, oldDir);
              switchStateTo(States::DRIVE);
            }
          }
          break;
      }

      // // whenever we look forward, make sure there's nothing in front to avoid.
      // if (distDataReady && (abs(distServo.getAngle()) < 0.1))
      // {
      //   if (distance < closeDist)
      //   {
      //     serialOut.println("Object too close. Stopping.");
      //     autoCar.drive(0);
      //   }
      // }

      // Kill. If we take too long, admit we are stuck and give up.
      if (currentTime - switchedTime > (5*60*1000))
      {
        serialOut.println(F("Stuck for too long. I give up!"));
        autoCar.drive(0);
        switchStateTo(States::IDLING);
      }
      break;

    // Wait for something to come close before following it
    case States::FOLLOW_WAIT:
      if (switched)
      {
        serialOut.println(F("In state: FOLLOW_WAIT"));
        switched = 0;
      }

      if (distDataReady)
      {
        // Arbitrary cutoff for follow mode
        if (distance < 600)
        {
          switchStateTo(States::FOLLOW_DRIVE);
        }
      }
      break;

    // Stay a fixed distance away from something close.
    case States::FOLLOW_DRIVE:
      if (switched)
      {
        serialOut.println(F("In state: FOLLOW_DRIVE"));
        switched = 0;
      }

      // Call car class function to actually do this
      if (distDataReady)
      {
        //autoCar.follow(distance);
      }
      break;

    // Does nothing.
    case States::IDLING:
      if (switched)
      {
        digitalWrite(LED_BUILTIN, HIGH);
        switched = 0;
        serialOut.println(F("In state: Idling"));
        autoCar.drive(0);
        autoCar.overrideDir();
      }
      break;
  }

  // Check for commands
  if (Serial.available())
  {
    switch (Serial.read())
    {
      // Stop everything
      case 'x':
        Serial.println(F("Command 'x' recieved. Stopping..."));
        #ifndef DEBUG_CAR
        serialOut.println(F("Command 'x' recieved. Stopping..."));
        #endif
        autoCar.drive(0);
        autoCar.overrideDir();
        switchStateTo(States::IDLING);
        break;

      // Does a systems check (battery volt, motor connection, etc)
      case 'c':
        readBatVolt(batPin);
        #ifndef DEBUG_CAR
        serialOut.println(readBatVolt(batPin));
        #endif
        break;

      // Enables subsystem functions
      case 'e':
        Serial.println(F("Command 'e' recieved."));
        #ifndef DEBUG_CAR
        serialOut.println(F("Command 'e' recieved."));
        #endif
        switch (subSysSelection)
        {
          case Subsystems::NONE:
            Serial.println(F("No subsystem selected"));
            #ifndef DEBUG_CAR
            serialOut.println(F("No subsystem selected"));
            #endif
            break;

          case Subsystems::ICM_ID:
            // turns on ICM bool
            ICMon = 1;
            break;

          case Subsystems::DIST_ID:
            // Turns on distance sensor functionality
            distOn = 1;
            break;

          case Subsystems::SERVO_ID:
            // Attaches servo
            distServo.attach();
            break;

          default:
            Serial.println(F("No 'on' functionality defined for selected subsystem."));
            #ifndef DEBUG_CAR
            serialOut.println(F("No 'on' functionality defined for selected subsystem."));
            #endif
        }
        break;

      // Disables subsystem functionality
      case 'r':
        Serial.println(F("Command 'r' recieved."));
        #ifndef DEBUG_CAR
        serialOut.println(F("Command 'r' recieved."));
        #endif
        switch (subSysSelection)
        {
          case Subsystems::NONE:
            Serial.println(F("No subsystem selected."));
            #ifndef DEBUG_CAR
            serialOut.println(F("No subsystem selected."));
            #endif
            break;

          case Subsystems::MOTOR_ID:
            // Quasi-redundant, just stops the thing (overrides direction too)
            Serial.println(F("Stopping the motor."));
            #ifndef DEBUG_CAR
            serialOut.println(F("Stopping the motor."));
            #endif
            autoCar.drive(0);
            autoCar.overrideDir();
            break;

          case Subsystems::ICM_ID:
            // Turns ICM functions off (but not ICM itself)
            Serial.println(F("Turning off the ICM."));
            #ifndef DEBUG_CAR
            serialOut.println(F("Turning off the ICM."));
            #endif
            ICMon = 0;
            break;

          case Subsystems::DIST_ID:
            // Turns dist sensor functions off (but not distSen itself)
            Serial.println(F("Turning off the distance sensor."));
            #ifndef DEBUG_CAR
            serialOut.println(F("Turning off the distance sensor."));
            #endif
            distOn = 0;
            break;

          case Subsystems::DIRECTION_ID:
            Serial.println(F("Overriding directional control."));
            #ifndef DEBUG_CAR
            serialOut.println(F("Overriding directional control."));
            #endif
            autoCar.overrideDir();
            switchStateTo(States::DRIVE);
            break;

          case Subsystems::SERVO_ID:
            distServo.detach();
            break;

          default:
            Serial.println(F("No 'off' functionality defined for selected subsystem."));
            #ifndef DEBUG_CAR
            serialOut.println(F("No 'off' functionality defined for selected subsystem."));
            #endif
        }
        break;

      // Triggers the number input
      case 'i':
      {
        Serial.println(F("Command 'i' recieved."));
        #ifndef DEBUG_CAR
        serialOut.println(F("Command 'i' recieved."));
        #endif
        // Reads whatever number is stated after 'i'
        int tempInt{ Serial.parseInt() };
        switch (subSysSelection)
        {
          case Subsystems::NONE:
            Serial.println(F("No subsystem selected."));
            #ifndef DEBUG_CAR
            serialOut.println(F("No subsystem selected."));
            #endif
            break;

          case Subsystems::MOTOR_ID:
          {
            int tempSpeed = tempInt;
            Serial.print(F("Set motor speed to ")); Serial.println(tempSpeed);
            #ifndef DEBUG_CAR
            serialOut.println(F("Set motor speed to ")); serialOut.println(tempSpeed);
            #endif

            // Do the driving
            if (autoCar.drive(tempSpeed))
            {
              switchStateTo(States::DRIVE);
            }
            else
            {
              Serial.println(F("Invalid speed entered. Stopping..."));
              #ifndef DEBUG_CAR
              serialOut.println(F("Invalid speed entered. Stopping..."));
              #endif
              autoCar.drive(0);
              switchStateTo(States::IDLING);
            }
            break;
          }

          case Subsystems::DIRECTION_ID:
          {
            float tempDir = (float) tempInt / 180;
            Serial.print(F("Set direction to ")); Serial.println(tempDir);
            #ifndef DEBUG_CAR
            serialOut.println(F("Set direction to ")); serialOut.println(tempDir);
            #endif
            // Do the driving
            if (autoCar.drive(tempDir))
            {
              switchStateTo(States::DRIVE);
            }
            else
            {
              Serial.println(F("Invalid direction entered. Stopping..."));
              #ifndef DEBUG_CAR
              serialOut.println(F("Invalid direction entered. Stopping..."));
              #endif
              autoCar.drive(0);
              switchStateTo(States::IDLING);
            }
            break;
          }

          case Subsystems::SERVO_ID:
            Serial.print(F("Set servo to ")); Serial.println(tempInt);
            #ifndef DEBUG_CAR
            serialOut.print(F("Set servo to ")); serialOut.println(tempInt);
            #endif
            distServo.writeMicroseconds(tempInt);
            break;

          default:
            Serial.println(F("No 'input' functionality defined for selected subsystem."));
            #ifndef DEBUG_CAR
            serialOut.println(F("No 'input' functionality defined for selected subsystem."));
            #endif
        }
        break;
      }

      // Selector bloc
      // Each sensor + motors can be turned off or manipulated using these commands
      // To add one:
      // 1. Add a const int to the top of this program
      // 2. Add a case here
      // 3. Add a case to each command it needs to use
      // These will always be capital letters.
      case '0':
        Serial.println(F("No subsystem selected."));
        #ifndef DEBUG_CAR
        serialOut.println(F("No subsystem selected."));
        #endif
        subSysSelection = Subsystems::NONE;
        break;

      case 'M':
        Serial.println(F("Motors selected."));
        #ifndef DEBUG_CAR
        serialOut.println(F("Motors selected."));
        #endif
        subSysSelection = Subsystems::MOTOR_ID;
        break;

      case 'I':
        Serial.println(F("ICM selected."));
        #ifndef DEBUG_CAR
        serialOut.println(F("ICM selected."));
        #endif
        subSysSelection = Subsystems::ICM_ID;
        break;

      case 'D':
        Serial.println(F("Distance sensor selected."));
        #ifndef DEBUG_CAR
        serialOut.println(F("Distance sensor selected."));
        #endif
        subSysSelection = Subsystems::DIST_ID;
        break;

      case 'N':
        Serial.println(F("Direction control selected."));
        #ifndef DEBUG_CAR
        serialOut.println(F("Direction control selected."));
        #endif
        subSysSelection = Subsystems::DIRECTION_ID;
        break;

      case 'S':
        Serial.println(F("Servo selected."));
        #ifndef DEBUG_CAR
        serialOut.println(F("Servo selected"));
        #endif
        subSysSelection = Subsystems::SERVO_ID;
        break;
      // End of selector bloc

      // Help case: prints out all commands
      case 'h':
        Serial.println(F("Command 'h' recieved."));
        #ifndef DEBUG_CAR
        serialOut.println(F("Command 'h' recieved."));
        #endif
        Serial.println(F("Current commands:"));
        Serial.println(F("x: stop everything"));
        Serial.println(F("c: systems check"));
        Serial.println(F("e: enable selected system"));
        Serial.println(F("r: disable selected system"));
        Serial.println(F("i: input an integer"));
        //Serial.println(F("k: delete all data stored on disk"));
        Serial.println(F("h: help"));
        Serial.println(F("----------Subsystems----------"));
        Serial.println(F("0: no selection"));
        Serial.println(F("M: motors"));
        Serial.println(F("I: ICM/IMU"));
        Serial.println(F("D: distance sensor"));
        Serial.println(F("N: direction control"));
        Serial.println(F("S: servo"));
        break;
    }
  }

  if (digitalRead(ctrlPin))
  {
    serialOut.println(F("Direction override engaged."));
    autoCar.drive(150);
    autoCar.overrideDir();
    delay(200);
  }

  #ifndef DEBUG_CAR
  if (currentTime - lastUpdateTime > 500)
  {
    lastUpdateTime = millis();
    serialOut.syncFile();
  }
  #endif

  if (currentTime - lastCheckTime > (60*1000))
  {
    lastCheckTime = millis();
    serialOut.print(currentTime);
    serialOut.print(F(" ms: "));
    serialOut.print(readBatVolt(batPin));
    serialOut.println(F(" V"));
  }
}

void loop()
{
  runState(millis());
}

/****************************
Switch state helper function
****************************/
void switchStateTo(States newState)
{
  if (currentState != newState)
  {
    digitalWrite(LED_BUILTIN, LOW);
    currentState = newState;
    switched = 1;
    switchedTime = millis();
  }
}
