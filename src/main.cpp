#pragma region BoringVexConfigStuff

#include "main.h"

#include <math.h>    //neccessary for functions like abs() and round()
#include <stdlib.h>  //neccessary for std::[commands]

#include <cstring>
#include <sstream>  //neccessary for... logic
#include <string>   //neccessary for... using strings :sob:

#include "robot-config.h"  //importing the motors and whatnot

using namespace std;

#pragma endregion


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region NotesMaybeReadMe

// here to document weird quirks of V5, VSCode, or this particular program

/*

Sometimes things just break, like the abs() function was demanding 0 args.

Restarting the program fixed this Strings do not work in vex without external library shenanigans

This is my code, and thus it is my god given right to use it as a diary. ignore the strange comments
*/
#pragma endregion


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region GlobalVars

///// Control Variables //////

bool twoStickMode = true;  // toggles single or float stick drive
const int deadband = 5;    // if the controller sticks are depressed less than deadband%, input will be ignored (to combat controller drift)

const float autonDriveMult = 1.0;
// unused variable to increase / decrease speed of autonomous driving. just make a good drivetrain lol you'll be fine

const float Pi = 3.14159265358;
const float e = 2.71828182845;

int mStartPosL;
int mStartPosR;

int globalTimer = 0;
const int timerTickRate = 50;  // the number of 'ticks' in one second
const int tickDeltaTime = 1000 / timerTickRate;
const int minPrintingDelay = 3;  // std::ceil(50 / tickDeltaTime);

const float degPerCM = (360 * 2) / (4.1875 * Pi * 2.54);  // # of degrees per centimeter = 360 / (2Pir" * 2.54cm/")

int maxFlywheelSpeed = 39;  // flywheel speed as a percent
int flystickArmPos = 0;     // flystick starts at kickstand position

int lastUpTimestamp = 0;
int lastDownTimestamp = 0;
int lastSpinTimestamp = 0;

float defaultArmPos;

int maxflystickArmPos = 5;

int tabVar = 1;

#pragma endregion


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region HelperFunctions //unit conversions and whatnot

const char *toChar(std::string string) { return string.c_str(); }


int toInt(float val) { return val; }

int timeSincePoint(int checkedTime) {
  return checkedTime < globalTimer ? (globalTimer - checkedTime) : -1;  // returns -1 if checkedtime is in the future
}

static bool IsWithinRange(float num, float lowerBound, float upperBound) { return num >= lowerBound && num <= upperBound; }

// i have no idea what im doing
float AccelSmoothingFunc(float stickVal, int xInput) {
  // takes a stick input and acceleration percent and returns a corresponding value from a smooth curve

  // variables which control the shape/range of the funct
  float curveExtremity = 0.19948;  // sigma
  float peakPos = 1;               // mu
  float minAmount = 0.235;         // kappa

  float x = xInput / 100;  // converting the input from percentage to a decimal btween 0-1

  const float multiplier =
      (0.5 / (curveExtremity * sqrt(2 * Pi))) * powf(e, (-0.5 * pow(((minAmount * x - minAmount * peakPos) / curveExtremity), 2)));
  return xInput >= (100) ? (multiplier * stickVal) : stickVal;
}  // function graphed here: [https://www.desmos.com/calculator/y0fwlh6j47]


void PrintToController(std::string prefix, float data, int row, int page) {
  if (globalTimer % 11 == 0) {  // refresh the screen every 11 ticks because 11 is a good number :)
    MainControl.clear();
  }

  if (tabVar == page && (globalTimer % 9 == (row * 3))) {
    MainControl.print(row, 0, prefix.c_str(), toInt(data));
  }
}

void lcdControl() {
  if (MainControl.get_digital_new_press(DIGITAL_LEFT)) {
    tabVar--;
  }
  if (MainControl.get_digital_new_press(DIGITAL_RIGHT)) {
    tabVar++;
  }
}

#pragma endregion


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region GPSAtHome

// this is both my magnum opus and the worst thing I've ever done i hate everything so so much

#pragma region relativeTracking

const float gAccel = 9.806;

std::array<double, 3> prevVelocity;

std::array<double, 3> calculateKinematics(bool isPrinting, bool getVelocity) {  // tracks displacement / acceleration / velocity relative to the robot

  const float deltaTime = tickDeltaTime / 1000;

  pros::c::imu_accel_s_t InertialAccelReading = Inertial.get_accel();  // checking the inertial is costly, so we do it once and capture the result
  std::array<double, 3> currAcceleration = {InertialAccelReading.x * gAccel, InertialAccelReading.y * gAccel, InertialAccelReading.z * gAccel};

  std::array<double, 3> currVelocity = {0.0, 0.0, 0.0};
  std::array<double, 3> distTravelled = {0.0, 0.0, 0.0};

  for (int i = 0; i < 3; i++) {  // tracks velocity / distance travelled over the current tick in all 3 axis

    currVelocity[i] = (currAcceleration[i] * deltaTime) + prevVelocity[i];
    distTravelled[i] = (prevVelocity[i] * deltaTime) + (0.5 * currAcceleration[i] * powf(deltaTime, 2));

    prevVelocity[i] = currVelocity[i];  // caches the velocity of the current tick for use in the next tick
  }

  if (isPrinting) {
    std::string accel = "Accel: ";
    std::string prevVel = "PrevVel: ";
    std::string currVel = "CurrVel: ";
    std::string Displ = "Displ: ";

    for (int i = 0; i < 3; ++i) {
      accel += std::to_string(static_cast<int>(currAcceleration[i] * 100) / 100);
      prevVel += std::to_string(static_cast<int>(prevVelocity[i] * 100) / 100);
      currVel += std::to_string(static_cast<int>(currVelocity[i] * 100) / 100);
      Displ += std::to_string(static_cast<int>(Displ[i] * 100) / 100);


      if (i < 2) {
        accel += ", ";
        prevVel += ", ";
        currVel += ", ";
        Displ += ", ";
      }
    }  // blow up robot

    PrintToController("Timer: %d", globalTimer, 0, 2);
    PrintToController(currVel, 0, 1, 2);
    PrintToController(Displ, 0, 2, 2);

    PrintToController(accel, 0, 0, 3);
    PrintToController(prevVel, 0, 1, 3);
  }

  return getVelocity ? currAcceleration : distTravelled;
}

#pragma endregion



#pragma region globalTracking

std::array<double, 3> globalCoordinates;
std::array<double, 3> globalVelocities;
std::array<double, 3> totalDist;  // temporary, curious to see what just tracking displacement does

void updateGlobalPosition(bool isPrinting) {
  // capturing values of displacement and velocity over the last tick
  std::array<double, 3> currDisplacements = calculateKinematics(isPrinting, false);
  std::array<double, 3> currVelocities = calculateKinematics(isPrinting, true);


  for (int i = 0; i < 3; i++) {  // tracks displacement across all axis, slow and prolly doesn't work
    totalDist[i] += calculateKinematics(true, false).at(i);
  }

  // identify component of displacement change that should be added to each coordinate
  const float thetaHeading = (Inertial.get_heading() > 180) ? (Inertial.get_heading() - 360) : Inertial.get_heading();
  const float thetaPitch = (Inertial.get_pitch() > 180) ? (Inertial.get_pitch() - 360) : Inertial.get_pitch();
  const float thetaYaw = (Inertial.get_yaw() > 180) ? (Inertial.get_yaw() - 360) : Inertial.get_yaw();

  const float cosThetaHeading = cosf(thetaHeading);
  const float sinThetaHeading = sinf(thetaHeading);

  const float cosThetaPitch = cosf(thetaPitch);
  const float sinThetaPitch = sinf(thetaPitch);

  const float cosThetaYaw = cosf(thetaYaw);
  const float sinThetaYaw = sinf(thetaYaw);

  // decomposing the displacement vectors calculated from the inertial, then reconstructing them into the change in coordinates
  // this math REALLY fucking sucks, but I'm not sure theres a better / more efficient way to do this than hardcoding.
  globalCoordinates[0] += currDisplacements.at(0) * cosThetaHeading * cosThetaPitch  // x component of forward displacement
                          + currDisplacements.at(1) * sinThetaHeading * cosThetaYaw  // x component of sideways displacement
                          + currDisplacements.at(2) * sinThetaPitch * sinThetaYaw;   // x component of vertical displacement

  globalCoordinates[1] += currDisplacements.at(0) * sinThetaHeading * cosThetaPitch  // y component of forward displacement
                          + currDisplacements.at(1) * cosThetaHeading * cosThetaYaw  // y component of sideways displacement
                          + currDisplacements.at(2) * sinThetaPitch * sinThetaYaw;   // y component of vertical displacement

  globalCoordinates[2] += currDisplacements.at(0) * sinThetaPitch                   // z component of forward displacement
                          + currDisplacements.at(1) * sinThetaYaw                   // z component of sideways displacement
                          + currDisplacements.at(2) * cosThetaPitch * cosThetaYaw;  // z component of vertical displacement

  if (isPrinting) {
    std::string coordinates = "Coords: ";

    for (int i = 0; i < 3; ++i) {
      coordinates += std::to_string(globalCoordinates.at(i));

      if (i < 2) {
        coordinates += ", ";
      }
    }

    PrintToController(coordinates, 0, 0, 2);  // print coordinates


    PrintToController("Fwd displ.: %d", currDisplacements.at(0), 0, 3);  // print displacement
    PrintToController("Side displ.: %d", currDisplacements.at(1), 1, 3);
    PrintToController("Vert. displ.: %d", currDisplacements.at(2), 2, 3);

    PrintToController("Heading: %d", thetaHeading, 0, 4);  // print angles
    PrintToController("Pitch: %d", thetaPitch, 1, 4);
    PrintToController("Yaw: %d", thetaYaw, 2, 4);
  }
}

#pragma endregion

#pragma endregion


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region PID //the code behind the autonomous Proportional Integral Derivative controller

#pragma region PIDVariables // holds all variables required for the PID controller

// many of these are unneccesarily global / nonstatic, but I find the somewhat negligible innefficiencies
// to be worth the ease of understanding / workability, especially for those newer to robotics and programming

// control variables

bool drivePIDIsEnabled = false;
bool autonPIDIsEnabled = true;

int desiredDist;  // temporary
int desiredHeading;

// tuning coefficients

float lP = 0.9;
float lD = 1.4;
float lI = 0.0;

float lOutput = 1.0;

float rP = 1.2;
float rD = 0.5;
float rI = 0.0;

float rOutput = 2.7;

int integralBoundL = 10 * degPerCM;
int integralBoundR = 5;

// Storage variables for Lateral (forward/back) PID

float errorProportionalL;  // reported value - desired value = position
float previousErrorL = 0;  // position from last loop
float errorDerivativeL;    //(error - prevError)
float errorIntegralL;

float lateralPower = 0;

// Storage variables for Rotational (turning) PID

float errorProportionalR;  // reported value - desired value = position
float previousErrorR = 0;  // position from last loop
float errorDerivativeR;    //(error - prevError)
float errorIntegralR;

float rotationalPower = 0;

// the second set of "rotational" storage vars above are technically useless, as the code executes in such a way that only one set of vars is needed
// to produce both outputs. however, readability is nice. change if memory ever becomes an issue

#pragma endregion  // end of PID variable declaration

// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //

int stuckTimeStamp = 0;
int avgMotorPosition = 0;

bool AutonPID() {
  if (autonPIDIsEnabled) {
    // sets heading from -180 < h < 180, meaning we turn the correct direction from error
    float heading = (Inertial.get_heading() > 180) ? (Inertial.get_heading() - 360) : Inertial.get_heading();

    ///////////////////////////////////////
    //////        Lateral PID        //////
    ///////////////////////////////////////

    avgMotorPosition = ((RDriveFrontM.get_position()) + (LDriveFrontM.get_position())) / 2;

    errorProportionalL = lP * (desiredDist - avgMotorPosition);     // proportional error
    errorDerivativeL = lD * (errorProportionalL - previousErrorL);  // derivative of error

    // filters out the integral at short ranges (no I if |error| < constant lower limit, eg. 10cm),
    // allowing it to be useful when needed without overpowering other elements
    if (fabs(errorProportionalL) > integralBoundL) {
      errorIntegralL += lI * (errorProportionalL);
    } else {
      errorIntegralL = 0;
    }

    lateralPower = (errorProportionalL + errorDerivativeL + errorIntegralL) * lOutput;

    ///////////////////////////////////////
    //////      Rotational PID       //////
    ///////////////////////////////////////

    errorProportionalR = rP * (desiredHeading - heading);           // proportional error
    errorDerivativeR = rD * (errorProportionalR - previousErrorR);  // derivative of error

    // filters out the integral at short ranges (no I if |error| < constant lower limit, eg. 10cm),
    // allowing it to be useful when needed without overpowering other elements
    if (fabs(errorProportionalR) > integralBoundR) {
      errorIntegralR += rI * (errorProportionalR);
    } else {
      errorIntegralR = 0;
    }

    rotationalPower = (errorProportionalR + errorDerivativeR + errorIntegralR) * rOutput;

    ///////////////////////////////////////
    //////        ending math        //////
    ///////////////////////////////////////

    LDrive.move_velocity(autonDriveMult * (lateralPower + rotationalPower));
    RDrive.move_velocity(autonDriveMult * (lateralPower - rotationalPower));

    previousErrorL = errorProportionalL;
    previousErrorR = errorProportionalR;
  }

  return (fabs(errorProportionalL) <= (3 * degPerCM) && fabs(errorProportionalR) <= 1.5) ? true : false;  // return true if done movement
}

// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //
#pragma region debugFunctions

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////                                                                                                                 ////
//                                                 PID TUNING INSTRUCTIONS:                                            //
//   1. call the tunePID function in opcontrol()                                                                       //
//   2. confirm that the P/I/D tuning variables in the "PIDVariables" region are set to 0.0, with the outputs at 1.0   //
//   3. follow the control layout found here: [http://tinyurl.com/3zrb6zj5]                                                                   //
//   4. increase the lP/rP coefficient(s) until the desired motion is completed with oscilations                       //
//   5. increase the lD/rD coefficient(s) until the oscilations dampen out over time                                   //
//   6. increase the lI/rI coefficient(s) until the motion is completed aggressively without oscilations               //
//      (somewhat optional, as not all applications benefit from an integral controller)                               //
//   7. increase the output coefficient(s) until the motion is completed with acceptable speed and precision           //
//                                                                                                                     //
//      as builds and use cases vary, you may need to fiddle with the values after initial tuning after more testing.   //
//      generally the P & D components should be larger than the I, and values should be between 0.0 and 5.0.          //
////                                                                                                                 ////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float adjustFactor = 0.1;  // the increment by which PID variables change during manual tuning
bool isTuningTurns = false;

void tunePID() {  // turns or oscilates repeatedly to test and tune the PID, allowing real-time tuning and adjustments

  lP = 0.0;
  lD = 0.0;
  lI = 0.0;

  lOutput = 1.0;

  rP = 0.0;
  rD = 0.0;
  rI = 0.0;

  rOutput = 1.0;

  desiredDist = 0 * degPerCM;
  desiredHeading = 0;

  while (true) {
    lcdControl();

    if (MainControl.get_digital_new_press(DIGITAL_B)) {  // changes proportional coefficient
      rP += isTuningTurns ? adjustFactor : 0;
      lP += isTuningTurns ? 0 : adjustFactor;
    }
    if (MainControl.get_digital_new_press(DIGITAL_A)) {  // changes integral coefficient
      rI += isTuningTurns ? adjustFactor : 0;
      lI += isTuningTurns ? 0 : adjustFactor;
    }
    if (MainControl.get_digital_new_press(DIGITAL_Y)) {  // changes derivative coefficient
      rD += isTuningTurns ? adjustFactor : 0;
      lD += isTuningTurns ? 0 : adjustFactor;
    }


    if (MainControl.get_digital_new_press(DIGITAL_X)) {  // toggles increases/decreases to tuning variables
      adjustFactor *= -1;
    }
    if (MainControl.get_digital_new_press(DIGITAL_UP)) {  // toggles between testing rotational / lateral drive
      isTuningTurns = !isTuningTurns;
    }
    if (MainControl.get_digital_new_press(DIGITAL_R1)) {  // changes output power of lateral PID
      lOutput += adjustFactor;
    }
    if (MainControl.get_digital_new_press(DIGITAL_L1)) {  // changes output power of rotational PID
      rOutput += adjustFactor;
    }


    PrintToController("PVar: %d", (isTuningTurns ? rP : lP) * 10, 0, 2);
    PrintToController("IVar: %d", (isTuningTurns ? rI : lI) * 10, 1, 2);
    PrintToController("DVar: %d", (isTuningTurns ? rD : lD) * 10, 2, 2);

    PrintToController("Turning?: %d", isTuningTurns, 0, 3);
    PrintToController("lOutput: %d", lOutput, 1, 3);
    PrintToController("rOutput: %d", rOutput, 2, 3);

    AutonPID();

    if (globalTimer % (6 * timerTickRate) < (3 * timerTickRate)) {  // flips 180 or drives 1m in alternating directions at regular intervals
      desiredDist = isTuningTurns ? 0 : 50 * degPerCM;
      desiredHeading = isTuningTurns ? 90 : 0;
    } else {
      desiredDist = isTuningTurns ? 0 : -50 * degPerCM;
      desiredHeading = isTuningTurns ? -90 : 0;
    }

    globalTimer++;
    delay(tickDeltaTime);
  }
}

#pragma endregion

#pragma endregion  // end of PID block


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region ActualCompetitionFunctions

#pragma region AutonomousFunctions //Functions for autonomous control

float previousErrorF = 0;  // previous error variable for the flystick arm

bool allowAdjust = false;

void AdjustFlystick(bool move) {
  float fP = 0.6;
  float fD = 0.8;
  float fO = 1.5;

  int deadzoneF = 0.3;

  // responsible for changing/maintaining the position of the flystick using a
  // PD controller the integral men live in my walls

  int desiredRotation;
  float armPos = ArmRot.get_angle() / 100;

  if (flystickArmPos == 0) {  // kills the function if the arm is in kickstand
                              // mode, or out of bounds
    return;
  }

  switch (flystickArmPos) {
    case 0:
      desiredRotation = defaultArmPos;
      break;
    case 1:
      desiredRotation = 130;
      maxFlywheelSpeed = 90;
      break;
    case 2:
      desiredRotation = 160;
      maxFlywheelSpeed = 45;
      break;
    case 3:
      desiredRotation = 215;
      maxFlywheelSpeed = 70;
      break;
    case 4:
      desiredRotation = 265;
      break;
    case 5:
      desiredRotation = 305;
      break;
  }

  float currentErrorF = desiredRotation - armPos;
  float errorDerivativeF = currentErrorF - previousErrorF;

  previousErrorF = currentErrorF;

  if (!move) {  // return the function before the arm is allowed to move to prevent unwanted motion
    return;
  }

  if (abs(currentErrorF) >= deadzoneF) {
    FlystickArmM.move_velocity(fO * (currentErrorF + errorDerivativeF));
  } else {
    FlystickArmM.move_velocity(0);
  }
}



int prevWheelPos;
int bopItSelectTimeStamp = 0;

int selectorStage = 0;
int selectedRoute = 3;

int startingWheelPos;

std::array<int, 2> ReadBopItOutputs() {
  lcdControl();

  int armInput = (ArmRot.get_angle() / 10000);

  int wheelDelta = abs(startingWheelPos - prevWheelPos);

  PrintToController("wheelDelt: %d", wheelDelta, 0, 2);
  PrintToController("refPos: %d", startingWheelPos, 1, 2);
  PrintToController("prevPos: %d", prevWheelPos, 2, 2);


  int wheelInput;

  if (IsWithinRange(wheelDelta, (startingWheelPos + 50), (startingWheelPos + 100)) && timeSincePoint(bopItSelectTimeStamp) > 10) {
    wheelInput = (wheelDelta == abs(wheelDelta)) ? -1 : 1;
    bopItSelectTimeStamp = globalTimer;
    startingWheelPos = FlywheelM.get_position();
  } else {
    wheelInput = 0;
  }

  prevWheelPos = (globalTimer % 5 == 0) ? (FlywheelM.get_position() - startingWheelPos) : prevWheelPos;

  return {armInput, wheelInput};
}



void controllerAutonSelect() {  // cool, efficient, but controllers are disabled during pre-auton so entirely useless

  while ((selectorStage < 2) && (globalTimer < (10 * timerTickRate))) {
    lcdControl();

    switch (selectorStage) {
      case 0:

        if (MainControl.get_digital_new_press(DIGITAL_UP) && selectedRoute < 4) {
          selectedRoute++;
        }
        if (MainControl.get_digital_new_press(DIGITAL_DOWN) && selectedRoute > 1) {
          selectedRoute--;
        }
        if (MainControl.get_digital_new_press(DIGITAL_A)) {
          selectorStage++;
        }

        PrintToController("Select Auton Route:", 0, 0, 1);
        PrintToController("Sk: 1 Off: 2 Def: 3", 0, 1, 1);
        PrintToController("Current Route: %d", selectedRoute, 2, 1);

        break;

      case 1:

        if (MainControl.get_digital_new_press(DIGITAL_A)) {
          selectorStage++;
        }

        if (MainControl.get_digital_new_press(DIGITAL_B)) {
          selectorStage--;
        }

        PrintToController("Selected Route:", 0, 0, 1);
        PrintToController("Confirm/Back: (A/B)", 0, 2, 1);

        switch (selectedRoute) {
          case 1:
            PrintToController("        Skills", 0, 1, 1);
            break;
          case 2:
            PrintToController("       Offence", 0, 1, 1);
            break;
          case 3:
            PrintToController("       Defence", 0, 1, 1);
            break;
        }

        break;
    }

    globalTimer++;  // need timer for print function
    delay(tickDeltaTime);
  }

  PrintToController("Auton %d Selected", selectedRoute, 1, 1);
  globalTimer = 0;
}



struct AutonCommand {  // structure containing all neccessary data for an autonomous command
  const float desiredDistInCM;
  const float desiredHeading;
  const int armPos;
  const int flywheelSpeed;
  const bool wingsOut;
  const int endDelay;

  const int targetSpeed;
};



const int stepChangeCooldown = timerTickRate / 3;  // sets the minimum delay between auton steps
int stepChangeTimeStamp = 0;                       // stores the time of the last step change

// initializing data variables (used to track details of each step)
int autonStep = 0;  // tracks which step of the auton the program is on
int flywheelSpeed = 0;
int prevFlywheelSpeed = 0;
int prevArmPos = 0;
bool wingsOut = false;
int endDelayTimeStamp = 0;  // holds the timestamp at which the current step will end

vector<float> autonCommands[50];

void ReadAutonStep() {
  vector<float> currentCommand = autonCommands[autonStep];

  desiredDist += currentCommand.at(0) * degPerCM;
  desiredHeading += currentCommand.at(1);

  flystickArmPos = (currentCommand.at(2) == 0) ? prevArmPos : currentCommand.at(2);
  flywheelSpeed = (currentCommand.at(3) == -1) ? prevFlywheelSpeed : currentCommand.at(3);

  endDelayTimeStamp = ((currentCommand.at(5) * timerTickRate) > stepChangeCooldown) ? (currentCommand.at(5) * timerTickRate) : stepChangeCooldown;

  // extends or retracts both wings depending on input
  wingsOut = currentCommand.at(4);
  WingPL.set_value(wingsOut);
  WingPR.set_value(wingsOut);
}

#pragma endregion  // end of AutonFunctions

#pragma region UserControlFunctions //handles all functions involving user input

int RAccelX = 0;
int LAccelX = 0;

bool isDriveReversed = false;
int reverseDrive = 1;

void DrivingControl(int8_t printingPage) {  // resoponsible for user control of the drivetrain

  if (MainControl.get_digital_new_press(DIGITAL_Y)) {  // inverts the drive upon button press, including steering
    isDriveReversed = !isDriveReversed;

    FwdDrive.set_reversed(isDriveReversed);
    RvsDrive.set_reversed(!isDriveReversed);

    reverseDrive = (isDriveReversed) ? 1 : -1;
  }

  // taking the position of both controller sticks. Y is forward/back, X is left/right

  float YStickPercent = MainControl.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) / 1.27;                    // w on graph
  float XStickPercent = (MainControl.get_analog(E_CONTROLLER_ANALOG_RIGHT_X) / 1.27 * reverseDrive);  // s on graph


  // increasing/decreasing the acceleratory variables whether the sticks are held down or not


  static float ptsPerTick = 4;

  LAccelX += (abs(YStickPercent) > deadband) && (LAccelX <= 100) ? ptsPerTick : -ptsPerTick;  // Y(x) on graph

  RAccelX += (abs(XStickPercent) > deadband) && (RAccelX <= 100) ? ptsPerTick : -ptsPerTick;  // X(x) on graph


  // applying the acceleratory curve to the stick inputs, multiplies the stick values by the output of the accel smoothing function

  int lateralOutput = (YStickPercent > 0) ? (LAccelX, YStickPercent / 100) : 0;
  int rotationalOutput = (XStickPercent > 0) ? AccelSmoothingFunc(RAccelX, XStickPercent / 100) : 0;

  // allows for turning at high speeds by lowering the "maximum" average power of the drive based on stick values.
  // Limits situations in which the functions are trying to return values over 100%, which would nerf turns (as they're be capped at 100% power IRL)
  int maxOutExponent = abs(YStickPercent * XStickPercent) / pow(100, 2);  // d on graph abs(YStickPercent * XStickPercent) / pow(100, 2);
  int maxOutputAdjust = (XStickPercent / abs(XStickPercent)) * powf(RAccelX, maxOutExponent);


  // converting the fwd/bckwd/turning power into output values for the left and right halves of the drivetrain, then driving if applicable

  int leftOutput = clamp(((lateralOutput + rotationalOutput) - maxOutputAdjust + 1), -100, 100);
  int rightOutput = clamp((((lateralOutput - rotationalOutput)) + maxOutputAdjust - 1), -100, 100);

  if ((abs(YStickPercent) + abs(XStickPercent)) >= deadband) {
    LDrive.move_velocity(6 * leftOutput);  // stepping up the output from 0-100% to 0-600rpm
    RDrive.move_velocity(6 * rightOutput);
  } else {
    LDrive.move_velocity(0);
    RDrive.move_velocity(0);
  }


  // diagnostic printing
  PrintToController("LDrive: %d", (10 * leftOutput), 1, printingPage);
  PrintToController("RDrive: %d", (10 * rightOutput), 2, printingPage);


  PrintToController("YAccelX: %d", LAccelX, 0, printingPage + 1);
  PrintToController("YMult: %d", (100 * AccelSmoothingFunc(1, LAccelX)), 1, printingPage + 1);
  PrintToController("YOut %d", lateralOutput, 2, printingPage + 1);

}  // graphed and simulated at [https://www.desmos.com/calculator/7tgvu9hlvs], modelled in % power output by default


#pragma region AuxiliaryStuff

bool flywheelFWD = true;  // flywheel is inherently inverted so this reads as the opposite of what it does
bool flywheelOn = false;

void FlystickControl(int8_t printingPage) {  // done
  // responsible for selecting the elevation of the flystick

  int cooldown = 3;

  if (MainControl.get_digital_new_press(DIGITAL_UP) && flystickArmPos < maxflystickArmPos) {
    flystickArmPos++;
    lastUpTimestamp = globalTimer;
  }

  if (MainControl.get_digital_new_press(DIGITAL_DOWN) && flystickArmPos > 1) {
    flystickArmPos--;
    lastDownTimestamp = globalTimer;
  }

  if (MainControl.get_digital_new_press(DIGITAL_B)) {
    flywheelFWD = !flywheelFWD;
    FlywheelM.set_reversed(flywheelFWD);
  }

  switch (flywheelOn) {
    case 0:
      FlywheelM.move_velocity(0);
      break;
    case 1:
      FlywheelM.move_velocity(maxFlywheelSpeed * 6);
      break;
  }

  if ((globalTimer - lastSpinTimestamp >= cooldown) && MainControl.get_digital_new_press(DIGITAL_A)) {
    flywheelOn = !flywheelOn;
  }
}



bool RWingLockedOut = false;
bool LWingLockedOut = false;

void WingsControl() {  // done

  if (MainControl.get_digital_new_press(DIGITAL_L2)) {
    LWingLockedOut = !LWingLockedOut;
    WingPL.set_value(LWingLockedOut);
  }

  if (MainControl.get_digital_new_press(DIGITAL_R2)) {
    RWingLockedOut = !RWingLockedOut;
    WingPR.set_value(RWingLockedOut);
  }
}

#pragma endregion

#pragma endregion  // end of UserControlFunctions

#pragma endregion  // end of Bot controlling functions


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region Pregame //code which executes before a game starts

void initialize() {
  WingPL.set_value(false);
  WingPR.set_value(false);

  FullDrive.tare_position();
  FullDrive.set_zero_position(0);
  startingWheelPos = FlywheelM.get_position();

  mStartPosL = LDriveTopM.get_position();
  mStartPosR = RDriveTopM.get_position();

  FullDrive.set_brake_modes(E_MOTOR_BRAKE_COAST);

  FlywheelM.set_brake_mode(E_MOTOR_BRAKE_COAST);

  FlystickArmM.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  ArmRot.reverse();

  defaultArmPos = ArmRot.get_position() / 100;
  Inertial.reset(true);
}

void disabled() {}


void competition_initialize() {  // auton selector (bop-it!)

  FlystickArmM.set_brake_mode(E_MOTOR_BRAKE_COAST);

  while ((selectorStage < 2) && (globalTimer < (20 * timerTickRate))) {
    selectorStage += ReadBopItOutputs().at(1);  // 0 = no spin, 1 = fwd, -1 = bwd. Has a cooldown between inputs

    switch (selectorStage) {
      case 0:

        selectedRoute = ReadBopItOutputs().at(0);  // takes the position of the arm ONLY if in first stage of selector

        PrintToController("Select Auton Route:", 0, 0, 1);
        PrintToController("Sk: 1 Off: 2 Def: 3", 0, 1, 1);
        PrintToController("Current Route: %d", selectedRoute, 2, 1);

        break;

      case 1:

        PrintToController("Selected Route:", 0, 0, 1);
        PrintToController("Confirm  /  Back", 0, 2, 1);

        switch (selectedRoute) {
          case 1:
            PrintToController("        Skills", 0, 1, 1);
            break;
          case 2:
            PrintToController("       Offence", 0, 1, 1);
            break;
          case 3:
            PrintToController("       Defence", 0, 1, 1);
            break;
        }

        break;
    }

    globalTimer++;  // timer for print function and emergency killswitch
    delay(tickDeltaTime);
  }

  FlystickArmM.set_brake_mode(E_MOTOR_BRAKE_COAST);
  PrintToController("Auton %d Selected", selectedRoute, 1, 1);
  globalTimer = 0;
}



#pragma region autonRoutes

int totalNumOfCommands = 50;

void skillsAuton() {
  // autonCommands[ autonStep ] = {[]} [lateralDistance(cm), rotationalDistance(degrees), flystickArmPos(1-5, 0 = no change), flywheelSpeed(%),
  // wingsOut(bool), delay(seconds)]

  autonCommands[0] = {0, 0, 0, 0, 0, 0};  // null start, copy/paste to make new step and KEEP AS POSITION 0
  autonCommands[1] = {0, 0, 0, 0, 0, 0};
  autonCommands[2] = {0, 0, 3, 100, 0, 35};    // Match loads for 35 seconds
  autonCommands[3] = {20, 0, 0, -1, 0, 0};     // Move forward in preperation for turn
  autonCommands[4] = {0, 90, 0, -1, 0, 0};     // 90 degree turn right to align robot for triball scoring in net
  autonCommands[5] = {-65, 0, 1, 0, 0, 0};     // push preload triballs towards goal
  autonCommands[6] = {20, 0, 0, 0, 0, 0};      // move back (preparing for 2nd push into goal)
  autonCommands[7] = {-25, 0, 0, 0, 0, 0};     // push triballs into goal
  autonCommands[8] = {110, 0, 0, 0, 0, 0};     // move towardsd wall, triballs are scored, time to try and score
                                               // triballs in oposite goal
  autonCommands[9] = {0, -45, 0, 0, 0, 0};     // turn to be parallel with the arena, facing our colors low hang bar
  autonCommands[10] = {200, 0, 0, 0, 0, 0};    // move from our side to the other side of the feild
  autonCommands[11] = {0, -45, 0, 0, 0, 0};    // turn towards the net
  autonCommands[12] = {70, 0, 0, 0, 0, 0};     // push triballs into the net
  autonCommands[13] = {0, -45, 0, 0, 0, 0};    // turn to face net
  autonCommands[14] = {-20, 0, 0, 0, 0, 0};    // move back to be in position to push triballs in the side of the net again
  autonCommands[15] = {45, 0, 0, 0, 0, 0};     // push triballs into net
  autonCommands[16] = {0, -90, 0, 0, 0, 0};    // turn away from the net to get into position to push the front
  autonCommands[17] = {70, 0, 0, 0, 0, 0};     // drive to get ahead of goal net
  autonCommands[18] = {0, 45, 0, 0, 0, 0};     // turn to be perpendicular to match load zone
  autonCommands[19] = {15, 0, 0, 0, 0, 0};     // position ourselves more in front of the net
  autonCommands[20] = {0, 112.5, 0, 0, 0, 0};  // turn to face front of net
  autonCommands[21] = {60, 0, 0, 0, 0, 0};     // push triballs into front of net
  // samich yummmmmmmmmy

  totalNumOfCommands = 21;
}

void offenceAuton() {  // starting on the enemy side of the field (no match
                       // loading)
  // autonCommands[ autonStep ] = {[]}
  //[lateralDistance(cm), rotationalDistance(degrees), flystickArmPos(1-5, 0 =
  // no change), flywheelSpeed(%), wingsOut(bool), delay(seconds)]

  autonCommands[0] = {0, 0, 0, 0, 0, 0};     // null start, copy/paste to make new step and KEEP AS POSITION 0
  autonCommands[1] = {-35, 0, 0, 0, 0, 0};   // drive along match load bar
  autonCommands[2] = {0, -10, 0, 0, 0, 0};   // turn slightly away from wall
  autonCommands[3] = {-45, 0, 0, 0, 0, 0};   // ram preload into net
  autonCommands[4] = {60, 0, 0, 0, 0, 0};    // drive back to halfway along match load bar
  autonCommands[5] = {0, 80, 0, 0, 0, 0};    // turn to face away from match load bar
  autonCommands[6] = {10, 0, 5, 80, 0, 0};   // lower arm and spin flywheel
  autonCommands[7] = {-30, 0, 0, -1, 0, 0};  // reverse into corner tribal, launching it out of corner
  autonCommands[8] = {0, 100, 3, 0, 0, 0};   // stop flystick and turn towards horizontal climb bar
  autonCommands[9] = {-35, 0, 0, 0, 0, 0};   // drive to entrance of climb bar corridor
  autonCommands[10] = {0, 45, 0, 0, 0, 0};   // turn to face horizontal climb bar
  autonCommands[11] = {-90, 0, 0, 0, 0, 0};  // drive until arm is touching horizontal bar

  totalNumOfCommands = 0;
}

void defenceAuton() {  // starting on the team side of the field (match loading)
  // autonCommands[ autonStep ] = {[]}
  //[lateralDistance(cm), rotationalDistance(degrees), flystickArmPos(1-5, 0 =
  // no change), flywheelSpeed(%), wingsOut(bool), delay(seconds)]

  autonCommands[0] = {0, 0, 0, 0, 0, 0};  // null start, copy/paste to make new step and KEEP AS POSITION 0
  autonCommands[1] = {0, 0, 0, 0, 0, 1};
  autonCommands[2] = {-35, 0, 0, 0, 0, 0};   // drive along match load bar
  autonCommands[3] = {0, 10, 0, 0, 0, 0};    // turn slightly away from wall
  autonCommands[4] = {-45, 0, 0, 0, 0, 0};   // ram preload into net
  autonCommands[5] = {60, 0, 0, 0, 0, 0};    // drive back to halfway along match load bar
  autonCommands[6] = {0, -90, 0, 0, 0, 0};   // turn to face away from match load bar
  autonCommands[7] = {20, 0, 5, 80, 0, 2};   // lower arm and spin flywheel
  autonCommands[8] = {-40, 0, 0, -1, 0, 2};  // reverse into corner tribal, launching it out of corner
  autonCommands[9] = {0, -100, 3, 0, 0, 0};  // stop flystick and turn towards horizontal climb bar
  autonCommands[10] = {-35, 0, 0, 0, 0, 0};  // drive to entrance of climb bar corridor
  autonCommands[11] = {0, -45, 0, 0, 0, 0};  // turn to face horizontal climb bar
  autonCommands[12] = {-90, 0, 0, 0, 0, 0};  // drive until arm is touching horizontal bar

  totalNumOfCommands = 12;
}

#pragma endregion

#pragma endregion


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //



void autonomous() {
  // selectedRoute = 3;

  switch (selectedRoute) {
    case 1:
      skillsAuton();
      break;
    case 2:
      offenceAuton();
      break;
    case 3:
      defenceAuton();
      break;
  }

  desiredDist = 0;
  desiredHeading = 0;
  FullDrive.move_velocity(0);

  while (true) {
    lcdControl();  // allows the LCD screen to show multiple pages of diagnostics, press left/right arrows to change pages
    FlywheelM.move_velocity(flywheelSpeed * 2);  // spins the flywheel at the desired speed (input as a percent)
    if ((ArmRot.get_angle() / 100) < 306) {
      AdjustFlystick(true);
    }  // manages the height of the flystick arm

    float inerHeading = (Inertial.get_heading() > 180) ? (-1 * (360 - Inertial.get_heading())) : Inertial.get_heading();

    bool stepPIDIsComplete = AutonPID();

#pragma region diagnostics

    // reports values relating to the movement of the flystick on page 1
    PrintToController("Step: %d", autonStep, 0, 1);
    PrintToController("Timer: %d", globalTimer, 1, 1);
    PrintToController("Route: %d", selectedRoute, 2, 1);

    // reports values relating to the overall progression of the autonomous
    // on page 2
    PrintToController("Timer: %d", globalTimer, 0, 2);
    PrintToController("Wing??: %d", wingsOut, 1, 2);
    PrintToController("DelayDone?: %d", (globalTimer - endDelayTimeStamp), 2, 2);

    // reports values relating to the PID movement on pages 3 & 4
    PrintToController("Complete?: %d", stepPIDIsComplete, 0, 3);
    PrintToController("Head: %d", inerHeading, 1, 3);
    PrintToController("ErrorR: %d", errorProportionalR, 2, 3);

    PrintToController("Complete?: %d", stepPIDIsComplete, 0, 4);
    PrintToController("TargDist: %d", desiredDist, 1, 4);
    PrintToController("ErrorL: %d", (RDriveTopM.get_position() + LDriveTopM.get_position()) / 2, 2, 4);

#pragma endregion

    if (autonStep > totalNumOfCommands) {  // kills the program if auton
                                           // route is complete
      while (true) {
        FullDrive.move_velocity(0);
        PrintToController("Out of bounds", 0, 1, 1);

        globalTimer++;
        delay(tickDeltaTime);
      }
    } else if ((MainControl.get_digital_new_press(DIGITAL_X) || stepPIDIsComplete) && (timeSincePoint(stepChangeTimeStamp) > endDelayTimeStamp)) {
      autonStep++;

      stepChangeTimeStamp = globalTimer;
      prevArmPos = flystickArmPos;
      prevFlywheelSpeed = flywheelSpeed;

      ReadAutonStep();
    }

    globalTimer++;
    delay(tickDeltaTime);
  }
}



// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //



void opcontrol() {
  // competition_initialize();
  //  autonomous();

  // tunePID();

  std::array<double, 3> displacement = {0.0, 0.0, 0.0};

  while (true) {
    std::array<double, 3> velocity = calculateKinematics(true, true);
    displacement[0] = calculateKinematics(false, false)[0] * 10000;

    lcdControl();

    PrintToController("Time: %d", globalTimer, 0, 1);
    PrintToController("XDispl: %d", displacement.at(0) * 100, 1, 1);
    PrintToController("XAccel: %d", Inertial.get_accel().x * 98.1, 2, 1);

    globalTimer++;
    delay(tickDeltaTime);
  }

  /*flywheelFWD = true;

  flywheelOn = false;


  // vector<double> currAcceleration;

  while (true) {
    DrivingControl(1);
    WingsControl();
    FlystickControl(-1);
    AdjustFlystick(true);
    // updateGlobalPosition(true);
    lcdControl();


    PrintToController("Time: %d", globalTimer, 0, 1);
    // PrintToController("XDispl: %d", displacement.at(0), 1, 1);
    // PrintToController("XAccel: %d", currAcceleration.at(0) * 10, 2, 1);

    globalTimer++;
    delay(tickDeltaTime);
  }*/
}