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

int maxFlywheelSpeed = 90;  // flywheel speed as a percent
int flystickArmPos = 0;     // flystick starts at kickstand position

int lastUpTimestamp = 0;
int lastDownTimestamp = 0;
int lastSpinTimestamp = 0;

float defaultArmPos;

int maxflystickArmPos = 5;

int currentPage = 1;

#pragma endregion


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region HelperFunctions //unit conversions and whatnot

const char *toChar(std::string string) { return string.c_str(); }


int toInt(float val) { return val; }


int timeSincePoint(int checkedTime) {
  return checkedTime < globalTimer ? (globalTimer - checkedTime) : -1;  // returns -1 if checkedtime is in the future
}


const bool IsWithinRange(float num, float lowerBound, float upperBound) { return num >= lowerBound && num <= upperBound; }


// variables which control the shape/range of the acceleratory function
float ACurveExtremity = 0.19948;  // sigma
float peakPos = 1;                // mu
float AMinAmount = 0.235;         // kappa

// i have no idea what im doing
float AccelSmoothingFunc(int time) {  // returns a multiplier 0 to 1 based on time
  float x = time / 100;               // converting the input from percentage to a decimal btween 0-1

  const float multiplier =
      (0.5 / (ACurveExtremity * sqrt(2 * Pi))) * powf(e, (-0.5 * pow(((AMinAmount * x - AMinAmount * peakPos) / ACurveExtremity), 2)));
  return time >= (100) ? multiplier : 1;
}  // function graphed here: [https://www.desmos.com/calculator/y0fwlh6j47]


float linearHarshness = 0.2;  // g on graph
float SCurveExtremity = 4.7;  // h on graph

// i now have some idea what im doing
float StickSmoothingFunc(float stickVal) {
  float curveExponent = (abs(stickVal) - 100) / (10 * ACurveExtremity);
  float linearExponent = (-1 / (10 * linearHarshness));

  return (stickVal * (powf(e, linearExponent) + ((1 - powf(e, linearExponent)) * powf(e, curveExponent))));
}  // function graphed here: [https://www.desmos.com/calculator/ti4hn57sa7]



#pragma endregion


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region printingChicanery


void lcdControl() {
  if (MainControl.get_digital_new_press(DIGITAL_LEFT)) {
    currentPage--;
  }
  if (MainControl.get_digital_new_press(DIGITAL_RIGHT)) {
    currentPage++;
  }
}


bool isPrintingList[9] = {false, false, false, false, false, false, false, false, false};  // tracks which functions are trying to print
const int pagesPerPrint[9] = {1, 1, 1, 2, 2, 1, 2, 1, 2};  // hardcoded list containing the number of pages required for each function
/**
 * [index] [function] - [num of allocated pages]
 * [0] Rand Diagnostics - 1  //should not be used in final polished builds
 * [1] AutoSel - 1
 * [2] AutRoute - 1
 * [3] PID - 2
 * [4] Drivetrain - 2
 * [5] GPS - 1
 * [6] Kinematics - 2
 * [7] Drive Tune - 1
 * [8] PID Tune - 2
 **/

int pageRangeFinder(int index) {
  int startingPage;

  for (int j = 0; j < index; ++j) {
    startingPage += isPrintingList[j] ? pagesPerPrint[j] : 0;
  }

  return startingPage;
}


void PrintToController(std::string prefix, double data, int numOfDigits, int row, int page) {  // handles single numbers
  if (currentPage == page && (globalTimer % 9 == (row * 3))) {
    if (globalTimer % 11 == 0) {  // refresh the screen every 11 ticks because 11 is a good number :)
      MainControl.clear();
    }

    std::string output =
        prefix + std::to_string(data).substr(0, numOfDigits + 1);  // takes the first n digits of the number, adds it to output as string

    MainControl.print(row, 0, output.c_str(), 0);
  }
}


template <typename T, size_t N>
void PrintToController(std::string prefix, const std::array<T, N> &data, int numOfDigits, int row, int page) {  // handles multiple numbers
  if (currentPage == page && (globalTimer % 9 == (row * 3))) {
    if (globalTimer % 11 == 0) {  // refresh the screen every 11 ticks because 11 is a good number :)
      MainControl.clear();
    }

    std::string output = prefix;

    for (size_t j = 0; j < N; ++j) {
      double currNum = data[j];
      output += (j < N - 1) ? std::to_string(currNum).substr(0, numOfDigits + 1) + ", " : std::to_string(currNum).substr(0, numOfDigits + 1);
    }

    MainControl.print(row, 0, output.c_str(), 0);
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
    if (!isPrintingList[6]) {  // [6] Kinematics - 2
      isPrintingList[6] = true;
    }

    int startingPage = pageRangeFinder(6);

    PrintToController("Time: ", globalTimer, 5, 0, startingPage);
    PrintToController("Accel: ", currAcceleration, 3, 1, startingPage);
    PrintToController("cVel: ", currVelocity, 3, 2, startingPage);

    PrintToController("pVel: ", prevVelocity, 3, 1, startingPage + 1);
    PrintToController("Displ: ", distTravelled, 3, 2, startingPage + 1);
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
    if (!isPrintingList[5]) {  // [5] GPS - 1
      isPrintingList[5] = true;
    }

    int startingPage = pageRangeFinder(5);

    PrintToController("Time: ", globalTimer, 3, 0, startingPage);
    PrintToController("Displ: ", currDisplacements, 3, 1, startingPage);
    PrintToController("Coords: ", globalCoordinates, 3, 2, startingPage);

    PrintToController("Heading: %d", thetaHeading, 4, 0, 4);  // print angles
    PrintToController("Pitch: %d", thetaPitch, 1, 4, 4);
    PrintToController("Yaw: %d", thetaYaw, 2, 4, 4);
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

float proportionalErrorL;  // reported value - desired value = position
float previousErrorL = 0;  // position from last loop
float derivativeErrorL;    //(error - prevError)
float integralErrorL;

float lateralPower = 0;

// Storage variables for Rotational (turning) PID

float proportionalErrorR;  // reported value - desired value = position
float previousErrorR = 0;  // position from last loop
float derivativeErrorR;    //(error - prevError)
float integralErrorR;

float rotationalPower = 0;

// the second set of "rotational" storage vars above are technically useless, as the code executes in such a way that only one set of vars is needed
// to produce both outputs. however, readability is nice. change if memory ever becomes an issue

#pragma endregion  // end of PID variable declaration

// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //

int stuckTimeStamp = 0;
int avgMotorPosition = 0;

bool AutonPID(bool isPrinting) {
  if (autonPIDIsEnabled) {  // toggle so the PID can be disabled while placed on a separate thread
    // sets heading from -180 < h < 180, meaning we turn the correct direction from error
    float heading = (Inertial.get_heading() > 180) ? (Inertial.get_heading() - 360) : Inertial.get_heading();

    ///////////////////////////////////////
    //////        Lateral PID        //////
    ///////////////////////////////////////

    avgMotorPosition = ((RDriveFrontM.get_position()) + (LDriveFrontM.get_position())) / 2;

    proportionalErrorL = lP * (desiredDist - avgMotorPosition);     // proportional error
    derivativeErrorL = lD * (proportionalErrorL - previousErrorL);  // derivative of error

    // filters out the integral at short ranges (no I if |error| < constant lower limit, eg. 10cm),
    // allowing it to be useful when needed without overpowering other elements
    if (fabs(proportionalErrorL) > integralBoundL) {
      integralErrorL += lI * (proportionalErrorL);
    } else {
      integralErrorL = 0;
    }

    lateralPower = (proportionalErrorL + derivativeErrorL + integralErrorL) * lOutput;

    ///////////////////////////////////////
    //////      Rotational PID       //////
    ///////////////////////////////////////

    proportionalErrorR = rP * (desiredHeading - heading);           // proportional error
    derivativeErrorR = rD * (proportionalErrorR - previousErrorR);  // derivative of error

    // filters out the integral at short ranges (no I if |error| < constant lower limit, eg. 10cm),
    // allowing it to be useful when needed without overpowering other elements
    if (fabs(proportionalErrorR) > integralBoundR) {
      integralErrorR += rI * (proportionalErrorR);
    } else {
      integralErrorR = 0;
    }

    rotationalPower = (proportionalErrorR + derivativeErrorR + integralErrorR) * rOutput;

    ///////////////////////////////////////
    //////        ending math        //////
    ///////////////////////////////////////

    LDrive.move_velocity(autonDriveMult * (lateralPower + rotationalPower));
    RDrive.move_velocity(autonDriveMult * (lateralPower - rotationalPower));

    previousErrorL = proportionalErrorL;
    previousErrorR = proportionalErrorR;

    if (isPrinting) {
      if (!isPrintingList[3]) {  // [3] PID - 2
        isPrintingList[3] = true;
      }

      int startingPage = pageRangeFinder(3);

      PrintToController("Time: ", globalTimer, 5, 0, startingPage);
      PrintToController("LOut: ", (autonDriveMult * (lateralPower + rotationalPower)), 5, 0, startingPage);
      PrintToController("ROut: ", (autonDriveMult * (lateralPower + rotationalPower)), 5, 0, startingPage);

      PrintToController("LError: ", proportionalErrorL, 5, 0, startingPage + 1);
      PrintToController("LatOut: ", lateralPower, 5, 1, startingPage + 1);
      PrintToController("RotOut: ", rotationalPower, 5, 2, startingPage + 1);
    }
  }

  return (fabs(proportionalErrorL) <= (3 * degPerCM) && fabs(proportionalErrorR) <= 1.5) ? true : false;  // return true if done movement
}


#pragma endregion  // end of PID block


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region ActualCompetitionFunctions

#pragma region AutonomousFunctions //Functions for autonomous control

float previousErrorF = 0;  // previous error variable for the flystick arm

bool allowAdjust = false;

void AdjustFlystick(bool isPrinting, bool move) {
  const float fP = 0.7;
  const float fD = 0.8;
  const float fO = 1.7;

  int deadzoneF = 0.3;

  // responsible for changing/maintaining the position of the flystick using a PD controller, as the integral men live in my walls

  int desiredRotation;
  float armPos = ArmRot.get_angle() / 100;

  if (flystickArmPos == 0) {  // kills the function if the arm is in kickstand mode or out of bounds
    return;
  }

  switch (flystickArmPos) {
    case 0:
      desiredRotation = defaultArmPos;
      break;
    case 1:
      desiredRotation = 130;
      break;
    case 2:
      desiredRotation = 160;
      break;
    case 3:
      desiredRotation = 215;
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

std::array<int, 2> ReadBopItOutputs(bool isPrinting) {
  lcdControl();

  int armInput = (ArmRot.get_angle() / 10000);

  int wheelDelta = abs(startingWheelPos - prevWheelPos);


  int wheelInput;

  if (IsWithinRange(wheelDelta, (startingWheelPos + 50), (startingWheelPos + 100)) && timeSincePoint(bopItSelectTimeStamp) > 10) {
    wheelInput = (wheelDelta == abs(wheelDelta)) ? -1 : 1;
    bopItSelectTimeStamp = globalTimer;
    startingWheelPos = FlywheelM.get_position();
  } else {
    wheelInput = 0;
  }

  prevWheelPos = (globalTimer % 5 == 0) ? (FlywheelM.get_position() - startingWheelPos) : prevWheelPos;

  if (isPrinting) {
    int startPage = pageRangeFinder(0);

    PrintToController("wheelDelt: %d", wheelDelta, 3, 0, startPage);
    PrintToController("refPos: %d", startingWheelPos, 3, 1, startPage);
    PrintToController("prevPos: %d", prevWheelPos, 2, 3, startPage);
  }

  return {armInput, wheelInput};
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

int RAccelTime = 0;
int LAccelTime = 0;

bool isDriveReversed = false;
int reverseDrive = 1;

int prevXStickPercent = 0;
int prevYStickPercent = 0;

void DrivingControl(bool isPrinting) {  // resoponsible for user control of the drivetrain

  if (MainControl.get_digital_new_press(DIGITAL_Y)) {  // inverts the drive upon button press, including steering
    reverseDrive = (reverseDrive > 0) ? 1 : -1;
  }

  // taking the position of the sticks and appplying gradient diffusion to them. Check the StickSmoothingFunc graph for details
  // X stick covers fwd/back, Y stick covers turning

  float XStickPercent = StickSmoothingFunc(MainControl.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) / 1.27 * reverseDrive);   // w on graph
  float YStickPercent = StickSmoothingFunc(MainControl.get_analog(E_CONTROLLER_ANALOG_RIGHT_X) / 1.27 * reverseDrive);  // s on graph


  // filter out stick drift / nonpressed sticks. saves resources by skipping calculations when not driving
  if ((abs(XStickPercent) + abs(YStickPercent)) >= deadband) {
    int fullStopThreshold = 150;

    static float ptsPerTick = 4;                                   // inreasing or decreasing the acceleration functions' timer
    LAccelTime += (LAccelTime <= 100) ? ptsPerTick : -ptsPerTick;  // Y(x) on graph
    RAccelTime += (RAccelTime <= 100) ? ptsPerTick : -ptsPerTick;  // X(x) on graph


    // applying the acceleratory curve to the stick inputs, multiplies the stick values by the output of the accel smoothing function

    int lateralOutput = AccelSmoothingFunc(LAccelTime) * XStickPercent;

    float rotationalMult = (-0.000014 * powf(lateralOutput, 2)) + (-0.0061 * lateralOutput) + 1;
    // graphed and explained here: [https://www.desmos.com/calculator/zyd1xfamrm]

    int rotationalOutput = (rotationalMult * AccelSmoothingFunc(RAccelTime) * YStickPercent);  // ((100 - abs(lateralOutput)) / 100)

    // converting the fwd/bckwd/turning power into output values for the left and right halves of the drivetrain, then driving


    int leftOutput = ((lateralOutput + rotationalOutput));
    int rightOutput = ((lateralOutput - rotationalOutput));

    // implementing hard stops if the sticks are flicked the opposite way
    if (abs(prevXStickPercent - XStickPercent) > fullStopThreshold || abs(prevYStickPercent - YStickPercent) > fullStopThreshold) {
      LDrive.move_velocity(0);
      RDrive.move_velocity(0);

    } else {
      LDrive.move_velocity(5.8 * leftOutput);  // stepping up the output from 0-100% to 0-600rpm
      RDrive.move_velocity(5.8 * rightOutput);
    }

    prevXStickPercent = XStickPercent;
    prevYStickPercent = YStickPercent;

    if (isPrinting) {  // [4] Drivetrain - 2
      if (!isPrintingList[4]) {
        isPrintingList[4] = true;
      }

      int startPage = pageRangeFinder(4);

      PrintToController("Time: ", globalTimer, 5, 0, startPage);
      PrintToController("LDrive: ", leftOutput, 5, 1, startPage);
      PrintToController("RDrive: ", rightOutput, 5, 2, startPage);

      PrintToController("OutAdj: ", rotationalMult, 5, 0, startPage + 1);
      PrintToController("LOut: ", lateralOutput, 5, 1, startPage + 1);
      PrintToController("ROut: ", rotationalOutput, 5, 2, startPage + 1);
    }


  } else {  // if not want move, dont move
    LDrive.move_velocity(0);
    RDrive.move_velocity(0);

    if (isPrinting) {  // [4] Drivetrain - 2
      if (!isPrintingList[4]) {
        isPrintingList[4] = true;
      }

      int startPage = pageRangeFinder(4);

      PrintToController("Time: ", globalTimer, 5, 0, startPage);
      PrintToController("LDrive: %d", 0.0, 5, 1, startPage);
      PrintToController("RDrive: %d", 0.0, 5, 2, startPage);

      PrintToController("OutAdj: ", 0.0, 5, 0, startPage + 1);
      PrintToController("LOut: %d", 0.0, 5, 1, startPage + 1);
      PrintToController("ROut: %d", 0.0, 5, 2, startPage + 1);
    }
  }
}  // graphed and simulated at [https://www.desmos.com/calculator/7tgvu9hlvs], modelled in % power output by default. Graph may be outdated


#pragma region AuxiliaryFunctions

bool flywheelFWD = true;  // flywheel is inherently inverted so this reads as the opposite of what it does
bool flywheelOn = false;

void FlystickControl(bool isPrinting) {  // controls driver interaction with the flystick

  const int cooldown = 5;

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


  if ((globalTimer - lastSpinTimestamp >= cooldown) && MainControl.get_digital_new_press(DIGITAL_A)) {
    flywheelOn = !flywheelOn;
  }

  switch (flywheelOn) {
    case 0:
      FlywheelM.move_velocity(0);
      break;
    case 1:
      FlywheelM.move_velocity(maxFlywheelSpeed * 2);
      break;
  }
}



bool RWingLockedOut = false;
bool LWingLockedOut = false;

void WingsControl() {  // controls... wings

  if (MainControl.get_digital_new_press(DIGITAL_L2)) {
    LWingLockedOut = !LWingLockedOut;
    WingPL.set_value(LWingLockedOut);
  }

  if (MainControl.get_digital_new_press(DIGITAL_R2)) {
    RWingLockedOut = !RWingLockedOut;
    WingPR.set_value(RWingLockedOut);
  }
}

#pragma endregion  // end of AuxilaryFunctions

#pragma endregion  // end of UserControlFunctions

#pragma endregion  // end of Bot controlling functions


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region debugFunctions

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////                                                                                                                 ////
//                                                 PID TUNING INSTRUCTIONS:                                            //
//   1. call the tunePID function in opcontrol()                                                                       //
//   2. confirm that the P/I/D tuning variables in the "PIDVariables" region are set to 0.0, with the outputs at 1.0   //
//   3. follow the control layout found here: [http://tinyurl.com/3zrb6zj5]                                            //
//   4. increase the lP/rP coefficient(s) until the desired motion is completed with oscilations                       //
//   5. increase the lD/rD coefficient(s) until the oscilations dampen out over time                                   //
//   6. increase the lI/rI coefficient(s) until the motion is completed aggressively without oscilations               //
//      (somewhat optional, as not all applications benefit from an integral controller)                               //
//   7. increase the output coefficient(s) until the motion is completed with acceptable speed and precision           //
//                                                                                                                     //
//      as builds and use cases vary, you may need to fiddle with the values after initial tuning after more testing.  //
//      generally the P & D components should be larger than the I, and values should be between 0.0 and 5.0.          //
////                                                                                                                 ////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float adjustFactor = 0.1;  // the increment by which PID variables change during manual tuning
bool isTuningTurns = false;

void tunePID(bool isPrinting) {  // turns or oscilates repeatedly to test and tune the PID, allowing real-time tuning and adjustments

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


    AutonPID(false);

    if (globalTimer % (6 * timerTickRate) < (3 * timerTickRate)) {  // flips 180 or drives 1m in alternating directions at regular intervals
      desiredDist = isTuningTurns ? 0 : 50 * degPerCM;
      desiredHeading = isTuningTurns ? 90 : 0;
    } else {
      desiredDist = isTuningTurns ? 0 : -50 * degPerCM;
      desiredHeading = isTuningTurns ? -90 : 0;
    }


    if (isPrinting) {
      if (!isPrintingList[8]) {  // [8] PID Tune - 2
        isPrintingList[8] = true;
      }


      int startPage = pageRangeFinder(8);

      PrintToController("PVar: %d", (isTuningTurns ? rP : lP) * 10, 4, 0, startPage);
      PrintToController("IVar: %d", (isTuningTurns ? rI : lI) * 10, 4, 1, startPage);
      PrintToController("DVar: %d", (isTuningTurns ? rD : lD) * 10, 4, 2, startPage);

      PrintToController("Turning?: %d", isTuningTurns, 1, 0, startPage + 1);
      PrintToController("lOutput: %d", lOutput, 1, 5, startPage + 1);
      PrintToController("rOutput: %d", rOutput, 2, 5, startPage + 1);
    }


    globalTimer++;
    delay(tickDeltaTime);
  }
}



void tuneDrive(bool isPrinting) {  // allows for user driving, with real time control over drive coefficients
  // default settings for acceleratory / stick curves

  ACurveExtremity = 0.19948;  // sigma
  AMinAmount = 0.235;         // kappa

  linearHarshness = 0.2;  // g on graph
  SCurveExtremity = 4.7;  // h on graph

  float adjustFactor = 1;

  while (true) {
    if (MainControl.get_digital_new_press(DIGITAL_X)) {
      ACurveExtremity += adjustFactor / 100000;
    }
    if (MainControl.get_digital_new_press(DIGITAL_A)) {
      AMinAmount += adjustFactor / 1000;
    }
    if (MainControl.get_digital_new_press(DIGITAL_B)) {
      linearHarshness += adjustFactor / 20;
    }
    if (MainControl.get_digital_new_press(DIGITAL_Y)) {
      SCurveExtremity += adjustFactor / 10;
    }

    if (MainControl.get_digital_new_press(DIGITAL_UP)) {
      adjustFactor++;
    }
    if (MainControl.get_digital_new_press(DIGITAL_DOWN)) {
      adjustFactor--;
    }
    if (MainControl.get_digital_new_press(DIGITAL_L1)) {
      adjustFactor *= -1;
    }

    DrivingControl(false);



    if (isPrinting) {  // [7] Drive Tune - 1
      if (!isPrintingList[7]) {
        isPrintingList[7] = true;
      }

      int startPage = pageRangeFinder(7);

      PrintToController("kappa: ", ACurveExtremity, 7, 0, startPage);
      PrintToController("smigma: ", AMinAmount, 5, 1, startPage);

      std::array<float, 2> HAndG = {linearHarshness, SCurveExtremity};
      PrintToController("g and h: ", HAndG, 3, 2, startPage);
    }


    globalTimer++;
    delay(tickDeltaTime);
  }
}

#pragma endregion


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region dataStorage // should be ported to individual header files at some point

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
  autonCommands[8] = {110, 0, 0, 0, 0, 0};     // move towardsd wall, triballs are scored, time to try and score triballs in oposite goal
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


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region printingConfigs

void selectorPrinting() {     // innefficient, hardcoding would be better
  isPrintingList[0] = false;  // debug - disabled
  isPrintingList[1] = true;   // Auton Selector - enabled
  isPrintingList[2] = false;  // Auton Route - disabled
  isPrintingList[3] = false;  // PID - disabled
  isPrintingList[4] = false;  // Drivetrain - disabled
  isPrintingList[5] = false;  // GPS - disabled
  isPrintingList[6] = false;  // Kinematic Controller - disabled
  isPrintingList[7] = false;  // Drive Tuning - disabled
  isPrintingList[8] = false;  // PID Tuning - disabled
}

void autonPrinting() {
  isPrintingList[0] = false;  // debug - disabled
  isPrintingList[1] = false;  // Auton Selector - disabled
  isPrintingList[2] = true;   // Auton Route - enabled
  isPrintingList[3] = true;   // PID - enabled
  isPrintingList[4] = false;  // Drivetrain - disabled
  isPrintingList[5] = true;   // GPS - enabled
  isPrintingList[6] = false;  // Kinematic Controller - disabled
  isPrintingList[7] = false;  // Drive Tuning - disabled
  isPrintingList[8] = false;  // PID Tuning - disabled
}

void userControlPrinting() {
  isPrintingList[0] = false;  // debug - disabled
  isPrintingList[1] = false;  // Auton Selector - disabled
  isPrintingList[2] = false;  // Auton Route - disabled
  isPrintingList[3] = false;  // PID - disabled
  isPrintingList[4] = true;   // Drivetrain - enabled
  isPrintingList[5] = true;   // GPS - enabled
  isPrintingList[6] = false;  // Kinematic Controller - disabled
  isPrintingList[7] = false;  // Drive Tuning - disabled
  isPrintingList[8] = false;  // PID Tuning - disabled
}

#pragma endregion  // printingConfigs

#pragma endregion


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //


#pragma region Pregame //code which executes before a game starts

void initialize() {
  selectorPrinting();

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
    lcdControl();
    selectorStage += ReadBopItOutputs(false).at(1);  // 0 = no spin, 1 = fwd, -1 = bwd. Has a cooldown between inputs

    switch (selectorStage) {
      case 0:

        selectedRoute = ReadBopItOutputs(false).at(0);  // takes the position of the arm ONLY if in first stage of selector

        if (isPrintingList[1]) {  // [1] AutoSel - 1
          int startPage = pageRangeFinder(1);

          PrintToController("Select Auton Route:", 0, 0, 0, startPage);
          PrintToController("Sk: 1 Off: 2 Def: 3", 0, 0, 1, startPage);
          PrintToController("Current Route: ", selectedRoute, 1, 2, startPage);
        }

        break;

      case 1:

        if (isPrintingList[1]) {  // [1] AutoSel - 1
          int startPage = pageRangeFinder(1);

          PrintToController("Selected Route:", 0, 1, 0, startPage);
          PrintToController("Confirm  /  Back", 0, 2, 0, startPage);

          switch (selectedRoute) {
            case 1:
              PrintToController("        Skills", 0, 1, 0, startPage);
              break;
            case 2:
              PrintToController("       Offence", 0, 1, 0, startPage);
              break;
            case 3:
              PrintToController("       Defence", 0, 1, 0, startPage);
              break;
          }
        }
        break;
    }

    globalTimer++;  // timer for print function and emergency killswitch
    delay(tickDeltaTime);
  }

  FlystickArmM.set_brake_mode(E_MOTOR_BRAKE_COAST);
  PrintToController("Auton Selected: ", selectedRoute, 1, 1, 1);
  globalTimer = 0;
}


#pragma endregion  // Pregame


// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //



void autonomous() {
  // selectedRoute = 3;
  autonPrinting();

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
      AdjustFlystick(false, true);
    }  // manages the height of the flystick arm

    float inerHeading = (Inertial.get_heading() > 180) ? (-1 * (360 - Inertial.get_heading())) : Inertial.get_heading();

    bool isCurrStepComplete = AutonPID(true);

    if (autonStep > totalNumOfCommands) {  // kills the program if auton route is complete

      while (true) {
        FullDrive.move_velocity(0);
        PrintToController("Out of bounds", 0, 0, 1, 1);

        globalTimer++;
        delay(tickDeltaTime);
      }

    } else if ((MainControl.get_digital_new_press(DIGITAL_X) || isCurrStepComplete) && (timeSincePoint(stepChangeTimeStamp) > endDelayTimeStamp)) {
      autonStep++;

      stepChangeTimeStamp = globalTimer;
      prevArmPos = flystickArmPos;
      prevFlywheelSpeed = flywheelSpeed;

      ReadAutonStep();
    }

    if (isPrintingList[2]) {  // [2] AutRoute - 1
      int startingPage = pageRangeFinder(2);

      PrintToController("Heading: ", inerHeading, 4, 1, startingPage);
      PrintToController("StepDone?: ", isCurrStepComplete, 1, 0, startingPage);
      PrintToController("DelayDone?: ", (globalTimer - endDelayTimeStamp), 3, 2, startingPage);
    }  // diagnostics section

    globalTimer++;
    delay(tickDeltaTime);
  }
}



// // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // // //



void opcontrol() {
  // competition_initialize();
  // autonomous();

  // tunePID();
  // tuneDrive();



  userControlPrinting();

  flywheelFWD = true;

  flywheelOn = false;

  while (true) {
    DrivingControl(true);
    WingsControl();
    FlystickControl(false);
    AdjustFlystick(false, true);
    lcdControl();

    globalTimer++;
    delay(tickDeltaTime);
  }
}