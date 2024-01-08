#pragma region BoringVexConfigStuff

#include "main.h"
#include <stdlib.h> //neccessary for std::[commands]
#include <sstream>  //neccessary for... logic
#include <math.h>   //neccessary for functions like abs() and round()
#include <string>   //neccessary for... using strings :sob:
#include <cstring>

#include "robot-config.h" //importing the motors and whatnot

using namespace std;

#pragma endregion

#pragma region NotesMaybeReadMe

// here to document weird quirks of V5, VSCode, or this particular program

/*
Sometimes things just break, like the abs() function was demanding 0 args. Restarting the program fixed this
Strings do not work in vex without external library shenanigans
*/

#pragma endregion

#pragma region GlobalVars

const float Pi = 3.14159265358;
const float e = 2.71828182845;

int mStartPosL;
int mStartPosR;

int globalTimer = 0;
const int timerTickRate = 50; //the number of 'ticks' in one second
const int tickDelay = 1000 / timerTickRate;
const int minPrintingDelay = 3; //std::ceil(50 / tickDelay);

const float degPerCM = (360 * 2) / (4.1875 * Pi * 2.54); // # of degrees per centimeter = 360 / (2Pir" * 2.54cm/")

int maxFlywheelSpeed = 39; //flywheel speed as a percent
int flystickArmPos = 0; //flystick starts at kickstand position

int lastUpTimestamp = 0;
int lastDownTimestamp = 0; 
int lastSpinTimestamp = 0;

float defaultArmPos;

int maxflystickArmPos = 5;

int tabVar = 1;

#pragma endregion

#pragma region HelperFunctions //unit conversions and whatnot

const char* toChar(std::string string) {
  return string.c_str();
}

int toInt(float val) {
  return val;
}

int timeSincePoint (int checkedTime) {
  return checkedTime < globalTimer ? (globalTimer - checkedTime) : -1; //returns -1 if checkedtime is in the future
}

//i have no idea what im doing
float AccelSmoothingFunc(float input, float x) { //takes a given point from -1 to 1 and returns a corresponding value from a smooth curve
  const float modifier = (2.5 / sqrt(2 * Pi)) * powf(e, (-0.5 * pow(((2.5 * x) - 2.5), 2)));
  return x >= (timerTickRate / 2) ? (modifier * input) : input;
} //function graphed here: https://www.desmos.com/calculator/rwlduqosuy

//  /  /  /  /  /  /  /  /  /

static bool IsWithinRange<T>(this T number, T rangeStart, T rangeEnd) where T : IComparable<T> {
  return number.CompareTo(rangeStart) >= 0 && number.CompareTo(rangeEnd) <= 0;
}

void PrintToController(std::string prefix, float data, int row, int page) {
  if (globalTimer % 11 == 0) { //refresh the screen every 11 ticks
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

////// / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

#pragma region AutonPID //the code behind the autonomous Proportional Integral Derivative controller

///// Control Variables //////

bool twoStickMode = true; // toggles single or float stick drive
int deadband = 12;         // if the controller sticks are depressed less than deadband%, input will be ignored (to combat controller drift)
float RCTurnDamping = 0.65;


  #pragma region PIDVariables

  // control variables
  bool drivePIDIsEnabled = false;
  bool autonPIDIsEnabled = true;

  int desiredDist
  ; // temporary
  int desiredHeading;

//////       TUNING INSTRUCTIONS       //////
/*
1. set all tuning values to 0
2. experiment with the P multiplier until you can drive a precise distance with slight oscilations at the end
3. experiment with the D multiplier until the oscilations slowly die out
4. experiment with the I multiplier until oscilations do not occur
*/

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

  float errorProportionalL;      // reported value - desired value = position
  float previousErrorL = 0; // position from last loop
  float errorDerivativeL;   //(error - prevError)
  float errorIntegralL;

  float lateralPower = 0;

  // Storage variables for Rotational (turning) PID

  float errorProportionalR;      // reported value - desired value = position
  float previousErrorR = 0; // position from last loop
  float errorDerivativeR;   //(error - prevError)
  float errorIntegralR;

  float rotationalPower = 0;

  // the second set of "rotational" storage vars above are technically useless, as the code executes
  // in such a way that only one set of vars is needed to produce both outputs. however, readability is nice
  // change if memory ever becomes an issue


  #pragma endregion //end of PID variable declaration

////// / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

const float autonDriveMult = 1.0;
int stuckTimeStamp = 0;
int avgMotorPosition = 0;

bool AutonPID() {

  if (autonPIDIsEnabled) {

    //sets heading from -180 < h < 180, meaning we turn the correct direction from error
    float heading = (Inertial.get_heading() > 180) ? (-1 * (360 - Inertial.get_heading())) : Inertial.get_heading();
  
    ///////////////////////////////////////
    //////        Lateral PID        //////
    ///////////////////////////////////////
    
    avgMotorPosition = ((RDriveFrontM.get_position()) + (LDriveFrontM.get_position())) / 2;

    errorProportionalL = lP * (desiredDist - avgMotorPosition); // proportional error
    errorDerivativeL = lD * (errorProportionalL - previousErrorL);       // derivative of error

    // filters out the integral at short ranges (no I if |error| < constant lower limit, eg. 10cm),
    // allowing it to be useful when needed without overpowering other elements
    if (fabs(errorProportionalL) > integralBoundL) { 
      errorIntegralL += lI * (errorProportionalL);
    }
    else {
      errorIntegralL = 0;
    }

    lateralPower = (errorProportionalL + errorDerivativeL + errorIntegralL) * lOutput;

    ///////////////////////////////////////
    //////      Rotational PID       //////
    ///////////////////////////////////////

    int avgBotRotation = heading; //LDriveFrontM.get_position() - RDriveFrontM.get_position();

    errorProportionalR = rP * (desiredHeading - avgBotRotation);   // proportional error
    errorDerivativeR = rD * (errorProportionalR - previousErrorR); // derivative of error

    // filters out the integral at short ranges (no I if |error| < constant lower limit, eg. 10cm),
    // allowing it to be useful when needed without overpowering other elements
    if (fabs(errorProportionalR) > integralBoundR) { 
      errorIntegralR += rI * (errorProportionalR);
    }
    else {
      errorIntegralR = 0;
    }

    rotationalPower = (errorProportionalR + errorDerivativeR + errorIntegralR) * rOutput;

    ///////////////////////////////////////
    //////        ending math        //////
    ///////////////////////////////////////

    //PrintToController("LOutput: %d", (lateralPower + rotationalPower), 1, 1); //lateralPower + rotationalPower
    //PrintToController("ROutput: %d", (lateralPower - rotationalPower), 2, 1);

    LDrive.move_velocity(autonDriveMult * (lateralPower + rotationalPower));
    RDrive.move_velocity(autonDriveMult * (lateralPower - rotationalPower));
    
    //stuckTimeStamp += ((errorProportionalL == previousErrorL) && (errorProportionalR == previousErrorR)) ? 1 : -1;

    previousErrorL = errorProportionalL;
    previousErrorR = errorProportionalR;
  }

  return (fabs(errorProportionalL) <= (3 * degPerCM) && fabs(errorProportionalR) <= 1.5) ? true : false; //return true if done movement
}

#pragma endregion // end of PID block

////// / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

#pragma region ActualCompetitionFunctions

  #pragma region AutonFunctions //Functions for autonomous control

float previousErrorF = 0; //previous error variable for the flystick arm

bool allowAdjust = false;

void AdjustFlystick(bool move) {

  float fP = 0.6;
  float fD = 0.8;
  float fO = 1.5;

  int deadzoneF = 0.3;

  //responsible for changing/maintaining the position of the flystick using a PD controller
  //the integral men live in my walls

  int desiredRotation;
  float armPos = ArmRot.get_angle() / 100;

  if (flystickArmPos == 0) { //kills the function if the arm is in kickstand mode, or out of bounds
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

  if (!move) { //return the function before the arm is allowed to move to prevent
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

vector<int> ReadBopIt() {

  int armInput = (ArmRot.get_angle() / 10000);
  
  int wheelDelta = abs(FlywheelM.get_position - prevWheelPos);

  int wheelInput;
  
  if (abs(wheelDelta).IsWithinRange(25, 75) && timeSincePoint(bopItSelectTimeStamp) > 10) {
    wheelInput = (wheelDelta == abs(wheelDelta)) ? 1 : -1;
    bopItSelectTimeStamp = globalTimer;
  } else {
    wheelInput = 0;
  }
  
  prevWheelPos = (globalTimer % 5 == 0) ? (FlywheelM.get_position) : prevWheelPos;

  return {armInput, wheelInput};
}

// structure containing all neccessary data for an autonomous command
struct AutonCommand {
  const float desiredDistInCM;
  const float desiredHeading;
  const int armPos;
  const int flywheelSpeed;
  const bool wingsOut;
  const int endDelay;

  const int targetSpeed;
};

////// / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /


#pragma endregion // end of AutonFunctions

  #pragma region UserControlFunctions //handles all functions involving user input

  int prevXVal = 0;
  int prevYVal = 0;
  int XAccelTimeStamp = 0;
  int YAccelTimeStamp = 0;

  int driveMult = 5.5;

  bool driveReversed = false;
  int reverseDrive = 1;

  void DrivingControl(int8_t printingPage) { //resoponsible for user control of the drivetrain

    if (MainControl.get_digital_new_press(DIGITAL_Y)) {
      driveReversed = !driveReversed;
      LDrive.set_reversed(!driveReversed);
      RDrive.set_reversed(driveReversed);

      reverseDrive = (reverseDrive >= 0) ? -1 : 1;
    }

    //Y is forward/back, X is left/right

    //if the controller is in 2 stick mode axis 4 is responsible for turning, axis 1 if not
    int XStickPos = twoStickMode ? (MainControl.get_analog(E_CONTROLLER_ANALOG_RIGHT_X) * reverseDrive) : (MainControl.get_analog(E_CONTROLLER_ANALOG_LEFT_X) * reverseDrive);

    float YStickPos = MainControl.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);

    //implementing a slow initial acceleration for precision movements

    if (abs(prevYVal <= deadband && (abs(YStickPos) - abs(prevYVal) >= 25))) {  
      //if the Y stick's position has changed drastically from zero, set a marker for the acceleration function
      YAccelTimeStamp = globalTimer;
    }
    if (abs(prevXVal <= deadband && (abs(XStickPos) - abs(prevXVal) >= 20))) {  
      //if the X stick's position has changed drastically from zero, set a marker for the acceleration function
      XAccelTimeStamp = globalTimer;
    }

    int YStickPercent = YStickPos / 1.27; //AccelSmoothingFunc((YStickPos), (globalTimer - YAccelTimeStamp));
    int XStickPercent = XStickPos / 1.27; //AccelSmoothingFunc((XStickPos) * RCTurnDamping, (globalTimer - XAccelTimeStamp));

    int outputL = 12 + driveMult * (YStickPercent + XStickPercent) * (fabs((YStickPercent + XStickPercent) / 100));
    int outputR = 12 + driveMult * (YStickPercent - XStickPercent) * (fabs((YStickPercent - XStickPercent) / 100));


    if ((abs(YStickPos) + abs(XStickPos)) >= deadband) {
      LDrive.move_velocity(outputL);
      RDrive.move_velocity(outputR);
    } else {
      LDrive.move_velocity(0);
      RDrive.move_velocity(0);
    }
    
    PrintToController("LDrive: %d", -1, 1, printingPage);
    PrintToController("RDrive: %d", -1, 2, printingPage);

    PrintToController("YOut %d", YStickPercent, 1, printingPage + 1);
    PrintToController("XOut %d", XStickPercent, 2, printingPage + 1);

    PrintToController("Funcval %d", AccelSmoothingFunc(1, 100 * (globalTimer - YAccelTimeStamp)), 1, printingPage + 2);
    PrintToController("FuncX %d", (globalTimer - YAccelTimeStamp), 2, printingPage + 2);

  prevXVal = XStickPos;
  prevYVal = YStickPos;
}  

bool flywheelFWD = true; //flywheel is inherently inverted so this reads as the opposite of what it does
bool flywheelOn = false;

void FlystickControl(int8_t printingPage) { //done
  //responsible for selecting the elevation of the flystick

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

void WingsControl() { //done

  if (MainControl.get_digital_new_press(DIGITAL_L2)) {
    LWingLockedOut = !LWingLockedOut;
    WingPL.set_value(LWingLockedOut);
  }

  if (MainControl.get_digital_new_press(DIGITAL_R2)) {
    RWingLockedOut = !RWingLockedOut;
    WingPR.set_value(RWingLockedOut);
  }
}

  #pragma endregion // end of UserControlFunctions

#pragma endregion // end of Bot controlling functions


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#pragma region ExecutionBlock //the region in which the above code is executed during a competition

  void initialize() {

  WingPL.set_value(false);
  WingPR.set_value(false); 

  FullDrive.tare_position();
  FullDrive.set_zero_position(0);

  mStartPosL = LDriveMidM.get_position();
  mStartPosR = RDriveMidM.get_position();

  FullDrive.set_brake_modes(E_MOTOR_BRAKE_COAST);

  FlywheelM.set_brake_mode(E_MOTOR_BRAKE_COAST);

  FlystickArmM.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  ArmRot.reverse();

  defaultArmPos = ArmRot.get_position() / 100;
  Inertial.reset(true);
}

#pragma region BoringBlocks

  /**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
  void disabled() {}

  /**
   * Runs after initialize(), and before autonomous when connected to the Field
   * Management System or the VEX Competition Switch. This is intended for
   * competition-specific initialization routines, such as an autonomous selector
   *   on the LCD.
   * 
   * This task will exit when the robot is enabled and autonomous or opcontrol
   * starts.
  */

  #pragma endregion

  int selectorStage = 0;
  int selectedRoute = 3;
 
  void competition_initialize() { //auton selector (bop-it!)
    /*
    while ((selectorStage < 2) && (globalTimer < (10 * timerTickRate))) {

      lcdControl();

      switch (selectorStage) {

      case 0:

        if (MainControl.get_digital_new_press(DIGITAL_UP) && selectedRoute < 4) {  selectedRoute++;  }
        if (MainControl.get_digital_new_press(DIGITAL_DOWN) && selectedRoute > 1) {  selectedRoute--;  }
        if (MainControl.get_digital_new_press(DIGITAL_A)) {  selectorStage++;  }

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
          case 1:  PrintToController("        Skills", 0, 1, 1);
            break;
          case 2:  PrintToController("       Offence", 0, 1, 1);
            break;
          case 3:  PrintToController("       Defence", 0, 1, 1);
            break;
        }

        break;
      }

      globalTimer++; //need timer for print function
      delay(tickDelay);
    }


    PrintToController("Auton %d Selected", selectedRoute, 1, 1);
    globalTimer = 0; */
  }

  #pragma region autonRoutes

  int totalNumOfCommands = 50;
  vector <float> autonCommands[50];

  void skillsAuton() {
    //autonCommands[ autonStep ] = {[]}
    //[lateralDistance(cm), rotationalDistance(degrees), flystickArmPos(1-5, 0 = no change), flywheelSpeed(%), wingsOut(bool), delay(seconds)]

    autonCommands[ 0 ] = {0 , 0 , 0 , 0 , 0 , 0};       //null start, copy/paste to make new step and KEEP AS POSITION 0
    autonCommands[ 1 ] = {0 , 0 , 0 , 0 , 0 , 0}; 
    autonCommands[ 2 ] = {0 , 0 , 3 , 100 , 0 , 35};    //Match loads for 35 seconds
    autonCommands[ 3 ] = {20 , 0 , 0 , -1 , 0 , 0};     //Move forward in preperation for turn
    autonCommands[ 4 ] = {0 , 90 , 0 , -1 , 0 , 0};     //90 degree turn right to align robot for triball scoring in net
    autonCommands[ 5 ] = {-65 , 0 , 1 , 0 , 0 , 0};     //push preload triballs towards goal
    autonCommands[ 6 ] = {20 , 0 , 0 , 0 , 0 , 0};      //move back (preparing for 2nd push into goal)
    autonCommands[ 7 ] = {-25 , 0 , 0 , 0 , 0 , 0};     //push triballs into goal
    autonCommands[ 8 ] = {110 , 0 , 0 , 0 , 0 , 0};     //move towardsd wall, triballs are scored, time to try and score triballs in oposite goal
    autonCommands[ 9 ] = {0 , -45 , 0 , 0 , 0 , 0};     //turn to be parallel with the arena, facing our colors low hang bar
    autonCommands[ 10 ] = {200 , 0 , 0 , 0 , 0 , 0};    //move from our side to the other side of the feild
    autonCommands[ 11 ] = {0 , -45 , 0 , 0 , 0 , 0};    //turn towards the net
    autonCommands[ 12 ] = {70 , 0 , 0 , 0 , 0 , 0};     //push triballs into the net
    autonCommands[ 13 ] = {0 , -45 , 0 , 0 , 0 , 0};    //turn to face net
    autonCommands[ 14 ] = {-20 , 0 , 0 , 0 , 0 , 0};    //move back to be in position to push triballs in the side of the net again
    autonCommands[ 15 ] = {45 , 0 , 0 , 0 , 0 , 0};     //push triballs into net
    autonCommands[ 16 ] = {0 , -90 , 0 , 0 , 0 , 0};    //turn away from the net to get into position to push the front
    autonCommands[ 17 ] = {70 , 0 , 0 , 0 , 0 , 0};     //drive to get ahead of goal net
    autonCommands[ 18 ] = {0 , 45 , 0 , 0 , 0 , 0};     //turn to be perpendicular to match load zone
    autonCommands[ 19 ] = {15 , 0 , 0 , 0 , 0 , 0};     //position ourselves more in front of the net
    autonCommands[ 20 ] = {0 , 112.5 , 0 , 0 , 0 , 0};  //turn to face front of net
    autonCommands[ 21 ] = {60 , 0 , 0 , 0 , 0 , 0};     //push triballs into front of net
    //samich yummmmmmmmmy

    totalNumOfCommands = 21;
  }

  void offenceAuton() { //starting on the enemy side of the field (no match loading)
    //autonCommands[ autonStep ] = {[]}
    //[lateralDistance(cm), rotationalDistance(degrees), flystickArmPos(1-5, 0 = no change), flywheelSpeed(%), wingsOut(bool), delay(seconds)]

    autonCommands[ 0 ] = {0 , 0 , 0 , 0 , 0 , 0};       //null start, copy/paste to make new step and KEEP AS POSITION 0
    autonCommands[ 1 ] = {-35 , 0, 0 , 0 , 0 , 0};      //drive along match load bar
    autonCommands[ 2 ] = {0 , -10, 0 , 0 , 0 , 0};      //turn slightly away from wall
    autonCommands[ 3 ] = {-45 , 0, 0 , 0 , 0 , 0};      //ram preload into net
    autonCommands[ 4 ] = {60 , 0, 0 , 0 , 0 , 0};       //drive back to halfway along match load bar
    autonCommands[ 5 ] = {0, 80, 0 , 0 , 0 , 0};        //turn to face away from match load bar
    autonCommands[ 6 ] = {10 , 0 , 5 , 80 , 0 , 0};     //lower arm and spin flywheel
    autonCommands[ 7 ] = {-30 , 0 , 0 , -1 , 0 , 0};    //reverse into corner tribal, launching it out of corner
    autonCommands[ 8 ] = {0 , 100 , 3 , 0 , 0 , 0};     //stop flystick and turn towards horizontal climb bar
    autonCommands[ 9 ] = {-35 , 0 , 0 , 0 , 0 , 0};     //drive to entrance of climb bar corridor
    autonCommands[ 10 ] = {0 , 45 , 0 , 0 , 0 , 0};     //turn to face horizontal climb bar
    autonCommands[ 11 ] = {-90 , 0 , 0 , 0 , 0 , 0};    //drive until arm is touching horizontal bar

    totalNumOfCommands = 0;
  }

  void defenceAuton() { //starting on the team side of the field (match loading)
    //autonCommands[ autonStep ] = {[]}
    //[lateralDistance(cm), rotationalDistance(degrees), flystickArmPos(1-5, 0 = no change), flywheelSpeed(%), wingsOut(bool), delay(seconds)]

    autonCommands[ 0 ] = {0 , 0 , 0 , 0 , 0 , 0};     //null start, copy/paste to make new step and KEEP AS POSITION 0
    autonCommands[ 1 ] = {0 , 0 , 0 , 0 , 0 , 1};
    autonCommands[ 2 ] = {-35 , 0, 0 , 0 , 0 , 0};    //drive along match load bar
    autonCommands[ 3 ] = {0 , 10, 0 , 0 , 0 , 0};     //turn slightly away from wall
    autonCommands[ 4 ] = {-45 , 0, 0 , 0 , 0 , 0};    //ram preload into net
    autonCommands[ 5 ] = {60 , 0, 0 , 0 , 0 , 0};     //drive back to halfway along match load bar
    autonCommands[ 6 ] = {0, -90, 0 , 0 , 0 , 0};     //turn to face away from match load bar
    autonCommands[ 7 ] = {20 , 0 , 5 , 80 , 0 , 2};   //lower arm and spin flywheel
    autonCommands[ 8 ] = {-40 , 0 , 0 , -1 , 0 , 2};  //reverse into corner tribal, launching it out of corner
    autonCommands[ 9 ] = {0 , -100 , 3 , 0 , 0 , 0};  //stop flystick and turn towards horizontal climb bar
    autonCommands[ 10 ] = {-35 , 0 , 0 , 0 , 0 , 0};  //drive to entrance of climb bar corridor
    autonCommands[ 11 ] = {0 , -45 , 0 , 0 , 0 , 0};  //turn to face horizontal climb bar
    autonCommands[ 12 ] = {-90 , 0 , 0 , 0 , 0 , 0};  //drive until arm is touching horizontal bar

    totalNumOfCommands = 12;
  }

  #pragma endregion

  #pragma region autonVars

    const int stepChangeCooldown = timerTickRate / 3; //sets the minimum delay between auton steps, default 0.25 seconds
    int stepChangeTimeStamp = 0; //stores the time of the last step change

    //initializing data variables (used to track details of each step)
    int autonStep = 0; //tracks which step of the auton the program is on
    int flywheelSpeed = 0;
    int prevFlywheelSpeed = 0;
    int prevArmPos = 0;
    bool wingsOut = false;
    int endDelayTimeStamp = 0; //holds the minimum objective time at which the current step can end 
    //                          (ex. flywheel spinning step lasts a min of 30 sec)

    void ReadAutonStep() {

      vector <float> currentCommand = autonCommands[autonStep];

      desiredDist += currentCommand.at(0) * degPerCM;
      desiredHeading += currentCommand.at(1);

      flystickArmPos = (currentCommand.at(2) == 0) ? prevArmPos : currentCommand.at(2);
      flywheelSpeed = (currentCommand.at(3) == -1) ? prevFlywheelSpeed : currentCommand.at(3);

      endDelayTimeStamp = ((currentCommand.at(5) * timerTickRate) > stepChangeCooldown) ? (currentCommand.at(5) * timerTickRate) : stepChangeCooldown;

      //extends/retracts both wings depending on input
      wingsOut = currentCommand.at(4);
      WingPL.set_value(wingsOut);
      WingPR.set_value(wingsOut);
}

#pragma endregion

  void autonomous() {

    //selectedRoute = 3;

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
     
      lcdControl(); //allows the LCD screen to show multiple pages of diagnostics, press left/right arrows to change pages
      FlywheelM.move_velocity(flywheelSpeed * 2); //spins the flywheel at the desired speed (input as a percent)
      if ((ArmRot.get_angle() / 100) < 306) { AdjustFlystick(true);  } //manages the height of the flystick arm

      float inerHeading = (Inertial.get_heading() > 180) ? (-1 * (360 - Inertial.get_heading())) : Inertial.get_heading();
      bool stepPIDIsComplete = AutonPID();

        #pragma region diagnostics

        //reports values relating to the movement of the flystick on page 1
        PrintToController("Step: %d", autonStep, 0, 1);
        PrintToController("Timer: %d", globalTimer, 1, 1);
        PrintToController("Route: %d", selectedRoute, 2, 1);

        //reports values relating to the overall progression of the autonomous on page 2
        PrintToController("Timer: %d", globalTimer, 0, 2);
        PrintToController("Wing??: %d", wingsOut, 1, 2);
        PrintToController("DelayDone?: %d", (globalTimer - endDelayTimeStamp), 2, 2);

        //reports values relating to the PID movement on pages 3 & 4
        PrintToController("Complete?: %d", stepPIDIsComplete, 0, 3);
        PrintToController("Head: %d", inerHeading, 1, 3);
        PrintToController("ErrorR: %d", errorProportionalR, 2, 3); 

        PrintToController("Complete?: %d", stepPIDIsComplete, 0, 4);
        PrintToController("TargDist: %d", desiredDist, 1, 4);
        PrintToController("ErrorL: %d",(RDriveMidM.get_position() + LDriveMidM.get_position()) / 2 , 2, 4); 

        #pragma endregion

      if (autonStep > totalNumOfCommands) { //kills the program if auton route is complete
        while (true) {
          FullDrive.move_velocity(0);
          PrintToController("Out of bounds", 0, 1, 1);

          globalTimer++;
          delay(tickDelay);
        }
      } else if ((MainControl.get_digital_new_press(DIGITAL_X) || stepPIDIsComplete ) && (timeSincePoint(stepChangeTimeStamp) > endDelayTimeStamp)) { //MainControl.get_digital_new_press(DIGITAL_X) || stepPIDIsComplete 
        
        autonStep++;

        stepChangeTimeStamp = globalTimer;
        prevArmPos = flystickArmPos;
        prevFlywheelSpeed = flywheelSpeed;

        ReadAutonStep();
      }

      globalTimer++;
      delay(tickDelay);
    }
  }

  #pragma region debugFunctions

  float adjustFactor = 0.1; //the amount PID variables change by during manual tuning

  void tunePID() {
    
    desiredDist = 0 * degPerCM;
    desiredHeading = 0;

    lOutput = 1.0;
    rOutput = 2.7;

    while (true) {

      lcdControl();

      if (MainControl.get_digital_new_press(DIGITAL_B)) {
        rP += adjustFactor;
      }
      if (MainControl.get_digital_new_press(DIGITAL_Y)) {
        rD += adjustFactor;
      }
      if (MainControl.get_digital_new_press(DIGITAL_A)) {
        rI += adjustFactor;
      }

      if (MainControl.get_digital_new_press(DIGITAL_X)) {
        adjustFactor *= -1;
      }

      PrintToController("PVar: %d", rP * 10, 0, 2);
      PrintToController("IVar: %d", rI * 10, 1, 2);
      PrintToController("DVar: %d", rD * 10, 2, 2);

      AutonPID();

      if (globalTimer % (6 * timerTickRate) < (3 * timerTickRate)) { //cycles between states to allow for real time tuning
        //comment out either heading or dist to test the other

        //desiredDist = 50 * degPerCM;
        desiredHeading = 90;
      } else {
        //desiredDist = -50 * degPerCM;
        desiredHeading = -90;
      }

      globalTimer++;
      delay(tickDelay);
    }
  }

  void minAuton() { // minimum viable "turn on flystick" auton

    flystickArmPos = 3;

    while (true) {
      AdjustFlystick(true);
      FlywheelM.move_velocity(-180);
    }
  }

  #pragma endregion

/**
 * Runs the user control code
 * If no competition control is connected, this function will run immediately
 * following initialize().
 */
void opcontrol() {

  //competition_initialize();
  //autonomous();                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               autonomous();

  flywheelFWD = true;

  flywheelOn = false;
  
  //flystickArmPos = 3; //ACTUAL DRIVE CODE, UNCOMMENT IF UPLOADING DRIVE

	while (true) {  

    DrivingControl(1);
    WingsControl();
    FlystickControl(-1);
    AdjustFlystick(true);
    lcdControl();
    
    globalTimer++;
    delay(tickDelay);
  }
}