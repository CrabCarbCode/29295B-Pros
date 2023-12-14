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

int globalTimer = 0;
const int timerTickRate = 50; //the number of 'ticks' in one second
const int tickDelay = 1000 / timerTickRate;
const int minPrintingDelay = 3; //std::ceil(50 / tickDelay);

const float degPerCM = (360 * 2) / (4.1875 * Pi * 2.54); // # of degrees per centimeter = 360 / (2Pir" * 2.54cm/")

int maxFlywheelSpeed = 39; //flywheel speed as a percent
int flystickPos = 0; //flystick starts at kickstand position

int lastUpTimestamp = 0;
int lastDownTimestamp = 0; 
int lastSpinTimestamp = 0;

float defaultArmPos;

int maxFlystickPos = 5;

int tabVar = 1;

#pragma endregion

#pragma region HelperFunctions //unit conversions and whatnot

const char* toChar(std::string string) {
  return string.c_str();
}

int toInt(float val) {
  return val;
}

void PrintToController(std::string prefix, float data, int row, int page) {
  if (globalTimer % 11 == 0) {
    MainControl.clear();
  }

  if (tabVar == page && (globalTimer % 9 == (row * 3))) {
    MainControl.print(row, 0, prefix.c_str(), toInt(data));
  }
}
//  /  /  /  /  /  /  /  /  /
int timeSincePoint (int checkedTime) {
  return checkedTime < globalTimer ? (globalTimer - checkedTime) : -1; //returns -1 if checkedtime is in the future
}

//i have no idea what im doing
float AccelSmoothingFunc(float input, float x) {
  const float modifier = (2.5 / sqrt(2 * Pi)) * powf(e, (-0.5 * pow(((2.5 * x) - 2.5), 2)));
  return x >= (timerTickRate / 2) ? (modifier * input) : input;
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

int AutonPID() {

  if (autonPIDIsEnabled) {

    //sets heading from -180 < h < 180, meaning we turn the correct direction from error
    float heading = (Inertial.get_heading() > 180) ? (-1 * (360 - Inertial.get_heading())) : Inertial.get_heading();

  
    ///////////////////////////////////////
    //////        Lateral PID        //////
    ///////////////////////////////////////

    int avgMotorPosition = (RDriveFrontM.get_position() + LDriveFrontM.get_position()) / 2;

    errorProportionalL = lP * (desiredDist - avgMotorPosition); // proportional error
    errorDerivativeL = lD * (errorProportionalL - previousErrorL);       // derivative of error

    // filters out the integral at long ranges (if |error| < constant upper limit, eg. 10cm),
    // allowing it to be useful when needed without overpowering other elements
    if (fabs(errorProportionalL) < integralBoundL) { 
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

    // filters out the integral at long ranges (if |error| < constant upper value, eg. 10cm),
    // allowing it to be useful when needed without overpowering other elements
    if (fabs(errorProportionalR) < integralBoundR) { 
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

    previousErrorL = errorProportionalL;
    previousErrorR = errorProportionalR;
  }

  return (fabs(errorProportionalL) <= (1 * degPerCM) && fabs(errorProportionalR) <= 0.75) ? true : false; //return true if done movement
}

#pragma endregion // end of PID block

////// / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

#pragma region ActualCompetitionFunctions

#pragma region AutonFunctions //Functions for autonomous control

float previousErrorF = 0; //previous error variable for the flystick arm

bool allowAdjust = false;

void AdjustFlystick() {

  float fP = 0.6;
  float fD = 0.8;
  float fO = 1.5;

  int deadzoneF = 0.3;

  //responsible for changing/maintaining the position of the flystick using a PD controlelr
  //no Integral because y'know

  int desiredRotation;
  float armPos = ArmRot.get_angle() / 100;

  switch (flystickPos) {
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

  if (flystickPos == 0) {
    return;
  }

  if (abs(currentErrorF) >= deadzoneF) {
    FlystickArmM.move_velocity(fO * (currentErrorF + errorDerivativeF));
  } else {
    FlystickArmM.move_velocity(0);
  }

  previousErrorF = currentErrorF;
}

// structure containing all neccessary data for an autonomous command
struct AutonCommand {
  float desiredDistInCM;
  float desiredHeading;

  float targetSpeed;
  int endDelay;
};

// struct AutonCommands CommandList[totalNumOfCommands]; //initializes the list of autonomous commands, must be populated in active code (preauton)
const int totalNumOfCommands = 2; // change depending on the total number of "steps" in your autonomous


////// / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /


#pragma endregion // end of AutonFunctions

  #pragma region UserControlFunctions

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

    int outputY = YStickPos / 127; //AccelSmoothingFunc((YStickPos), (globalTimer - YAccelTimeStamp));
    int outputX = XStickPos / 155; //AccelSmoothingFunc((XStickPos) * RCTurnDamping, (globalTimer - XAccelTimeStamp));


    if ((abs(YStickPos) + abs(XStickPos)) >= deadband) {
      LDrive.move_velocity(driveMult * (outputY + outputX) * (fabs((outputY + outputX) / 127)) + 1);
      RDrive.move_velocity(driveMult * (outputY - outputX) * (fabs((outputY - outputX) / 127)) + 1);
    } else {
      LDrive.move_velocity(0);
      RDrive.move_velocity(0);
    }
    
    PrintToController("LDrive: %d", (driveMult * (outputY + outputX) * (fabs((outputY + outputX) / 127)) + 1), 1, printingPage);
    PrintToController("RDrive: %d", (driveMult * (outputY + outputX) * (fabs((outputY - outputX) / 127)) + 1), 2, printingPage);

    PrintToController("YOut %d", outputY, 1, printingPage + 1);
    PrintToController("XOut %d", outputX, 2, printingPage + 1);

    PrintToController("Funcval %d", AccelSmoothingFunc(1, 100 * (globalTimer - YAccelTimeStamp)), 1, printingPage + 2);
    PrintToController("FuncX %d", (globalTimer - YAccelTimeStamp), 2, printingPage + 2);

  prevXVal = XStickPos;
  prevYVal = YStickPos;
}  

bool FlywheelReversed = false;
bool flywheelOn = false;

void FlystickControl(int printingRow) { //done
  //responsible for selecting the elevation of the flystick

  int cooldown = 3;

  if (MainControl.get_digital_new_press(DIGITAL_UP) && (globalTimer - lastUpTimestamp >= cooldown) && flystickPos < maxFlystickPos) {
    flystickPos++;
    lastUpTimestamp = globalTimer;
  }

  if (MainControl.get_digital_new_press(DIGITAL_DOWN) && (globalTimer - lastDownTimestamp >= cooldown) && flystickPos > 1) {
    flystickPos--;
    lastDownTimestamp = globalTimer;
  }

  if (MainControl.get_digital_new_press(DIGITAL_B)) {
    FlywheelReversed = !FlywheelReversed;
    FlywheelM.set_reversed(FlywheelReversed);
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


int mStartPos;

#pragma region ExecutionBlock //the region in which the above code is executed during a competition
  /**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
  void initialize() {

  WingPL.set_value(false);
  WingPR.set_value(false); 

  FullDrive.tare_position();
  FullDrive.set_zero_position(0);

  mStartPos = LDriveFrontM.get_position();

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
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
  void competition_initialize() {}

  /**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.

\ */

  #pragma endregion

  #pragma region autonRoutes

  vector <float> autonCommands[50];

  void skillsAuton(bool red) {

    int mult = red ? 1 : -1;

    //autonCommands[  ] = { , * mult};

    autonCommands[ 0 ] = {0, 0 * mult};
    autonCommands[ 1 ] = {0, 0 * mult};
    /*autonCommands[ 2 ] = {0 , 45 * mult};
    autonCommands[ 3 ] = {40 , 0 * mult};*/

  }

  void offenceAuton(bool red) {

    int mult = red ? 1 : -1;

    //autonCommands[  ] = { , * mult};

  }

  void defenceAuton(bool red) {

    int mult = red ? 1 : -1;

    //autonCommands[  ] = { , * mult};

  }

  #pragma endregion

  int stepChangeCooldown = timerTickRate / 4; //sets the minimum delay between auton steps, default 0.25 seconds
  int stepChangeTimeStamp = 0; //stores the time of the last step change

  void autonomous() {

    skillsAuton(true);

    int autonStep = 0; //tracks which step of the auton the program is on
    int shootOrb = 1; //set to -1 for non-shooting autons
    

    while (true) {

      if (autonStep == shootOrb) { //handles the shooting stage of autonomous
        flystickPos = 3;

        while (globalTimer <= (15 * timerTickRate)) { //objective timer, starts relative to skills run start

          //speen
          AdjustFlystick();
          FlywheelM.move_velocity(-180);

          globalTimer++;
          delay(tickDelay);
        }

        autonStep++;
        flywheelOn = false;
        flystickPos = 1;
      }

      lcdControl(); //allows the LCD screen to show multiple pages of diagnostics, press left/right arrows to change pages
      AdjustFlystick(); //manages the height of the flystick arm

      //diagnostics, not neccessary
      float inerHeading = (Inertial.get_heading() > 180) ? (-1 * (360 - Inertial.get_heading())) : Inertial.get_heading();

      PrintToController("Step: %d", autonStep, 0, 1);
      PrintToController("DesDist: %d", desiredDist, 1, 1);
      PrintToController("DesHead: %d", desiredHeading, 2, 1);

      PrintToController("Complete?: %d", AutonPID(), 0, 2);
      PrintToController("Head: %d", inerHeading, 1, 2);
      PrintToController("ErrorR: %d", errorProportionalR, 2, 2); 

      PrintToController("Complete?: %d", AutonPID(), 0, 3);
      PrintToController("TargDist: %d", desiredDist, 1, 3);
      PrintToController("ErrorL: %d", errorProportionalL, 2, 3); 


      if ((MainControl.get_digital_new_press(DIGITAL_X)  || AutonPID() == 1) && (timeSincePoint(stepChangeTimeStamp) >= stepChangeCooldown)) { //MainControl.get_digital_new_press(DIGITAL_X) || 
        autonStep++;

        stepChangeTimeStamp = globalTimer;

        vector <float> currentCommand = autonCommands[autonStep];
        desiredDist += currentCommand.at(0) * degPerCM;
        desiredHeading += currentCommand.at(1);

      } else if (autonStep >= 4) {
        return;
      }

      globalTimer++;
      delay(tickDelay);
    }
  }



  float adjustFactor = 0.1; //the amount PID variables change by during manual tuning

  void tunePID() {
    
    desiredDist = 0 * degPerCM;
    desiredHeading = 0;

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

  void minAuton() {

    flystickPos = 3;

    while (true) {
      AdjustFlystick();
      FlywheelM.move_velocity(-180);
    }
  }

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {

  //minAuton();

  //tunePID();

  //autonomous(); //temporarily running autonomous in usercontrol

  //FlywheelM.set_reversed(false);

  //flywheelOn = false;
  
  /*flystickPos = 3; //ACTUAL DRIVE CODE, UNCOMMENT IF UPLOADING DRIVE

	while (true) {  

    DrivingControl(1);
    WingsControl();
    FlystickControl(-1);
    AdjustFlystick();
    lcdControl();
    
    globalTimer++;
    delay(tickDelay);
  }*/
}