#include "main.h"
#include "stdlib.h" //neccessary for std::[commands]
#include "sstream"  //neccessary for... logic
#include "math.h"   //neccessary for functions like abs() and round()
#include "string"   //neccessary for... using strings :sob:
#include "robot-config.h"

#pragma region BoringVexConfigStuff

#include "robot-config.h"

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Clawbot Competition Template                              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// MainControl          controller
// Drivetrain           drivetrain    1, 10, D
// ClawMotor            motor         3
// ArmMotor             motor         8
// ---- END VEXCODE CONFIGURED DEVICES ----

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
int GlobalTimer = 0;
const double degPerCM = 360 / (4.1875 * Pi) * 2.54; // # of degrees per centimeter = 360 / (2Pir" * 2.54cm/")

#pragma endregion

#pragma region HelperFunctions //unit conversions and whatnot

//stupid fix to a bug in stdlib that prevents the concatonation of strings and other variable types
namespace patch { //instead of std::to_string([var]) its now patch::to_string([var])

    template < typename T > std::string to_string(const T & n)
    {
        std::ostringstream stm ;
        stm << n ;
        return stm.str();
    }
}

void PrintToController(std::string prefix, double data, int row, int column, bool clear) {
  if (clear) {
    MainControl.clear();
  }  
  MainControl.print(row, column, prefix.c_str(), data);
}
 
//broken 
/*void PrintToController(std::string prefix, double data, int row, int column, bool clear) {
  if (clear) {
    screen.clear();
  }  
  screen::print(row, column, prefix.c_str(), data);
}*/

// Math Functions
int absInt(int val) { // Convert integers to their absolute value
  return val < 0 ? -val : val;
}

double absDouble(double val) { // Convert integers to their absolute value
  return val < 0 ? -val : val;
}

float absFloat(float val) { // Convert integers to their absolute value
  return val < 0 ? -val : val;
}

int timerInPID;

#pragma endregion

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	Inertial.reset();

  LDrive.set_zero_position(0);
  RDrive.set_zero_position(0);

}

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
void autonomous() {}

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

  //Y is forward/back, X is left/right

	while (true) {

  // if the controller is in 2 stick mode axis 4 is responsible for turning, axis 1 if not
  int turnAxisValue = twoStickMode ? MainControl.get_analog(E_CONTROLLER_ANALOG_RIGHT_X) : MainControl.get_analog(E_CONTROLLER_ANALOG_LEFT_X);

  PrintToController("XAxisPos: ", MainControl.get_analog(E_CONTROLLER_ANALOG_LEFT_Y), 0, 0, true);

  if (absInt(MainControl.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) + absInt(turnAxisValue) >= deadband)) {

    double YAxisSpeed = MainControl.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
    double turnSpeed = turnAxisValue * RCTurnDamping;

    LDrive.move_velocity(YAxisSpeed + turnSpeed);
    RDrive.move_velocity(YAxisSpeed - turnSpeed);

    }
	}
}




///// Control Variables //////

bool twoStickMode = true; // toggles single or double stick drive
int deadband = 7;         // if the controller sticks are depressed less than deadband%, input will be ignored (to combat controller drift)
double RCTurnDamping = 0.65;

////// / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

#pragma region AutonPID //the code behind the autonomous Proportional Integral Derivative controller

  #pragma region PIDVariables

  // control variables
  bool drivePIDIsEnabled = false;
  bool autonPIDIsEnabled = false;

  int desiredDistInDegrees = 200; // temporary
  int desiredHeading = 0;

//////       TUNING INSTRUCTIONS       //////
/*
1. set all tuning values to 0
2. experiment with the P multiplier until you can drive a precise distance with slight oscilations at the end
3. experiment with the D multiplier until the oscilations slowly die out
4. experiment with the I multiplier until oscilations do not occur
*/

  // tuning coefficients

  double lP = 1.0;
  double lI = 0.0;
  double lD = 0.0;

  double lOutput = 1.0;

  double rP = 1.0;
  double rI = 0.0;
  double rD = 0.0;

  double rOutput = 1.0;

  int integralBoundL = 10;
  int integralBoundR = 10;

  // Storage variables for Lateral (forward/back) PID

  double currentErrorL;      // reported value - desired value = position
  double previousErrorL = 0; // position from last loop
  double errorDerivativeL;   //(error - prevError)
  double errorIntegralL = 0;

  double lateralPower = 0;

  // Storage variables for Rotational (turning) PID

  double currentErrorR;      // reported value - desired value = position
  double previousErrorR = 0; // position from last loop
  double errorDerivativeR;   //(error - prevError)
  double errorIntegralR = 0;

  double rotationalPower = 0;

  // the second set of "turning" storage vars above are technically useless, as the code executes
  // in such a way that only one set of vars is needed to produce both outputs. however, readability is nice
  // change if memory ever becomes an issue

  #pragma endregion //end of PID variable declaration

////// / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

void autonPID() {

  if (autonPIDIsEnabled) {
 
    ///////////////////////////////////////
    //////        Lateral PID        //////
    ///////////////////////////////////////

    int avgMotorPosition = (RDriveFrontM.get_position() + LDriveFrontM.get_position()) / 2;

    currentErrorL = lP * (desiredDistInDegrees - avgMotorPosition); // proportional error
    errorDerivativeL = lD * (currentErrorL - previousErrorL);       // derivative of error

    // filters out the integral at long ranges (if |error| < constant upper limit, eg. 10cm),
    // allowing it to be useful when needed without overpowering other elements

    if (absDouble(currentErrorL) < integralBoundL) { // NEED TO MAKE ABSOLUTE OF CURRENTERROR WORK
      errorIntegralL += lI * (currentErrorL);
    }
    else {
      errorIntegralL = 0;
    }

    lateralPower = (currentErrorL + errorDerivativeL + errorIntegralL) / lOutput;

    ///////////////////////////////////////
    //////      Rotational PID       //////
    ///////////////////////////////////////

    int avgBotRotation = LDriveFrontM.get_position() - RDriveFrontM.get_position();

    currentErrorR = rP * (desiredHeading - avgBotRotation);   // proportional error
    errorDerivativeR = rD * (currentErrorR - previousErrorR); // derivative of error

    // filters out the integral at long ranges (if |error| < constant upper value, eg. 10cm),
    // allowing it to be useful when needed without overpowering other elements

    if (absDouble(currentErrorR) < integralBoundR)
    { 
      errorIntegralR += rI * (currentErrorR);
    }
    else
    {
      errorIntegralR = 0;
    }

    rotationalPower = (currentErrorL + errorDerivativeL + errorIntegralL) / rOutput;

    ///////////////////////////////////////
    //////        ending math        //////
    ///////////////////////////////////////

    PrintToController("Test: ", timerInPID, 0, 0, false);

    timerInPID += 1;

    previousErrorL = currentErrorL;
    previousErrorR = currentErrorR;
    
    delay(20);
  }

  // return 1;
}

#pragma endregion // end of PID block

////// / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

#pragma region ActualCompetitionFunctions

#pragma region AutonFunctions //Functions for autonomous control

// structure containing all neccessary data for an autonomous command
struct AutonCommand {
  double desiredDistInCM;
  double desiredHeading;

  double targetSpeed;
  int endDelay;
};

const int totalNumOfCommands = 10; // change depending on the total number of "steps" in your autonomous

// struct AutonCommands CommandList[totalNumOfCommands]; //initializes the list of autonomous commands, must be populated in active code (preauton)

////// / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

void ExecuteAutonCommands(struct AutonCommand CurrentCommandList[]) {

  const double wheelCircumferance = (4.1875 * Pi) * 2.54;
  const double degPerCM = 360 / wheelCircumferance;

  // 360 degrees = circumferance cm

  for (int i = 0; ++i >= totalNumOfCommands;)
  {

    PrintToController("Executing Step: ", i, 0, 0, true);

    struct AutonCommand CurrentCommand = CurrentCommandList[i];

    // setting the PID's target values for this stage of the autonomous routine
    desiredDistInDegrees = CurrentCommand.desiredDistInCM * degPerCM;
    desiredHeading = CurrentCommand.desiredHeading;

    double speedMult = CurrentCommand.targetSpeed / 100;

    // vex::task auton(autonPID);

    // RotPower is incorporated as a positive on the left side, meaning positive angles make righthand turns

    LDrive.set_voltage_limit(speedMult * (lateralPower + rotationalPower));
    LDrive.set_voltage_limit(speedMult * (lateralPower - rotationalPower));
  }
}

#pragma endregion // end of AutonFunctions

#pragma region UserControlFunctions

void UserControl() {

  
}

#pragma endregion // end of UserControlFunctions

#pragma endregion // end of Bot controlling functions

#pragma region CodeExecution

void autonomous(void) {

  const double degPerCM = 360 / (4.1875 * Pi) * 2.54; // # of degrees per centimeter = 360 / (2Pir" * 2.54cm/")

  desiredDistInDegrees = 50 * degPerCM;
  desiredHeading = 0;

  // Prevent main from exiting with an infinite loop.
  while (true) {

    // vex::task auton(autonPID);

    PrintToController("LateralPower: ", lateralPower, 0, 0, true);

    LDrive.move(lateralPower + rotationalPower);
    RDrive.move(lateralPower - rotationalPower);

    delay(20);
  }
}


#pragma endregion // closing the block with pre-auton / auton / driver control execution functions
