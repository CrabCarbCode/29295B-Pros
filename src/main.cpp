#pragma region BoringVexConfigStuff

#include "main.h"
#include "stdlib.h" //neccessary for std::[commands]
#include "sstream"  //neccessary for... logic
#include "math.h"   //neccessary for functions like abs() and round()
#include "string"   //neccessary for... using strings :sob:
#include "cstring"

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
const int timerTickRate = 20; //the number of 'ticks' in one second
const float degPerCM = 360 / (4.1875 * Pi) * 2.54; // # of degrees per centimeter = 360 / (2Pir" * 2.54cm/")
const int minPrintingDelay = std::ceil(50 / (1000 / timerTickRate));

int flystickPos = 0; //flystick starts at kickstand position
int lastUpTimestamp = 0;
int lastDownTimestamp = 0; 
int lastSpinTimestamp = 0;
int maxFlywheelSpeed = 55 ; //flywheel speed as a percent

double defaultArmPos;

int maxFlystickPos = 4;

int tabVar = 1;

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

const char* toChar(std::string string) {
  return string.c_str();
}

int toInt(float val) {
  return val * 10;
}

void PrintToController(std::string prefix, float data, int row, int page) {
  if (globalTimer % 7 == 0) {
    MainControl.clear();
  }

  if (tabVar == page && (globalTimer % (3 * minPrintingDelay)) == row) {
    MainControl.print(row, 0, prefix.c_str(), toInt(data));
  }
}

int timeSincePoint (int checkedTime) {
  return checkedTime < globalTimer ? (globalTimer - checkedTime) : -1; //returns -1 if checkedtime is in the future
}

//i have no idea what im doing
float AccelSmoothingFunc(float input, float x) {
  const float modifier = (2.5 / sqrt(2 * Pi)) * powf(e, (-0.5 * pow(((2.5 * x) - 2.5), 2)));
  return x >= (timerTickRate / 2) ? (modifier * input) : input;
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

  float lP = 1.0;
  float lI = 0.0;
  float lD = 0.0;

  float lOutput = 1.0;

  float rP = 1.0;
  float rI = 0.0;
  float rD = 0.0;

  float rOutput = 1.0;

  int integralBoundL = 10;
  int integralBoundR = 10;

  // Storage variables for Lateral (forward/back) PID

  float currentErrorL;      // reported value - desired value = position
  float previousErrorL = 0; // position from last loop
  float errorDerivativeL;   //(error - prevError)
  float errorIntegralL = 0;

  float lateralPower = 0;

  // Storage variables for Rotational (turning) PID

  float currentErrorR;      // reported value - desired value = position
  float previousErrorR = 0; // position from last loop
  float errorDerivativeR;   //(error - prevError)
  float errorIntegralR = 0;

  float rotationalPower = 0;

  // the second set of "rotational" storage vars above are technically useless, as the code executes
  // in such a way that only one set of vars is needed to produce both outputs. however, readability is nice
  // change if memory ever becomes an issue

  #pragma endregion //end of PID variable declaration

////// / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

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

    if (fabs(currentErrorL) < integralBoundL) { // NEED TO MAKE ABSOLUTE OF CURRENTERROR WORK
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

    if (fabs(currentErrorR) < integralBoundR) { 
      errorIntegralR += rI * (currentErrorR);
    }
    else {
      errorIntegralR = 0;
    }

    rotationalPower = (currentErrorL + errorDerivativeL + errorIntegralL) / rOutput;

    ///////////////////////////////////////
    //////        ending math        //////
    ///////////////////////////////////////

    //PrintToController("Test: ", globalTimer, 0, 0, false);

    globalTimer += 1;

    PrintToController("ErrorL: %d", currentErrorL, 1, 1);
    PrintToController("LPower: %d", lateralPower, 2, 1);
    PrintToController("ErrorR %d", currentErrorR, 1, 2);
    PrintToController("RPower: %d", rotationalPower, 2, 2);

    previousErrorL = currentErrorL;
    previousErrorR = currentErrorR;
    
    delay(20);
  }

  // return 1;
}

#pragma endregion // end of PID block

////// / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

#pragma region ActualCompetitionFunctions

#pragma region AutonFunctions //Functions for autonomous control

                                                         //CHANGE VALUES THEY ARE FILLER

float previousErrorF = 0; //previous error variable for the flystick arm

bool allowAdjust = false;

void AdjustFlystick() {

  float fP = 0.7;
  float fD = 0.6;
  float fO = 1.5;

  int deadzoneF = 0.3;

  //responsible for changing/maintaining the position of the flystick using a PD controlelr
  //no Integral because fuck you

  int desiredRotation;
  float armPos = ArmRot.get_angle() / 100;

  switch (flystickPos) {
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

const int totalNumOfCommands = 10; // change depending on the total number of "steps" in your autonomous

// struct AutonCommands CommandList[totalNumOfCommands]; //initializes the list of autonomous commands, must be populated in active code (preauton)

////// / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

void ExecuteAutonCommands(struct AutonCommand CurrentCommandList[]) { 
  //responsible for unpacking, tracking and completing each "stage" of the selected autonomous routine

  // 360 degrees = circumferance cm

  for (int i = 0; ++i >= totalNumOfCommands;) {

    //PrintToController("Executing Step: ", i, 0, 0, true);

    struct AutonCommand CurrentCommand = CurrentCommandList[i];

    // setting the PID's target values for this stage of the autonomous routine
    desiredDistInDegrees = CurrentCommand.desiredDistInCM * degPerCM;
    desiredHeading = CurrentCommand.desiredHeading;
 
    float speedMult = CurrentCommand.targetSpeed / 100;
    
    while (currentErrorL >= 5 || currentErrorR >= 2) { 
      autonPID(); 
      PrintToController("Step: %d", i, 1, 1);
    }

    //RotPower is incorporated as a positive on the left side, meaning positive angles make righthand turns

    LDrive.set_voltage_limit(speedMult * (lateralPower + rotationalPower));
    LDrive.set_voltage_limit(speedMult * (lateralPower - rotationalPower));
  }
}

#pragma endregion // end of AutonFunctions

  #pragma region UserControlFunctions

  int prevXVal = 0;
  int prevYVal = 0;
  int XAccelTimeStamp = 0;
  int YAccelTimeStamp = 0;

  int driveMult = 5;

  void DrivingControl(int8_t printingRow) { //resoponsible for user control of the drivetrain

    //Y is forward/back, X is left/right

    //if the controller is in 2 stick mode axis 4 is responsible for turning, axis 1 if not
    int XStickPos = twoStickMode ? MainControl.get_analog(E_CONTROLLER_ANALOG_RIGHT_X) : MainControl.get_analog(E_CONTROLLER_ANALOG_LEFT_X);

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

    int outputY = YStickPos; //AccelSmoothingFunc((YStickPos), (globalTimer - YAccelTimeStamp));
    int outputX = XStickPos; //AccelSmoothingFunc((XStickPos) * RCTurnDamping, (globalTimer - XAccelTimeStamp));

    if ((abs(YStickPos) + abs(XStickPos)) >= deadband) {
      LDrive.move_velocity(driveMult * (outputY + outputX) * fabs(0.5 * (outputY + outputX)));
      RDrive.move_velocity(driveMult * (outputY - outputX) * fabs(0.5 * (outputY - outputX)));
    } else {
      LDrive.move_velocity(0);
      RDrive.move_velocity(0);
    }

    PrintToController("LDrive: %d", (outputY + outputX), 1, 1);
    PrintToController("RDrive: %d", (outputY - outputX), 2, 1);
    PrintToController("Funcval %d", AccelSmoothingFunc(1, (globalTimer - YAccelTimeStamp)), 1, 2);
    PrintToController("FuncX %d", (globalTimer - YAccelTimeStamp), 1, 2);
  
    /*switch ((globalTimer % 4)) {
      case 0:
        if (tabVar == 1) { MainControl.print(1, 0, "LDrive: %d", (outputY + outputX)); } //in units of 1/100th of a degree
        break;
      case 1:
        if (tabVar == 1) { MainControl.print(2, 0, "RDrive: %d", (outputY - outputX)); }
        break;
      case 2:
        if (tabVar == 2) { MainControl.print(1, 0, "funcval %d", AccelSmoothingFunc(1, (globalTimer - YAccelTimeStamp))); }
        break;
      case 3:
        if (tabVar == 2) { MainControl.print(2, 0, "Time: %d", (globalTimer - YAccelTimeStamp)); }
        break;
    }*/

  prevXVal = XStickPos;
  prevYVal = YStickPos;
}  

bool FlywheelFWD = true;
bool flywheelOn = false;

void FlystickControl(int printingRow) { //done
  //responsible for selecting the elevation of the flystick

  int cooldown = 3;

  //MainControl.print(1, 0, "ArmVal: %d", flystickPos);

  //changing the position of the arm
  if (MainControl.get_digital_new_press(DIGITAL_UP) && (globalTimer - lastUpTimestamp >= cooldown) && flystickPos < maxFlystickPos) {
    flystickPos++;
    lastUpTimestamp = globalTimer;
  }

  if (MainControl.get_digital_new_press(DIGITAL_DOWN) && (globalTimer - lastDownTimestamp >= cooldown) && flystickPos > 1) {
    flystickPos--;
    lastDownTimestamp = globalTimer;
  }

  if (MainControl.get_digital_new_press(DIGITAL_B)) {
    FlywheelFWD = !FlywheelFWD;
    FlystickWheelM1.set_reversed(FlywheelFWD);
    FlystickWheelM2.set_reversed(!FlywheelFWD);
  }

  switch (flywheelOn) {
    case 0:
      Flywheel.move_velocity(0);
      break;
    case 1:
      Flywheel.move_velocity(maxFlywheelSpeed * 6); 
      break;
  }

  if ((globalTimer - lastSpinTimestamp >= cooldown) && MainControl.get_digital_new_press(DIGITAL_A)) {
    flywheelOn = !flywheelOn;
  }

  if (printingRow != -1) { 

  }
}

bool RWingLockedOut = false;
bool LWingLockedOut = false;

void WingsControl() { //done

  if (MainControl.get_digital(DIGITAL_L1) && !LWingLockedOut) {
    WingPL.set_value(true);
  } else if (!LWingLockedOut) {
    WingPL.set_value(false);
  }

  if (MainControl.get_digital(DIGITAL_R1) && !RWingLockedOut) {
    WingPR.set_value(true);
  } else if (!RWingLockedOut) {
    WingPR.set_value(false);
  }

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

  #pragma region BoringBlocks
  /**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
  void initialize() {
	Inertial.reset();

  WingPL.set_value(false);
  WingPR.set_value(false); 

  FullDrive.set_zero_position(0);
  FullDrive.set_brake_modes(E_MOTOR_BRAKE_COAST);

  Flywheel.set_brake_modes(E_MOTOR_BRAKE_COAST);

  FlystickArmM.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  ArmRot.reverse();

  defaultArmPos = ArmRot.get_position() / 100;
  delay(100);
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

  #pragma endregion

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


  delay(300);

  /*if (globalTimer == (timerTickRate / 2)) {
    flystickPos = 3;
  }*/

  PrintToController("Timer: %d", globalTimer, 1, -1);


  /*  test motors driving autonomously
    *  FullDrive.move(100);
    *  delay(300);
    *  FullDrive.brake();
    *  delay(300);
    *  LDrive.move(-100);
    *  RDrive.move(-100);
    *  delay(200);
    *  FullDrive.brake;
    *  delay(5000);
    */ 

	while (true) { 

  if (MainControl.get_digital_new_press(DIGITAL_LEFT)) {
    flystickPos++;
  }

  DrivingControl(-1);
  WingsControl();
  FlystickControl(-1);
  AdjustFlystick();
    
    globalTimer++;
    delay(1000 / timerTickRate);
  }
}