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
const int timerTickRate = 50; //the number of 'ticks' in one second
const float degPerCM = 360 / (4.1875 * Pi * 2.54); // # of degrees per centimeter = 360 / (2Pir" * 2.54cm/")
const int minPrintingDelay = std::ceil(50 / (1000 / timerTickRate));

int flystickPos = 0; //flystick starts at kickstand position
int lastUpTimestamp = 0;
int lastDownTimestamp = 0; 
int lastSpinTimestamp = 0;
int maxFlywheelSpeed = 48; //flywheel speed as a percent

double defaultArmPos;

int maxFlystickPos = 5;

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
  return val;
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
  bool autonPIDIsEnabled = true;

  int desiredDistInDegrees; // temporary
  int desiredHeading;

//////       TUNING INSTRUCTIONS       //////
/*
1. set all tuning values to 0
2. experiment with the P multiplier until you can drive a precise distance with slight oscilations at the end
3. experiment with the D multiplier until the oscilations slowly die out
4. experiment with the I multiplier until oscilations do not occur
*/

  // tuning coefficients

  float lP = 1.0;
  float lD = 1.2;
  float lI = 0.3;

  float lOutput = 3.0;

  float rP = 0.7;
  float rD = 1.0;
  float rI = 0.4;

  float rOutput = 2.0;

  int integralBoundL = 75;
  int integralBoundR = 25;

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

void AutonPID() {

  if (autonPIDIsEnabled) {
  
    ///////////////////////////////////////
    //////        Lateral PID        //////
    ///////////////////////////////////////

    int avgMotorPosition = (RDriveFrontM.get_position() + LDriveFrontM.get_position()) / 2;

    errorProportionalL = lP * (desiredDistInDegrees - avgMotorPosition); // proportional error
    errorDerivativeL = lD * (errorProportionalL - previousErrorL);       // derivative of error

    // filters out the integral at long ranges (if |error| < constant upper limit, eg. 10cm),
    // allowing it to be useful when needed without overpowering other elements

    if (fabs(errorProportionalL) < integralBoundL) { // NEED TO MAKE ABSOLUTE OF CURRENTERROR WORK
      errorIntegralL += lI * (errorProportionalL);
    }
    else {
      errorIntegralL = 0;
    }

    lateralPower = (errorProportionalL + errorDerivativeL + errorIntegralL) / lOutput;

    ///////////////////////////////////////
    //////      Rotational PID       //////
    ///////////////////////////////////////

    int avgBotRotation = LDriveFrontM.get_position() - RDriveFrontM.get_position(); //Inertial.get_heading(); 

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

    rotationalPower = (errorProportionalR + errorDerivativeR + errorIntegralR) / rOutput;

    ///////////////////////////////////////
    //////        ending math        //////
    ///////////////////////////////////////

    /*PrintToController("Heading: %d", avgBotRotation, 0, 2);
    PrintToController("Heading: %d", avgBotRotation, 0, 3);
    PrintToController("ErrorL: %d", errorProportionalL, 1, 2);
    PrintToController("ErrorR %d", errorProportionalR, 2, 2);
    PrintToController("LPower: %d", lateralPower + rotationalPower, 1, 3);
    PrintToController("RPower: %d", lateralPower - rotationalPower, 2, 3);
    */

    PrintToController("LOutput: %d", (lateralPower + rotationalPower) * 0.3, 1, 3); //lateralPower + rotationalPower
    PrintToController("ROutput: %d", (lateralPower - rotationalPower) * 0.3, 2, 3);

    LDrive.move_velocity((lateralPower + rotationalPower));
    RDrive.move_velocity((lateralPower - rotationalPower));

    previousErrorL = errorProportionalL;
    previousErrorR = errorProportionalR;
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

  float fP = 0.6;
  float fD = 0.8;
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
    FlystickStabilizerM.move_velocity(FlystickArmM.get_actual_velocity() / 5);
  } else {
    FlystickArmM.move_velocity(0);
    FlystickStabilizerM.brake();
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
    
    while (errorProportionalL >= 5 || errorProportionalR >= 2) { 
      AutonPID(); 
      PrintToController("Step: %d", i, 1, 2);
      PrintToController("Step: %d", i, 2, 2);
    }
  }
}

#pragma endregion // end of AutonFunctions

  #pragma region UserControlFunctions

  int prevXVal = 0;
  int prevYVal = 0;
  int XAccelTimeStamp = 0;
  int YAccelTimeStamp = 0;

  int driveMult = 5;

  bool driveReversed = false;

  void DrivingControl(int8_t printingRow) { //resoponsible for user control of the drivetrain

    if (MainControl.get_digital_new_press(DIGITAL_Y)) {
      driveReversed = !driveReversed;
      LDrive.set_reversed(!driveReversed);
      RDrive.set_reversed(driveReversed);
    }

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

    //PrintToController("LDrive: %d", (outputY + outputX), 1, 1);
    //PrintToController("RDrive: %d", (outputY - outputX), 2, 1);
    PrintToController("Funcval %d", AccelSmoothingFunc(1, (globalTimer - YAccelTimeStamp)), 1, 2);
    PrintToController("FuncX %d", (globalTimer - YAccelTimeStamp), 1, 2);

  prevXVal = XStickPos;
  prevYVal = YStickPos;
}  

bool FlywheelFWD = true;
bool flywheelOn = false;

void FlystickControl(int printingRow) { //done
  //responsible for selecting the elevation of the flystick

  int cooldown = 3;

  //changing the position of the arm
  /*if (MainControl.get_digital_new_press(DIGITAL_DOWN) && flystickPos == 0) { //dealing with the kickstand, never to be used afterwards
    flystickPos = 3;
  }*/

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

  WingPL.set_value(false);
  WingPR.set_value(false); 

  FullDrive.set_zero_position(0);
  FullDrive.set_brake_modes(E_MOTOR_BRAKE_COAST);

  Flywheel.set_brake_modes(E_MOTOR_BRAKE_COAST);

  FlystickArmM.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  ArmRot.reverse();

  defaultArmPos = ArmRot.get_position() / 100;
  //Inertial.set_heading(0);
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

  vector <double> autonCommands[34];
  int stageChangeTimeStamp = 0;


  void autonomous() {

    int autonStep = 1;

    //autonCommands[  ] = { , };
    autonCommands[ 1 ] = {10 , 0}; //dummy, shooting
    autonCommands[ 2 ] = {-10, 0};
    autonCommands[ 3 ] = {0, 90}; //experiment autonCommands[ 2 ] = {0 , -60};
    /*autonCommands[ 4 ] = {78 , 0};
    autonCommands[ 5 ] = {0 , 90};
    autonCommands[ 6 ] = {297 , 0};
    autonCommands[ 7 ] = {-58 , 0};
    autonCommands[ 8 ] = {52 , 0};
    autonCommands[ 9 ] = {-52 , 0};
    autonCommands[ 10 ] = {0 , 90};
    autonCommands[ 11 ] = {70 , 0};
    autonCommands[ 12 ] = {0 , -90};
    autonCommands[ 13 ] = {88 , 0};
    autonCommands[ 14 ] = {0 , -90};
    autonCommands[ 15 ] = {31 , 0};
    autonCommands[ 16 ] = {-31 , 0};
    autonCommands[ 17 ] = {0 , -90};
    autonCommands[ 18 ] = {90 , 0};
    autonCommands[ 19 ] = {0 , 90};
    autonCommands[ 20 ] = {77 , 0};
    autonCommands[ 21 ] = {0 , 90};
    autonCommands[ 22 ] = {58 , 0};
    autonCommands[ 23 ] = {-60 , 0};
    autonCommands[ 24 ] = {0 , -90};
    autonCommands[ 25 ] = {50 , 0};
    autonCommands[ 26 ] = {0 , 90};
    autonCommands[ 27 ] = {58 , 0};
    autonCommands[ 28 ] = {0 , 90};
    autonCommands[ 29 ] = {31 , 0};
    autonCommands[ 31 ] = {-31 , 0}; */
  //AutonCommand Gfirst = { 100, 0, 127, 500};
  //AutonCommand Gsecond = { 100, 0, 127, 500};


  //AutonCommand GameAuton[totalNumOfCommands] = { Gfirst, Gsecond};

  flystickPos = 3;

  while (globalTimer <= 5 * timerTickRate) {

    AdjustFlystick();

    Flywheel.move_velocity(50 * 6);
    PrintToController("Step: %d", autonStep, 0, 1);
    PrintToController("Speed: %d", FlystickWheelM1.get_actual_velocity(), 0, 1);

    globalTimer++;
    delay(1000 / timerTickRate);
  }

  flystickPos = 1;

  while (globalTimer <= 6 * timerTickRate) {
  AdjustFlystick();
  Flywheel.move_velocity(0);
  globalTimer++;
  delay(1000 / timerTickRate);
  }       

  while (true) {   

     FullDrive.set_brake_modes(E_MOTOR_BRAKE_BRAKE);

    if (MainControl.get_digital_new_press(DIGITAL_X)) {
      autonStep++;
    }

    vector <double> currentCommand = autonCommands[autonStep];

    desiredDistInDegrees = currentCommand.at(0) * degPerCM; //currentCommand.at(0) * degPerCM;
    desiredHeading = (currentCommand.at(1) * degPerCM * 1.15); 

    PrintToController("Step: %d", autonStep, 0, 1);
    PrintToController("TargetL: %d", desiredDistInDegrees, 1, 1); //desiredDistInDegrees
    PrintToController("ErrorL: %d", errorProportionalL, 2, 1); //desiredDistInDegrees

    PrintToController("Timer: %d", globalTimer, 0, 2);
    PrintToController("ErrorL: %d", errorProportionalL, 1, 2); //desiredDistInDegrees
    PrintToController("TargetR: %d", desiredHeading, 2, 2);

    if (MainControl.get_digital_new_press(DIGITAL_LEFT)) {
      tabVar--;
    } 
    if (MainControl.get_digital_new_press(DIGITAL_RIGHT)) {
      tabVar++;
    }

    AutonPID();

    if ((fabs(errorProportionalL) <= 45 && fabs(errorProportionalR) <= 30)) {

      stageChangeTimeStamp = globalTimer;
      previousErrorL = 0;
      FullDrive.brake();

      
      //autonStep++;
    }
  
    globalTimer++;
    delay(1000 / timerTickRate); 
    }
  }

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

  //flystickPos = 2;

  autonomous();

  /*flystickPos = 3;

	while (true) { 

    PrintToController("Timer: %d", globalTimer, 0, 1);
    PrintToController("Heading: %d", Inertial.get_heading(), 1, 1);
    PrintToController("ArmPos: %d", flystickPos, 2, 1);

    if (MainControl.get_digital_new_press(DIGITAL_LEFT)) {
      tabVar--;
    } 
    if (MainControl.get_digital_new_press(DIGITAL_RIGHT)) {
      tabVar++;
    }

    DrivingControl(-1);
    WingsControl();
    FlystickControl(-1);
    AdjustFlystick();
    
    globalTimer++;
    delay(1000 / timerTickRate);
  }*/
}