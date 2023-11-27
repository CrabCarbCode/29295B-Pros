#pragma region BoringVexConfigStuff

#include "main.h"
#include "stdlib.h" //neccessary for std::[commands]
#include "sstream"  //neccessary for... logic
#include "math.h"   //neccessary for functions like abs() and round()
#include "string"   //neccessary for... using strings :sob:

#include "robot-config.h" //importing the motors and whatnot

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
int globalTimer = 0;
const double degPerCM = 360 / (4.1875 * Pi) * 2.54; // # of degrees per centimeter = 360 / (2*Pi*r" * 2.54cm/")

int flystickPos = 1; //flystick starts at kickstand position
int lastUpTimestamp = 0;
int lastDownTimestamp = 0;
int lastSpinChangeTimestamp = 0;
int maxFlywheelSpeed = 100; //flywheel speed as a percent

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

  std::string format = prefix.append("%d");
  MainControl.print(row, column, format.c_str(), data);
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

int timeSincePoint (int checkedTime) {
  return checkedTime < globalTimer ? (globalTimer - checkedTime) : -1; //returns -1 if checkedtime is in the future
}


#pragma endregion

////// / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

#pragma region AutonPID //the code behind the autonomous Proportional Integral Derivative controller


///// Control Variables //////

bool twoStickMode = true; // toggles single or double stick drive
int deadband = 12;         // if the controller sticks are depressed less than deadband%, input will be ignored (to combat controller drift)
double RCTurnDamping = 0.65;


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

    if (absDouble(currentErrorR) < integralBoundR) { 
      errorIntegralR += rI * (currentErrorR);
    }
    else {
      errorIntegralR = 0;
    }

    rotationalPower = (currentErrorL + errorDerivativeL + errorIntegralL) / rOutput;

    ///////////////////////////////////////
    //////        ending math        //////
    ///////////////////////////////////////

    PrintToController("Test: ", globalTimer, 0, 0, false);

    globalTimer += 1;

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

double previousErrorF = 0; //previous error variable for the flystick arm

void AdjustFlystick() {
  //responsible for changing/maintaining the position of the flystick using a PD controlelr
  //no Integral because fuck you

  int desiredRotation;
  double fO = 1.0;

  switch (flystickPos) {
    case 0:
      desiredRotation = -45;
      break;
    case 1:
      desiredRotation = 0;
      break;
    case 2:
      desiredRotation = 10;
      break;
    case 3:
      desiredRotation = 45;
      break;
  }

  double currentErrorF = desiredRotation - ArmRot.get_position();
  double errorDerivativeF = currentErrorF - previousErrorF;

  if (abs(currentErrorF) >= 3) { //FlystickArmM.move_velocity((currentErrorF + errorDerivativeF) * fO);
  //PrintToController("ArmPos: ", ((currentErrorF + errorDerivativeF) * fO), 3, 0, false);
  }

  previousErrorF = currentErrorF;

}

// structure containing all neccessary data for an autonomous command
struct AutonCommand {
  double desiredDistInCM;
  double desiredHeading;

  double targetSpeed;
  int endDelay;
};

const int totalNumOfCommands = 10; // change depending on the total number of "steps" in your autonomous

// struct AutonCommands CommandList[totalNumOfCommands]; //initializes the list of autonomous commands, must be populated in active code (preauton)

////// / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / / /

void ExecuteAutonCommands(struct AutonCommand CurrentCommandList[]) { 
  //responsible for unpacking, tracking and completing each "stage" of the selected autonomous routine

  // 360 degrees = circumferance cm

  for (int i = 0; ++i >= totalNumOfCommands;) {

    PrintToController("Executing Step: ", i, 0, 0, true);

    struct AutonCommand CurrentCommand = CurrentCommandList[i];

    // setting the PID's target values for this stage of the autonomous routine
    desiredDistInDegrees = CurrentCommand.desiredDistInCM * degPerCM;
    desiredHeading = CurrentCommand.desiredHeading;

    double speedMult = CurrentCommand.targetSpeed / 100;
    
    while (currentErrorL >= 5 || currentErrorR >= 2) { 

      autonPID(); 

      PrintToController("Executing Step: ", i, 1, 0, true);
      PrintToController("ErrorL: ", currentErrorL, 2, 0, false);
      PrintToController("ErrorR: ", currentErrorR, 3, 0, false);
    }

    //RotPower is incorporated as a positive on the left side, meaning positive angles make righthand turns

    LDrive.set_voltage_limit(speedMult * (lateralPower + rotationalPower));
    LDrive.set_voltage_limit(speedMult * (lateralPower - rotationalPower));
  }
}

#pragma endregion // end of AutonFunctions

  #pragma region UserControlFunctions

  void DrivingControl(int8_t printingRow) { //resoponsible for user control of the drivetrain

    //if the controller is in 2 stick mode axis 4 is responsible for turning, axis 1 if not
    int turnAxisValue = twoStickMode ? MainControl.get_analog(E_CONTROLLER_ANALOG_RIGHT_X) : MainControl.get_analog(E_CONTROLLER_ANALOG_LEFT_X);

    if (absInt(MainControl.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) + absInt(turnAxisValue) >= deadband)) {

    double YAxisSpeed = MainControl.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
    double turnSpeed = turnAxisValue * RCTurnDamping;

    //PrintToController("FWD: ", (YAxisSpeed), 0, 0, true);
    //PrintToController("SIDE: ", (turnSpeed), 1, 0, true);

    //LDrive.move_velocity(pow((YAxisSpeed + turnSpeed), 2));
    //RDrive.move_velocity(pow((YAxisSpeed - turnSpeed), 2));

    } else {
   
      LDrive.move_velocity(0);
      RDrive.move_velocity(0);

    }
  }

bool FlywheelFWD = true;
bool FlywheelOn = false;

void FlystickControl(int printingRow, int timeSinceUp, int timeSinceDown, int timeSinceSpin) {
  //responsible for selecting the elevation of the flystick

  int cooldown = 3;

  //changing the position of the arm
  if (MainControl.get_digital_new_press(DIGITAL_UP) && (timeSinceUp >= cooldown)) {
    flystickPos++;
    lastUpTimestamp = globalTimer;
  }

  if (MainControl.get_digital_new_press(DIGITAL_DOWN) && (timeSinceUp >= cooldown)) {
    flystickPos--;
    lastDownTimestamp = globalTimer;
  }

  if (MainControl.get_digital_new_press(DIGITAL_B)) {
    FlywheelFWD = !FlywheelFWD;
    Flywheel.set_reversed(FlywheelFWD);
  }

  //PrintToController(" Speed: ", (FlystickWheelM1.get_actual_velocity() / 600), printingRow, 7, false);

  if ((timeSinceSpin <= cooldown) || !MainControl.get_digital_new_press(DIGITAL_A)) { 
    //kills the function if spin button is on cooldown or button has not been pressed   
    return;
  } else {

    if (FlywheelOn) {FlywheelOn = false; Flywheel.brake(); } //clunky and bad, should be replaced
    else {FlywheelOn = true; Flywheel.move_velocity(maxFlywheelSpeed * 6); }
  }
}

  #pragma endregion // end of UserControlFunctions

#pragma endregion // end of Bot controlling functions

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



#pragma region ExecutionBlock //the region in which the above code is executed during a competition

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
  FullDrive.set_brake_modes(E_MOTOR_BRAKE_HOLD);
  ArmRot.set_position(30);

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

    DrivingControl(0);
    FlystickControl(1, (globalTimer - lastUpTimestamp), (globalTimer - lastDownTimestamp), (globalTimer - lastSpinChangeTimestamp));
    double XstickPos = MainControl.get_analog(E_CONTROLLER_ANALOG_LEFT_X);
    double YstickPos = MainControl.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);

    //PrintToController("YStick: ", YstickPos, 0, 0, true);
    //PrintToController("XStick: ", XstickPos, 1, 0, false);
    //MainControl.print(0, 0, "XStick: %d", XstickPos);
    //MainControl.print(0 , 0, "Timer: %d", globalTimer);
    //PrintToController("XStick: ", XstickPos, 1, 0, false);
    //PrintToController("FS - Pos: ", flystickPos, 2, 0, false);
    //AdjustFlystick();

    
    MainControl.print(0 , 0, "ControllerVal: %d", (YstickPos));
    
    if (globalTimer % 3 == 0) {
      MainControl.clear_line(0);
    }
    
    globalTimer++;
    delay(50);
	}
}

#pragma endregion //end of execution block