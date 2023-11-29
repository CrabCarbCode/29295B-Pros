#pragma region BoringVexConfigStuff

#include "main.h"
#include "stdlib.h" //neccessary for std::[commands]
#include "sstream"  //neccessary for... logic
#include "math.h"   //neccessary for functions like abs() and round()
#include "string"   //neccessary for... using strings :sob:

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
int timerTickRate = 20; //the number of 'ticks' in one second
const double degPerCM = 360 / (4.1875 * Pi) * 2.54; // # of degrees per centimeter = 360 / (2*Pi*r" * 2.54cm/")

int flystickPos = 1; //flystick starts at kickstand position
int lastUpTimestamp = 0;
int lastDownTimestamp = 0; 
int lastSpinTimestamp = 0;
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
  MainControl.print(row, column, prefix.c_str(), data);
}

int timeSincePoint (int checkedTime) {
  return checkedTime < globalTimer ? (globalTimer - checkedTime) : -1; //returns -1 if checkedtime is in the future
}

//i have no idea what im doing
double AccelSmoothingFunc(double input, double x) {
    const float modifier = (2.5 / sqrt(2 * Pi)) * powf(e, (-0.5 * pow(((2.5 * x) - 2.5), 2)));

    return x >= (timerTickRate / 2) ? (modifier * input) : input;
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

  /*
  store previous stick amounts
  if stick changes significantly, set time marker
  input smoothing function with time since marker

  */

  #pragma region UserControlFunctions

  int prevXVal = 0;
  int prevYVal = 0;
  int XAccelTimeStamp = 0;
  int YAccelTimeStamp = 0;

  void DrivingControl(int8_t printingRow) { //resoponsible for user control of the drivetrain

    //Y is forward/back, X is left/right

    //if the controller is in 2 stick mode axis 4 is responsible for turning, axis 1 if not
    int currentXVal = twoStickMode ? MainControl.get_analog(E_CONTROLLER_ANALOG_RIGHT_X) : MainControl.get_analog(E_CONTROLLER_ANALOG_LEFT_X);

    double currentYVal = MainControl.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
    double turnSpeed = currentXVal * RCTurnDamping;

    //implementing a slow initial acceleration for precision movements

    if (abs(prevYVal <= deadband && (abs(currentYVal) - abs(prevYVal) >= 25))) {  
      //if the Y stick's position has changed drastically from zero, set a marker for the acceleration function
      YAccelTimeStamp = globalTimer;
    }
    if (abs(prevXVal <= deadband && (abs(currentXVal) - abs(prevXVal) >= 20))) {  
      //if the X stick's position has changed drastically from zero, set a marker for the acceleration function
      XAccelTimeStamp = globalTimer;
    }

    if ((abs(MainControl.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)) + abs(currentXVal)) >= deadband) {

    int outputY = AccelSmoothingFunc((currentYVal), (globalTimer - YAccelTimeStamp));
    int outputX = AccelSmoothingFunc((currentXVal), (globalTimer - XAccelTimeStamp));
    
    /* print drive stuff
    *  PrintToController("LDrive: %d", LDriveFrontM.get_actual_velocity(), 1, 0, false);
    *  PrintToController("RDrive: %d", RDriveFrontM.get_actual_velocity(), 1, 0, false);
    */

    //LDrive.move_velocity(outputY);
    //RDrive.move_velocity(outputX);

    } else {
   
      LDrive.move_velocity(0);
      RDrive.move_velocity(0);

    }

    prevXVal = currentXVal;
    prevYVal = currentYVal;
  }

bool FlywheelFWD = true;
bool FlywheelOn = false;

void FlystickControl(int printingRow) {
  //responsible for selecting the elevation of the flystick

  int cooldown = 3;

  //changing the position of the arm
  if (MainControl.get_digital_new_press(DIGITAL_UP) && (globalTimer - lastUpTimestamp >= cooldown)) {
    flystickPos++;
    lastUpTimestamp = globalTimer;
  }

  if (MainControl.get_digital_new_press(DIGITAL_DOWN) && (globalTimer - lastDownTimestamp >= cooldown)) {
    flystickPos--;
    lastDownTimestamp = globalTimer;
  }

  if (MainControl.get_digital_new_press(DIGITAL_B)) {
    FlywheelFWD = !FlywheelFWD;
    Flywheel.set_reversed(FlywheelFWD);
  }

  if (printingRow != -1) { PrintToController("Speed: %d", (FlystickWheelM1.get_actual_velocity() / 600), printingRow, 7, false); }

  if ((globalTimer - lastSpinTimestamp <= cooldown) || !MainControl.get_digital_new_press(DIGITAL_A)) { 
    //kills the function if spin button is on cooldown or button has not been pressed   
    return;
  } else {
    lastSpinTimestamp = globalTimer;
    if (FlywheelOn) {FlywheelOn = false; Flywheel.brake(); } //clunky and bad, should be replaced
    else {FlywheelOn = true; Flywheel.move_velocity(maxFlywheelSpeed * 6); }
  }
}

void WingsControl() {

  if (MainControl.get_digital(DIGITAL_R1)) {
    WingP1.set_value(true);
  } else {
    WingP1.set_value(false);
  }

  if (MainControl.get_digital(DIGITAL_L1)) {
    WingP2.set_value(true);
  } else {
    WingP2.set_value(false);
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

  FullDrive.set_zero_position(0);
  FullDrive.set_brake_modes(E_MOTOR_BRAKE_HOLD);

  Flywheel.set_brake_modes(E_MOTOR_BRAKE_COAST);

  FlystickArmM.set_brake_mode(E_MOTOR_BRAKE_HOLD);
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

  /*  test motors driving autonomously
    *  FullDrive.move(100);
    *  delay(300);
    *  FullDrive.brake();
    *  delay(300);
    *  LDrive.move(-100);
    *  RDrive.move(-100);
    *  delay(200);
    *  FullDrive.brake;
    */ 

	while (true) { 

    if (globalTimer % 3 == 0) {
      //testing the printing function
      
      //std::string tempString = "Timer: %d"
      //MainControl.clear_line(0);
      //MainControl.print(0, 0, tempString.c_str(), globalTimer);
      PrintToController("Timer: %d", globalTimer, 0, 0, true);

      /* report the rotation sensor's readings
      *  PrintToController("RotSens: %d", ArmRot1, 0, false)
      *  PrintToController("ArmStep: %d",)
      */
    }

  //DrivingControl(-1);
  WingsControl();
  FlystickControl(1);
  double XstickPos = MainControl.get_analog(E_CONTROLLER_ANALOG_LEFT_X);
  double YstickPos = MainControl.get_analog(E_CONTROLLER_ANALOG_LEFT_Y);
    
  MainControl.print(0 , 0, "ControllerVal: %d", (YstickPos));
    
  /* test to see if the controller is being read (negative and positive)
  *  FullDrive.move(MainControl.get_analog(E_CONTROLLER_ANALOG_LEFT_Y));
  */

  /* test to see if the stick value can be printed
  *  PrintToController("YStickVal: %d", YstickPos, )
  */ 

  /*  test to see if multiple things can be printed
  *  MainControl.print(1, 0, "Battery: %d", MainControl.get_battery_level());
  *  MainControl.print(2, 0, "Battery2: %d", MainControl.get_battery_capacity());
  */ 

  /* test to see whether the flywheel button delays are being calculated properly
  *  PrintToController("UpDelay: %d", (globalTimer - lastUpTimestamp), 1, 0, false);
  *  PrintToController("DownStamp: %d", lastDownTimestamp, 2, 0. false);
  *  PrintToController("Spin: %d", FlywheelFWD, 3, 0, false);
  */ 
    
    globalTimer++;
    delay(1000 / timerTickRate);
	}
}

#pragma endregion //end of execution block