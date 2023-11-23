#include "main.h"

/*
 * note on name conventions, every variable is camelcase 
 * while all components/functions have capitalized first words
 * motors end with M
*/
pros::Controller MainControl (E_CONTROLLER_MASTER); //declared with "pros::" to avoid ambiguity with stock vex controller


Motor LDriveFrontM (20, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
Motor LDriveBackM (10, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
Motor RDriveFrontM (11, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
Motor RDriveBackM (1, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);

Motor FlystickArmM (12, E_MOTOR_GEARSET_36, true, E_MOTOR_ENCODER_DEGREES);
Motor FlystickWheelM (13, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);


Imu Inertial(15); //initializing the Inertial sensor

Motor_Group LDrive ({LDriveFrontM, LDriveBackM});
Motor_Group RDrive ({RDriveFrontM, RDriveBackM});

Motor_Group FullDrive ({LDriveFrontM, LDriveBackM});
