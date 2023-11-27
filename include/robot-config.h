#include "main.h"

/*
 * note on name conventions, every variable is camelcase 
 * while all components/functions have capitalized first words
 * motors end with M
*/
pros::Controller MainControl (E_CONTROLLER_MASTER); //declared with "pros::" to avoid ambiguity with stock vex controller


Motor LDriveFrontM (2, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
Motor LDriveBackM (1, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
Motor RDriveFrontM (8, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
Motor RDriveBackM (9, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);

Motor FlystickArmM (6, E_MOTOR_GEARSET_36, true, E_MOTOR_ENCODER_DEGREES);
Motor FlystickWheelM1 (13, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
Motor FlystickWheelM2 (15, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);

ADIDigitalOut WingP1 ('G');
ADIDigitalOut WingP2 ('H');

Imu Inertial(19); //initializing the Inertial sensor
Rotation ArmRot(18);

Motor_Group LDrive ({LDriveFrontM, LDriveBackM});
Motor_Group RDrive ({RDriveFrontM, RDriveBackM});

Motor_Group FullDrive ({LDriveFrontM, LDriveBackM});
Motor_Group Flywheel ({FlystickWheelM1, FlystickWheelM2});
