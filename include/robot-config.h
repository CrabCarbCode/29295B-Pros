#include "main.h"

/*
 * note on name conventions, every variable is camelcase
 * while all components/functions have capitalized first words
 * motors end with M
 */
pros::Controller MainControl(E_CONTROLLER_MASTER);  // declared with "pros::" to avoid ambiguity with stock vex controller


Motor LDriveFrontM(8, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
Motor LDriveTopM(3, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
Motor LDriveBackM(1, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);

Motor RDriveFrontM(7, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
Motor RDriveTopM(9, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
Motor RDriveBackM(10, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);

Motor FlystickArmM(18, E_MOTOR_GEARSET_36, true, E_MOTOR_ENCODER_DEGREES);

Motor FlywheelM(14, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);

ADIDigitalOut WingPR('A');
ADIDigitalOut WingPL('H');

Imu Inertial(17);  // initializing the Inertial sensor
Rotation ArmRot(20);

Motor_Group FwdDrive({LDriveFrontM, RDriveTopM, LDriveBackM});
Motor_Group RvsDrive({RDriveFrontM, LDriveTopM, RDriveBackM});

Motor_Group LDrive({LDriveFrontM, LDriveTopM, LDriveBackM});  // LDriveFrontM, LDriveTopM, LDriveBackM
Motor_Group RDrive({RDriveFrontM, RDriveTopM, RDriveBackM});  // RDriveFrontM, RDriveTopM, RDriveBackM

Motor_Group FullDrive({LDriveFrontM, LDriveTopM, LDriveBackM, RDriveFrontM, RDriveTopM, RDriveBackM});