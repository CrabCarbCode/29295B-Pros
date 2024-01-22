#include "main.h"

/*
 * note on name conventions, every variable is camelcase
 * while all components/functions have capitalized first words
 * motors end with M
 */
pros::Controller MainControl(E_CONTROLLER_MASTER);  // declared with "pros::" to avoid ambiguity with stock vex controller


Motor LDriveFrontM(5, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
Motor LDriveMidM(4, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
Motor LDriveBackM(1, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);

Motor RDriveFrontM(7, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
Motor RDriveMidM(10, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
Motor RDriveBackM(8, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);

Motor FlystickArmM(20, E_MOTOR_GEARSET_36, false, E_MOTOR_ENCODER_DEGREES);

Motor FlywheelM(18, E_MOTOR_GEARSET_18, false, E_MOTOR_ENCODER_DEGREES);

ADIDigitalOut WingPR('G');
ADIDigitalOut WingPL('H');

Imu Inertial(9);  // initializing the Inertial sensor
Rotation ArmRot(19);

Motor_Group LDrive({LDriveFrontM, LDriveMidM, LDriveBackM});
Motor_Group RDrive({RDriveFrontM, RDriveMidM, RDriveBackM});

Motor_Group FullDrive({LDriveFrontM, LDriveMidM, LDriveBackM, RDriveFrontM, RDriveMidM, RDriveBackM});