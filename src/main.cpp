#include "main.h"

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	Motor FL(FLPort, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
	Motor ML(MLPort, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
	Motor BL(BLPort, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
	Motor FR(FRPort, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
	Motor MR(MRPort, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
	Motor BR(BRPort, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);

	Motor intake(intakePort, E_MOTOR_GEARSET_06, true, E_MOTOR_ENCODER_DEGREES);
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
 */
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
	Motor FL(FLPort);
	Motor ML(MLPort);
	Motor BL(BLPort);
	Motor FR(FRPort);
	Motor MR(MRPort);
	Motor BR(BRPort);

	Motor intake(intakePort);

	FL.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	ML.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	BL.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	FR.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	MR.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
	BR.set_brake_mode(E_MOTOR_BRAKE_BRAKE);

	Controller master(E_CONTROLLER_MASTER);
	bool tankDrive = true;
	while(true) {
		double left, right;
		if(master.get_digital_new_press(DIGITAL_Y)) tankDrive = !tankDrive;

		if(tankDrive) {
			left = master.get_analog(ANALOG_LEFT_Y);
			right = master.get_analog(ANALOG_RIGHT_Y);
		} else {
			double power = master.get_analog(ANALOG_LEFT_Y);
			double turn = master.get_analog(ANALOG_RIGHT_X);
			left = power + turn;
			right = power - turn;
		}

		intake.move((master.get_digital(DIGITAL_R1) - master.get_digital(DIGITAL_R2))*127);

		FL.move(left);
		ML.move(left);
		BL.move(left);
		FR.move(right);
		MR.move(right);
		BR.move(right);

		delay(5);
  }
}
