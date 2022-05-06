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

	Motor intake(intakePort, E_MOTOR_GEARSET_06, false, E_MOTOR_ENCODER_DEGREES);
	Motor arm(armPort, E_MOTOR_GEARSET_36, true, E_MOTOR_ENCODER_DEGREES);

  Rotation armRot(armRotPort);
  ADIDigitalIn clampLimit(clampLimitPort);

	ADIDigitalOut needleTilter(needleTilterPort);
	ADIDigitalOut needle(needlePort);
	ADIDigitalOut clamp(clampPort);
	ADIDigitalOut hook(hookPort);
	// ADIDigitalOut lTilter(lTilterPort);
	// ADIDigitalOut rTilter(rTilterPort);

  armRot.reverse();

	Task armControlTask(armControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Arm Control Task");
	// Task tilterControlTask(tilterControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Arm Control Task");
	Task intakeControlTask(intakeControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Intake Control Task");
	Task hookControlTask(hookControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Intake Control Task");
  Task sensorsTask(sensors, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Debug Task");
  Task debugTask(Debug, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Debug Task");

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
void red() {
	double start = millis();
	// setArmPos(2);
	setOffset(30);
	baseTurn(30);
	delay(100);
  Task odometryTask(Odometry, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odom Task");
	Task controlTask(PPControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PP Task");

	setMaxRPMV(500);
	// setArmHeight(1400);

  double smooth = 0.75;
	setEnableControl(true);
  // basePP({position, Node(24, 72)}, 1-smooth, smooth, 18);
  // waitPP(0);
  // enableBase(true, true);
  // baseTurn(-90);
  // waitTurn(10000);

	// setArmPos(1);
	// setNeedleState(HIGH);
	// setNeedleTilterState(LOW);
	setIntake(-127);
	delay(300);
	setIntake(127);
	delay(200);
	baseMove(-1);
	delay(300);
	waitPP(1000);

	baseMove(8);
	waitPP(1000);

	setNeedleState(HIGH);
	setArmPos(1);
	delay(700);
	setNeedleTilterState(LOW);
	delay(700);
	setNeedleState(LOW);
	delay(1000);

	setArmPos(0);
	setNeedleTilterState(HIGH);
	baseMove(13);
	waitPP(1000);

	baseMove(-20);
	waitPP(1000);

	enableBase(true, false);
	baseTurn(115);
	waitTurn(1500);

	// enableBase(false, true);
	// baseTurn(150);
	// waitTurn(1500);

	setClampState(LOW);
	basePP({position, Node(-8, 59)}, 1-smooth, smooth, 14, true);
	waitPP(2000);
	setClampState(HIGH);

	baseMove(14);
	waitPP(2000);

	// baseTurn(calcBaseTurn(-24, 0, true));
	// waitTurn(1500);


	enableBase(true, true);
	baseTurn(360);
	waitTurn(1500);

	setClampState(LOW);
	delay(300);
	setClampState(LOW);

	setNeedleState(HIGH);
	setArmPos(1);
	setNeedleTilterState(LOW);
	basePP({position, Node(-14, (120-23.5))}, 1-smooth, smooth, 13, false);
	waitPP(3000);
	baseTurn(356);
	waitTurn(10);

	while(millis() - start < 14900) {
		printf("time: %.2fs\n", millis() - start);
		delay(5);
	}
	setNeedleState(LOW);
	while(millis() - start < 14990);
	controlTask.remove();
	setEnableControl(false);
	// setIntake(127);
	printf("program finished in %.2fs", start - millis());
}

void blue() {
	double start = millis();
	// setArmPos(2);
	setOffset(30);
	baseTurn(30);
	delay(100);
  Task odometryTask(Odometry, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Odom Task");
	Task controlTask(PPControl, (void*)"PROS", TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "PP Task");

	setMaxRPMV(500);
	// setArmHeight(1400);

  double smooth = 0.75;
	setEnableControl(true);
  // basePP({position, Node(24, 72)}, 1-smooth, smooth, 18);
  // waitPP(0);
  // enableBase(true, true);
  // baseTurn(-90);
  // waitTurn(10000);

	// setArmPos(1);
	// setNeedleState(HIGH);
	// setNeedleTilterState(LOW);
	setIntake(-127);
	delay(300);
	setIntake(127);
	delay(200);
	baseMove(-1);
	delay(300);
	waitPP(1000);

	baseMove(7.8);
	waitPP(1000);

	setNeedleState(HIGH);
	setArmPos(1);
	delay(700);
	setNeedleTilterState(LOW);
	delay(700);
	setNeedleState(LOW);
	delay(1000);

	setArmPos(0);
	setNeedleTilterState(HIGH);
	baseMove(13);
	waitPP(1000);

	baseMove(-20);
	waitPP(1000);

	enableBase(true, false);
	baseTurn(115);
	waitTurn(1500);

	// enableBase(false, true);
	// baseTurn(150);
	// waitTurn(1500);

	setClampState(LOW);
	basePP({position, Node(-8, 59)}, 1-smooth, smooth, 14, true);
	waitPP(2000);
	setClampState(HIGH);

	baseMove(14);
	waitPP(2000);

	// baseTurn(calcBaseTurn(-24, 0, true));
	// waitTurn(1500);


	enableBase(true, true);
	baseTurn(360);
	waitTurn(1500);

	setClampState(LOW);
	delay(300);
	setClampState(LOW);

	setNeedleState(HIGH);
	setArmPos(1);
	setNeedleTilterState(LOW);
	basePP({position, Node(-14.5, (120-23.7))}, 1-smooth, smooth, 13, false);

	while(millis() - start < 14900) {
		printf("time: %.2fs\n", millis() - start);
		delay(5);
	}
	setEnableControl(false);
	controlTask.remove();
	setNeedleState(LOW);
	// setIntake(127);
	printf("program finished in %.2fs", start - millis());
}

void autonomous() {
	// blue: opp goal center and as towards middle as possible
	// red:: opp goal further and not covering head
	setHookState(LOW);
	red();
}

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

	setEnableControl(false);

	bool tankDrive = true;
	bool manual = false;
	int armPos = 0;
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

		FL.move(left);
		ML.move(left);
		BL.move(left);
		FR.move(right);
		MR.move(right);
		BR.move(right);

		setIntake((master.get_digital(DIGITAL_R1) - master.get_digital(DIGITAL_R2))*127);

		// if(master.get_digital_new_press(DIGITAL_UP)) toggleTilterState();
		if(master.get_digital_new_press(DIGITAL_DOWN)) toggleClampState();
		if(master.get_digital_new_press(DIGITAL_X)) toggleNeedleTilterState();
		if(master.get_digital_new_press(DIGITAL_B)) toggleNeedleState();
		if(master.get_digital_new_press(DIGITAL_UP)) toggleHookState();

		if(master.get_digital_new_press(DIGITAL_A)) manual = !manual;

		if(master.get_digital_new_press(DIGITAL_A)) toggleArmManual();
		if(master.get_digital_new_press(DIGITAL_L1) && armPos < 4) setArmPos(++armPos);
		else if(master.get_digital_new_press(DIGITAL_L2) && armPos > 0) setArmPos(--armPos);

		delay(5);
  }
}
