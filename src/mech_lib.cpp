#include "main.h"

// Arm control

const double armHeights[] = {3648, 6531, 8895, 10416, 14142};
double armTarg = armHeights[0], armUKP = 0.10, armDKP= 0.05, armKD = 0, prevArmError = 0, armPower = 0;
bool needleState = LOW, needleTilterState = HIGH, clampState = LOW;

double abscap(double x, double abscap){
  if(x > abscap) return abscap;
  else if(x < -abscap) return -abscap;
  else return x;
}

void armControl(void*ignore) {
  Motor arm(armPort);
  arm.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  ADIDigitalOut needleTilter(needleTilterPort);
  ADIDigitalOut needle(needlePort);
  ADIDigitalOut clamp(clampPort);
  Rotation armRot(armRotPort);

  while(true) {
    double armError = armTarg - armRot.get_position();
    double deltaError = armError - prevArmError;
    double targArmPower = (armError>0?armError*armUKP : armError*armDKP) + deltaError*armKD;
    // armPower += abscap(targArmPower, 10);

    arm.move(targArmPower);

    prevArmError = armError;
    printf("Target: %f, Potentiometer: %d, Error: %f\n", armTarg, armRot.get_position(), armError);

    needle.set_value(needleState);
    needleTilter.set_value(needleTilterState);
    clamp.set_value(clampState);

    delay(5);
  }
}
void setArmHeight(double height) {armTarg = height;}
void setArmPos(int pos) {armTarg = armHeights[pos];}

void setNeedleState(bool state) {needleState = state;}
void toggleNeedleState() {needleState = !needleState;}

void setNeedleTilterState(bool state) {needleTilterState = state;}
void toggleNeedleTilterState() {needleTilterState = !needleTilterState;}

void setClampState(bool state) {clampState = state;}
void toggleClampState() {clampState = !clampState;}

// Tilter control
bool tilterState = LOW;
void tilterControl(void*ignore) {
  ADIDigitalOut lTilter(lTilterPort);
	ADIDigitalOut rTilter(rTilterPort);

  while(true) {
    lTilter.set_value(tilterState);
    rTilter.set_value(tilterState);

    delay(5);
  }
}

void setTilterState(bool state) {tilterState = state;}
void toggleTilterState() {tilterState = !tilterState;}

// Intake control
double intakeTarg = 0;
void intakeControl(void*ignore) {
  Motor intake(intakePort);
  while(true) {
    intake.move(intakeTarg);

    delay(5);
  }
}

void setIntake(double pow) {intakeTarg = pow;}
