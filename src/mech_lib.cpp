#include "main.h"

// Arm control
const double armHeights[] = {995, 1860, 2400};
double armTarg = armHeights[0], armKP = 0.5;
bool needleState = LOW, needleTilterState = LOW, clampState = LOW;

void armControl(void*ignore) {
  Motor arm(armPort);
  ADIDigitalOut needleTilter(needleTilterPort);
  ADIDigitalOut needle(needlePort);
  ADIDigitalOut clamp(clampPort);
  Rotation armRot(armRotPort);

  while(true) {
    double armError = armTarg - armRot.get_position();
    arm.move(armError*armKP);
    //printf("Target: %f, Potentiometer: %d, Error: %f\n", armTarg, potentiometer.get_value(), armError);

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
void toggleClampState() {clampState = !needleState;}

// Tilter control
bool tilterState;
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
void toggleTilterState(bool state) {tilterState = !tilterState;}

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
