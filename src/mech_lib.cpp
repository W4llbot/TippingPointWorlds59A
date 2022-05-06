#include "main.h"

// Arm control

const double armHeights[] = {3400, 6000, 8150, 10416, 13000};
double armTarg = armHeights[0], armUKP = 0.05, armDKP= 0.05, armKI = 0.00003, armKD = 0.5, prevArmError = 0, armPower = 0;
bool needleState = LOW, needleTilterState = HIGH, clampState = LOW;
bool armManual = false;

void armControl(void*ignore) {
  Motor arm(armPort);
  arm.set_brake_mode(E_MOTOR_BRAKE_HOLD);
  ADIDigitalOut needleTilter(needleTilterPort);
  ADIDigitalOut needle(needlePort);
  ADIDigitalOut clamp(clampPort);
  ADIDigitalIn clampLimit(clampLimitPort);
  Rotation armRot(armRotPort);

  Controller master(E_CONTROLLER_MASTER);

  double armErrorSum = 0;
  while(true) {
    double armError = armTarg - armRot.get_position();
    armErrorSum += armError * dT;
    if(fabs(armError) < 100) armErrorSum = 0;
    if(fabs(armError) > 2000) armErrorSum = 0;
    double deltaError = armError - prevArmError;
    double targArmPower = (armError>0?armError*armUKP : armError*armDKP) + armErrorSum * armKI + deltaError*armKD + 20;
    // armPower += abscap(targArmPower, 10);

    double armPower;
    if(armManual) armPower = (master.get_digital(DIGITAL_L1) - master.get_digital(DIGITAL_L2))*127;
    else armPower = fmax(targArmPower, -800);
    arm.move(armPower-20);

    prevArmError = armError;
    // printf("Target: %f, Potentiometer: %d, Error: %f, integral: %.2f, torque: %.2f\n", armTarg, armRot.get_position(), armError, armErrorSum*armKI, arm.get_torque());
    // master.print(0, 2, "torque/Nm: %.5f", arm.get_torque());

    needle.set_value(needleState);
    needleTilter.set_value(needleTilterState);
    if(clampLimit.get_new_press()) clampState = HIGH;
    clamp.set_value(clampState);

    delay(5);
  }
}
void setArmHeight(double height) {armTarg = height;}
void setArmPos(int pos) {armTarg = armHeights[pos];}
void toggleArmManual() {
  Rotation armRot(armRotPort);
  armManual = !armManual;
  if(!armManual) setArmHeight(armRot.get_position());
}

void setNeedleState(bool state) {needleState = state;}
void toggleNeedleState() {needleState = !needleState;}

void setNeedleTilterState(bool state) {
  needleTilterState = state;
  needleState = !state;
}
void toggleNeedleTilterState() {setNeedleTilterState(!needleTilterState);}

void setClampState(bool state) {clampState = state;}
void toggleClampState() {clampState = !clampState;}

// Tilter control
// bool tilterState = LOW;
// void tilterControl(void*ignore) {
//   ADIDigitalOut lTilter(lTilterPort);
// 	ADIDigitalOut rTilter(rTilterPort);
//
//   while(true) {
//     lTilter.set_value(tilterState);
//     rTilter.set_value(tilterState);
//
//     delay(5);
//   }
// }
//
// void setTilterState(bool state) {tilterState = state;}
// void toggleTilterState() {tilterState = !tilterState;}

// Intake control
double intakeTarg = 0;
void intakeControl(void*ignore) {
  Motor intake(intakePort);
  while(true) {
    intake.move(intakeTarg);

    delay(5);
  }
}

void setIntake(double pow) {
  if(pow > 100) setNeedleState(LOW);
  intakeTarg = pow;
}

// Hook control
bool hookState = LOW;
void hookControl(void * ignore) {
  ADIDigitalOut hook(hookPort);

  while(true) {
    hook.set_value(hookState);
    // printf("hookState: %d\n", hookState);
    delay(5);
  }
}

void setHookState(bool state) {
  hookState = state;
  if(state) setArmHeight(4000);
}
void toggleHookState() {setHookState(!hookState);}
