#ifndef _MECH_LIB_HPP_
#define _MECH_LIB_HPP_

void armControl(void*ignore);
void setArmHeight(double height);
void setArmPos(int pos);
void toggleArmManual();

void setNeedleState(bool state);
void toggleNeedleState();
void setNeedleTilterState(bool state);
void toggleNeedleTilterState();
void setClampState(bool state);
void toggleClampState();

// void tilterControl(void*ignore);
// void setTilterState(bool state);
// void toggleTilterState();

void intakeControl(void*ignore);
void setIntake(double pow);


#endif
