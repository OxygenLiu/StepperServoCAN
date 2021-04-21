/*
	All functions to be called from lesser priority task than StepperCtrl !
*/
#include "control_api.h"

#include "stepper_controller.h"
#include "sine.h"

//api - commanded
extern volatile int32_t desiredLocation;
extern volatile int_fast16_t feedForward;
extern volatile int_fast16_t closeLoopMax;

//api - measured
extern volatile int32_t currentLocation;
extern volatile int_fast16_t closeLoop;
extern volatile int16_t control;
extern volatile int32_t speed_slow;
extern volatile int32_t loopError;


void StepperCtrl_setDesiredLocation(int32_t deltaLocation){
	disableTCInterrupts(); //reading from a global may result in partial data if called from outside
	desiredLocation = StepperCtrl_getCurrentLocation() + deltaLocation;
	enableTCInterrupts();
}
void StepperCtrl_setFeedForwardTorque(int16_t Iq_feedforward){ //set feedforward torque
	disableTCInterrupts(); //reading from a global may result in partial data if called from outside
	feedForward = Iq_feedforward;
	enableTCInterrupts();
}

void StepperCtrl_setCloseLoopTorque(uint16_t Iq_closeloopLim){ //set error correction max torque
	disableTCInterrupts(); //reading from a global may result in partial data if called from outside
	closeLoopMax = (int_fast16_t) min(Iq_closeloopLim, INT16_MAX);
	enableTCInterrupts();
}
void StepperCtrl_setControlMode(uint8_t mode){ 
	disableTCInterrupts(); //reading from a global may result in partial data if called from outside
	
	enableTCInterrupts();
}



int32_t StepperCtrl_getCurrentLocation(void) {
	int32_t ret;
	disableTCInterrupts(); 
	ret = currentLocation;
	enableTCInterrupts();
	return ret;
}

int16_t StepperCtrl_getCloseLoop(void) {
	int16_t ret;
	disableTCInterrupts(); 
	ret = (int16_t) closeLoop;
	enableTCInterrupts();
	return ret;
}

int16_t StepperCtrl_getControlOutput(void) {
	int16_t ret;
	disableTCInterrupts(); 
	ret = control;
	enableTCInterrupts();
	return ret;
}

int32_t StepperCtrl_getSpeedRev(void) { //revolutions/s
	int32_t ret;
	disableTCInterrupts(); 
	ret = speed_slow / (int32_t) (ANGLE_STEPS);
	enableTCInterrupts();
	return ret;
}

int32_t StepperCtrl_getPositionError(void) {
	int32_t ret;
	disableTCInterrupts(); 
	ret = loopError;
	enableTCInterrupts();
	return ret;
}