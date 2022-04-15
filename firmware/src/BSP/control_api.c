/*
	All functions to be called from lesser priority task than StepperCtrl !
*/
#include "control_api.h"

#include "stepper_controller.h"
#include "sine.h"

extern volatile bool StepperCtrl_Enabled;
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

extern volatile SystemParams_t systemParams;

#define CHOOSE_DIR(x) ((systemParams.dirRotation==CW_ROTATION) ? (x) : (-x))	//short hand for swapping direction

void StepperCtrl_setDesiredLocation(int32_t deltaLocation){
	disableTCInterrupts(); //reading from a global may result in partial data if called from outside
	desiredLocation = StepperCtrl_getCurrentLocation() + CHOOSE_DIR(deltaLocation);
	enableTCInterruptsCond(StepperCtrl_Enabled);
}

void StepperCtrl_setFeedForwardTorque(int16_t Iq_feedforward){ //set feedforward torque
	disableTCInterrupts(); //reading from a global may result in partial data if called from outside
	feedForward = CHOOSE_DIR(Iq_feedforward);
	enableTCInterruptsCond(StepperCtrl_Enabled);
}

void StepperCtrl_setCloseLoopTorque(uint16_t Iq_closeloopLim){ //set error correction max torque
	disableTCInterrupts(); //reading from a global may result in partial data if called from outside
	closeLoopMax = (int_fast16_t) min(Iq_closeloopLim, INT16_MAX); //keep as absolute value
	enableTCInterruptsCond(StepperCtrl_Enabled);
}

void StepperCtrl_setControlMode(uint8_t mode){ 
	switch (mode){
		case 0:
			StepperCtrl_feedbackMode(STEPCTRL_OFF);
			break;
		case 1:
			StepperCtrl_feedbackMode(STEPCTRL_FEEDBACK_POSITION_RELATIVE);
			break;
		default:
			StepperCtrl_feedbackMode(STEPCTRL_FEEDBACK_SOFT_TORQUE_OFF);
			
			break;
	}
}

int32_t StepperCtrl_getCurrentLocation(void) {
	int32_t ret;
	disableTCInterrupts(); 
	ret = currentLocation;
	enableTCInterruptsCond(StepperCtrl_Enabled);
	return ret;
}

int16_t StepperCtrl_getCloseLoop(void) {
	int16_t ret;
	disableTCInterrupts(); 
	ret = (int16_t) CHOOSE_DIR(closeLoop);
	enableTCInterruptsCond(StepperCtrl_Enabled);
	return ret;
}

int16_t StepperCtrl_getControlOutput(void) {
	int16_t ret;
	disableTCInterrupts(); 
	ret = CHOOSE_DIR(control);
	enableTCInterruptsCond(StepperCtrl_Enabled);
	return ret;
}

int32_t StepperCtrl_getSpeedRev(void) { //revolutions/s
	int32_t ret;
	disableTCInterrupts(); 
	ret = CHOOSE_DIR(speed_slow) / (int32_t) (ANGLE_STEPS);
	enableTCInterruptsCond(StepperCtrl_Enabled);
	return ret;
}

int32_t StepperCtrl_getPositionError(void) {
	int32_t ret;
	disableTCInterrupts(); 
	ret = CHOOSE_DIR(loopError);
	enableTCInterruptsCond(StepperCtrl_Enabled);
	return ret;
}
