#include "Arduino.h"
#include <PID.h>

PID::PID(float *input_, float *setpoint_, float *output_, float kP_, float kI_, float kD_, float outMin_, float outMax_, long dT_) {
	input = input_;
	setpoint = setpoint_;
	output = output_;
	dT = dT_;
	kP = kP_;
	kI = kI_* (dT / 1000.);
	kD = kD_ / (dT / 1000.);
	outMax = outMax_;
	outMin = outMin_;
	lastTime = 0;
	online = true;
}

PID::PID() {}

void PID::update(long time) {
	if ((time - lastTime) >= dT && online) {
		
		error = (*setpoint) - (*input);
		integral += kI * error;
		
		if(integral > outMax) integral = outMax;
		else if(integral < outMin) integral = outMin;
		
		(*output) = kP * error + integral - kD * (error - lastInput);
		lastInput = error;
		
		if(*output > outMax) *output = outMax;
		else if(*output < outMin) *output = outMin;
		
	}
}

void PID::setMode(int mode) {
	if((mode == AUTOMATIC) && !online) {
		lastInput = *input;
		integral = *output;
		if(integral > outMax) integral = outMax;
		else if(integral < outMin) integral = outMin;
	}
	online = (mode == AUTOMATIC);
}