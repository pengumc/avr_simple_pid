//author: Michiel van der Coelen
//date 2010-10-10

#include "pid.h"
#include <avr/io.h>
#include <math.h>
void initializePID(struct PID_DATA *pid, float p, float i, float d){
	pid->lastValue = 0;
	pid->Kp = p;
	pid->Ki = i;
	pid->Kd = d;
	pid->sumError = 0;
	pid->maxError = DEFAULT_MAXERROR;
	if(i) pid->maxSumError = DEFAULT_MAXSUMERROR/i;
	else pid->maxSumError = DEFAULT_MAXSUMERROR;
}

int16_t stepPID(struct PID_DATA *pid, float value, float reference){
	float error, temp, pFactor, iFactor, dFactor;

	error = reference - value;
	if(error > pid->maxError) error = pid->maxError;
	if(error < -pid->maxError) error = -pid->maxError;
	
	//P
	pFactor = error * pid->Kp;
	
	//I
	temp = pid->sumError + error;
	if(temp > pid->maxSumError) temp = pid->maxSumError;
	if(temp < -pid->maxSumError) temp = -pid->maxSumError;
	pid->sumError = temp;
	iFactor = pid->sumError * pid->Ki;
	
	//D
	dFactor = (value - pid->lastValue)*pid->Kd;
	pid->lastValue = value;
	
	temp = pFactor + iFactor + dFactor;
	
	return ((int16_t) temp);
	
}

void resetSumError(struct PID_DATA *pid){
	pid->sumError =0;
	pid->maxSumError = DEFAULT_MAXSUMERROR/pid->Ki;
}
