#ifndef PID2_H
#define PID2_H
#include <math.h>
#include <avr/io.h>
#define DEFAULT_MAXERROR 10e8
#define DEFAULT_MAXSUMERROR 10e12

struct PID_DATA{
	float lastValue;
	float Kp;
	float Ki;
	float Kd;
	float sumError;
	float maxError;
	float maxSumError;
};

void initializePID(struct PID_DATA *pid, float p, float i, float d);

//step takes about 500us on an atmega8 at 20 Mhz
int16_t stepPID(struct PID_DATA *pid, float value, float reference);

void resetSumError(struct PID_DATA *pid);

#endif
