#ifndef PID_H
#define PID_H
#include <vector>

class PID {
	float Kp;
	float Ki;
	float Kd;
	float yk;
	float tau;
	float error;

	float prev[6]={ 0,0,0,0,0,0 };
	float yk_1[6] = { 0,0,0,0,0,0 };
	float uk[6] = { 0,0,0,0,0,0 };
	float uk_1[6] = { 0,0,0,0,0,0 };
	float integr[6] = { 0,0,0,0,0,0 };
	float differ[6] = { 0,0,0,0,0,0 };
	float propor[6] = { 0,0,0,0,0,0 };
	float prevMeasur[6] = { 0,0,0,0,0,0 };
	float limMin;
	float limMax;

	


	float T;
	
	float integrator;
	float prevError;
	float differentiator;
	float prevMeasurement;

	float out;
	public:
	PID();
	float PID_UD(float setpoint, float measurement, int index);

};


#endif
