#ifndef PID_H
#define PID_H

class PID {
	float Kp;
	float Ki;
	float Kd;
	float uk_1;
	float uk;
	float yk_1;
	float yk;
	float tau;

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
	float PID_UD(float setpoint, float measurement);

};


#endif
