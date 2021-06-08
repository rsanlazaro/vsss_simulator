#include "PID.hpp"


PID::PID() {

	integrator = 0.0f;
	prevError = 0.0f;
	differentiator = 0.0f;
	prevMeasurement = 0.0f;
	uk = 0.0f;
	uk_1 = 0.0f;
	yk_1 = 0.0f;
	yk = 0.0f;

	out = 0.0f;
}




float PID::PID_UD(float setpoint, float measurement) { //Set point y measurement es en RPS
	float error = setpoint - measurement;
	
	float proportional = Kp * error;
	integrator = integrator + 0.5f * Ki * T * (error + prevError);

	float limMinInt, limMaxInt;

	if (limMax > proportional) {
		limMaxInt = limMax - proportional;
	}else{
		limMaxInt = 0.0f;
	}
	if (limMin < proportional) {
		limMinInt = limMin - proportional;
	}
	else {
		limMinInt = 0.0f;
	}
	if (integrator > limMaxInt) {
		integrator = limMaxInt;
	}
	else if (integrator < limMinInt) {
		integrator = limMinInt;
	}

	differentiator = (2.0f * Kd * (measurement - prevMeasurement)
		+ (2.0f * tau - T) * differentiator)
		/ (2.0f * tau + T);

		out = proportional + integrator + differentiator;

		if (out > limMax) {
			out = limMax;
		}
		else if (out < limMin) {
			out = limMin;
		}
		prevError = error;
		prevMeasurement = measurement;
		uk = out;
		yk = uk_1 * (uk * 0.02864f + uk_1 * 0.0286f) + yk_1 * 0.7469f;
		uk_1 = uk;
		yk_1 = yk;



			return yk;

		
		

}