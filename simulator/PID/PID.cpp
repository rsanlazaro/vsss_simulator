#include "PID.hpp"
#include <iostream>
using namespace std;
PID::PID() {

	integrator = 0.0f;
	prevError = 0.0f;
	differentiator = 0.0f;
	prevMeasurement = 0.0f;
	uk = 0.0f;
	uk_1 = 0.0f;
	yk_1 = 0.0f;
	yk = 0.0f;
	Ki = 6.3329f;
	Kd = 0.0f;
	Kp = 6.3329f;
	out = 0.0f;
}	
	




float PID::PID_UD(float setpoint, float measurement) { //Set point y measurement es en RPS
	float error = setpoint - measurement;
	
	float proportional = Kp * error;
	
	integrator = integrator + 0.5f * Ki * 0.0166f * (error + prevError);
	
	float limMinInt, limMaxInt;

	/*if (limMax > proportional) {
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
	}*/

	differentiator = (2.0f * Kd * (measurement - prevMeasurement)
		+ (2.0f * tau - 0.0166f) * differentiator)
		/ (2.0f * tau + 0.0166);
	
		out = proportional + integrator + differentiator;
		//cout << out << endl;
		if (out > 10.0f) {
			out = 10.0f;
		}
		else if (out < -10.0f) {
			out = -10.0f;
		}
		
		prevError = error;
		prevMeasurement = measurement;
		uk = out;
		yk = uk_1 * (uk * 0.02864f + uk_1 * 0.0286f) + yk_1 * 0.7469f;
		uk_1 = uk;
		yk_1 = yk;
		cout << "error = "<<error << "   previous error = " << prevError<< endl;


			return yk;

		
		

}