#include "PID.hpp"
#include <iostream>
#include <vector>
using namespace std;
PID::PID() {


	prevError = 0.0f;
	differentiator = 0.0f;
	prevMeasurement = 0.0f;
	T = 0.0166f;
	tau = 0.0f;
	yk = 0;
	
	Ki = 114.7202f;
	Kd = 0.0f;
	Kp = 6.3329f;
	out = 0.0f;
}	

float PID::PID_UD(float setpoint, float measurement, int i) { //Set point y measurement es en RPS
	error = setpoint - measurement;
	cout << " error =" << error << "   previous error = " << prev[i] << endl;
	//cout << "setpoint = " << setpoint << " measurement = " << measurement << endl;
	propor[i] = Kp * error;
	//cout << "proportional = " << propor[i] << " Kp = " << Kp << endl;
	integr[i] = integr[i] + 0.5f * Ki * 0.0166f * (error + prev[i]);
	
	//cout << "integrator = " << integr[i] << " Ki = " << Ki << endl;
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
	}*/
	if (integr[i] > 80) {
		integr[i] = 80;
	}
	else if (integr[i] < -80) {
		integr[i] = -80;
	}

	differ[i] = (2.0f * Kd * (measurement - prevMeasur[i])
		+ (2.0f * tau - 0.0166f) * differ[i])
		/ (2.0f * tau + 0.0166);
	//cout << "Differentiator = " << differ[i] << "  Kd = "<< Kd<< endl;
		out = propor[i] + integr[i] + differ[i];
		
		if (out > 50.0f) {
			out = 50.0f;
		}
		else if (out < -50.0f) {
			out = -50.0f;
		}
	//	cout << "PID out = " << out << endl;
		prev[i] = error;
		prevMeasur[i] = measurement;
		uk[i] = out;
		
		
		yk = uk[i] * 0.02864f + uk_1[i] * 0.0286f + yk_1[i] * 0.7469f;
		uk_1[i] = uk[i];
		//cout << "yk = " << yk << " previous yk = " << yk_1[i] << endl;
		yk_1[i] = yk;
		

			return yk;

		
		

}