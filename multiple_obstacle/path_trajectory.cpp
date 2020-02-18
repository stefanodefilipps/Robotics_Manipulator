#include "path_trajectory.h"

VectorXf PathTrajectory::p_d(float t){
	if(path.compare("linear") == 0){
		float s = t / L;
		return pin + s * (p_fin - pin);
	}
	if(path.compare("circular") == 0){
		VectorXf pos_d(3);
		pos_d << C(0) + R * cos(multiplier*t*M_PI/180.0 + phi*M_PI/180.0),
	    	     0.0,
	    	     C(1) + R * sin(multiplier*t*M_PI/180.0 + phi*M_PI/180.0);
	    return pos_d;
	}
	if(path.compare("regulation") == 0){
		return points[current_point];
	}

	VectorXf pos_d(3);
	pos_d << -100.0,-100.0,-100.0;
	return pos_d;
}

VectorXf PathTrajectory::p_dot_d(float t){
	if(path.compare("linear") == 0){
		return (p_fin - pin) / L;
	}
	if(path.compare("circular") == 0){
		VectorXf v_d(3);
		v_d << -R*sin(multiplier*t*M_PI/180.0 + phi*M_PI/180.0)*multiplier,
				0.0,
				R*cos(multiplier*t*M_PI/180.0 + phi*M_PI/180.0)*multiplier;
	    return v_d;
	}
	if(path.compare("regulation") == 0){
		VectorXf v_d(3);
		v_d << 0.0,0.0,0.0;
		return v_d;
	}

	VectorXf v_d(3);
	v_d << -100.0,-100.0,-100.0;
	return v_d;
}