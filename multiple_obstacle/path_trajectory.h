#ifndef PATHTRAJECTORY
#define PATHTRAJECTORY

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <string>
#include <math.h>
#include <cmath>

#define _USE_MATH_DEFINES

using namespace Eigen;
using namespace std;

class PathTrajectory{
public:

	PathTrajectory(string path_type,vector<VectorXf> param, float r = 0.05, float mult = 20.0, float phi_ = 0.0){
		if(path_type.compare("linear") == 0){
			path = path_type;
			pin = param[0];
			p_fin = param[1];
			L = sqrt(pow(p_fin(0,0)-pin(0,0),2) + pow(p_fin(1,0)-pin(1,0),2) + pow(p_fin(2,0)-pin(2,0),2));
		}
		if(path_type.compare("circular") == 0){
			path = path_type;
			R = r;
			multiplier = mult;
			phi = phi_;
			C = param[0];
		}
		if(path_type.compare("regulation") == 0){
			path = path_type;
			points = param;
		}
	}

	VectorXf p_d(float t);
	VectorXf p_dot_d(float t);
	string get_path(){
		return path;
	}
	float get_end_condition(VectorXf current_position){
		if(path.compare("linear") == 0){
			return L;
		}
		if(path.compare("circular") == 0){
			return 360;
		}
		if(path.compare("regulation") == 0){
			// If the current target is the last point passed in initialization and the error norm between current position and target is less than threshold then return that we fininshed
			if(current_point == points.size() - 1 && (current_position - points[current_point]).norm() < 0.03){
				return 1;
			}
			// If the target is not the last point in the vector but error norm is less than threshold than we reached the target point and we need to reach the new one
			if((current_position - points[current_point]).norm() < 0.03){
				current_point = current_point + 1;
			}
			// But we did not fininsh the regulation problem so we keep going on
			return 0;
		}
		return -100.0;
	}
private:
	//Attributes of the path
	float L;
	float R;
	float multiplier;
	float phi;
	string path;
	VectorXf pin;
	VectorXf p_fin;
	VectorXf C;
	vector<VectorXf> points;
	int current_point = 0;
};

#endif