/*
This file was made by:
Stefano De Filippis.

Write me @ defilippis.1707247@studenti.uniroma1.it

*/

#include<iostream>
#include<Eigen/Dense>
#include"Manipulator.h"
#include<vector>
#include<math.h>
#include "flacco_impl.h"

#define _USE_MATH_DEFINES
 
#include <cmath>

using namespace std;

int main() {
	cout << "KUKA 7 DOPF EXAMPLE:\n\n";
	Eigen::MatrixXf DH(7,4);
	float d1 = 0.4;
	float d2 = 0.39;
	DH << M_PI_2, 0, 0, 1,
		 -M_PI_2, 0, 0, 1,
		 -M_PI_2, 0, d1,1,
		  M_PI_2, 0, 0, 1,
		  M_PI_2, 0, d2,1,
		 -M_PI_2, 0, 0, 1,
		       0, 0, 0, 1;
	Manipulator man(DH);
	vector<float> v{0,0,0,0,0,0,0};
	cout << "Manipulator direct kinematics in nominal configuration is: "<< endl;
	cout << man.dKin(v) << endl;
	cout << "\nManipulator jacobian in nominal configuration: " << endl;
	cout << man.jacobian(v) << endl;
	
	cout << "\nType the joint variable of the configuration you're interested in: ";
	cin >> v[0] >> v[1] >> v[2] >> v[3] >> v[4] >> v[5] >> v[7];
	
	cout << "Manipulator direct kinematics in the given configuration is: "<< endl;
	cout << man.dKin(v) << endl;
	cout << "\nManipulator jacobian in the given configuration: " << endl;
	MatrixXf J = man.jacobian(v);
	cout << J << endl;
	/* I try to use the flacco algorithm*/
	MatrixXf J2(1,7);
	J2 << 0.0,1.0,0.0,1.0,0.0,0.0,0.0;
	vector<MatrixXf> Ji = {J2,J};
	float t = 0.0;
	MatrixXf b(3,1);
	b << 3.0,
		 0.0,
		 0.0;
	MatrixXf b2(1,1);
	b2 << 0.0;
	vector<MatrixXf> bi = {b2,b};
	MatrixXf q_dot = FlaccoPrioritySolution(Ji, bi, 0.1, 0.1);
	cout << "SOLUTION" << endl;
	cout << q_dot << endl;
	cout << "TASK 1" << endl; 
	cout << J * q_dot << endl;
	cout << "TASK 2" << endl;
	cout << J2 * q_dot << endl;
}

