/*
This file was made by:
Marco Menchetti.

Write me @ menchetti.1713013@studenti.uniroma1.it

*/

#include<iostream>
#include<Eigen/Dense>
#include"Manipulator.h"
#include<vector>
#include<math.h>
#include <fstream>
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
	vector<float> q{0,-M_PI_2,0,-M_PI_2,0,0,0};
	float t = 0.0;
	float R = 0.0001;
	/* this is the sampling time and is of 10 milliseconds */
	float T = 0.01;
	MatrixXf p_in(3,1);
	MatrixXf J;
	MatrixXf J2(1,7);
	J2 << 1.0,1.0,1.0,1.0,0.0,0.0,0.0;
	MatrixXf b(3,1);
	cout << b << endl;
	MatrixXf b2(1,1);
	b2 << 0;
	vector<MatrixXf> Ji;
	vector<MatrixXf> bi;
	ofstream data_joints_file;
  	data_joints_file.open ("data_joints_file.txt");
  	ofstream data_ee_file;
  	data_ee_file.open ("data_ee_file.txt");
	while(t <= 360.0){
		VectorXf ee_position = man.dKin(q);
		for (int i = 0; i < q.size(); ++i)
		{
			data_joints_file << q[i];
			data_joints_file << ";";
		}
		data_joints_file << "\n";
		for (int i = 0; i < ee_position.size(); ++i)
		{
			data_ee_file << ee_position[i];
			data_ee_file << ";";
		}
		data_ee_file << "\n";
	    J = man.jacobian(q);
		/* I try to use the flacco algorithm*/
		Ji = {J,J2};
		b << -R*sin(t*M_PI/180.0),
			 0.0,
			 R*cos(t*M_PI/180.0);
		bi = {b,b2};
		MatrixXf q_dot = FlaccoPrioritySolution(Ji, bi, 0.1, 0.1);
		/* Now I solve the inverse kynematic problem given the joint velocities and the sampling time*/
		for (int i = 0; i < q.size(); ++i)
		{
			q[i] = q[i] + T*q_dot(i,0);
		}
		t = t + T;
	}
	cout << "FINISHED" << endl;
	data_joints_file.close();
	data_ee_file.close();
}

