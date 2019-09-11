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
#include <fstream>
#include "flacco_impl.h"

#define _USE_MATH_DEFINES
 
#include <cmath>

using namespace std;
using namespace Eigen;

int main() {
	cout << "5R PLANAR EXAMPLE:\n\n";
	MatrixXf DH(5,4);
	DH << 0, 1, 0, 1,
		  0, 1, 0, 1,
		  0, 1, 0, 1,
		  0, 1, 0, 1,
		  0, 1, 0, 1;
	Manipulator man(DH);
	vector<float> q{0,M_PI_2,-M_PI_2,0,0};
	float t = 0.0;
	/* this is the sampling time and is of 10 milliseconds */
	float T = 0.01;

	/* This is the radius ot the circular path
	   OBS: here I need to investigate more because i think i have some problem maybe with the formula. The radius infact actually represent the diameter of the final
	   circular path described by the end effector and in this case with 0.02 I am defining a circular path of diamaeter 2 and radius 1*/

	float R = 0.02;
	MatrixXf J;
	MatrixXf J2(1,5);

	/* The second task consist in keeping the second arm link always vertical. This means theta the second link ABSOLUTE angle needs to remains at 90°.
	   Since the angles are measured in trelative reference frames, the absolute angle of the second link is given by q1 + q2.
	   I need to have q1 + q2 = 90° so the direct kynematics of this second task only depends on q1 and q2 (f2(q1,q2) = 90°) and the differential kynematics is given by
	   q1_dot + q2_dot = 0 and so the jacobian form is given by 0 = (1 1 0 0 0)*q_dots */

	J2 << 1.0,1.0,0.0,0.0,0.0;
	MatrixXf b(3,1);
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
		for (int i = 0; i < ee_position.size()-1; ++i)
		{
			data_ee_file << ee_position[i];
			data_ee_file << ";";
		}
		data_ee_file << "\n";
	    J = man.jacobian(q);
		/* I try to use the flacco algorithm*/
		Ji = {J2,J};

		/* A parametrized equation for a circular path is given by p(s) = C + R*(cos(s) sin(s))^T and deriving with respect to time I get p_dot(t) = R*(-sin(s) cos(s))^T * s_dot
		   and in this simple case I chose s(t) = t so the task velocity is given simply by R*(-sin(s) cos(s))^T */
		
		b << -R*sin(t*M_PI/180.0),
			 R*cos(t*M_PI/180.0),
			 0.0;
		bi = {b2,b};
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
