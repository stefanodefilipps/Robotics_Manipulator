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
	MatrixXf p_in(3,1);

	/* In this case one of the task is to follow a linear trajectory from the initial ee position to an arbitrary final position*/

	p_in << man.dKin(q)[0],
			man.dKin(q)[1],
			man.dKin(q)[2];
	MatrixXf p_fin(3,1);
	p_fin << 0,
			 2,
			 0;

	/* This is the total length of the path*/

	float L = sqrt(pow(p_fin(0,0)-p_in(0,0),2) + pow(p_fin(1,0)-p_in(1,0),2));
	MatrixXf J;

	/* The second task consist in keeping the second arm link always vertical. This means theta the second link ABSOLUTE angle needs to remains at 90°.
	   Since the angles are measured in trelative reference frames, the absolute angle of the second link is given by q1 + q2.
	   I need to have q1 + q2 = 90° so the direct kynematics of this second task only depends on q1 and q2 (f2(q1,q2) = 90°) and the differential kynematics is given by
	   q1_dot + q2_dot = 0 and so the jacobian form is given by 0 = (1 1 0 0 0)*q_dots */

	MatrixXf J2(1,5);
	J2 << 1.0,1.0,0.0,0.0,0.0;
	MatrixXf b(3,1);

	/* The parametrization of a linear path is given by p(s) = p_in + s*(p_fin - p_in) with s a paremeter between 0 and 1 and set as sigma/L with sigma a value between 0 and L
	   and dependent on time (sigma(t)). Therefore differentiating with respect time I get that the task desired velocities is given by r_dot = (p_fin-p_in)/L * sigma_dot. In 
	   this simple case I consider sigma(t) = t and therefore the derivative is a constant value simply given by (p_fin-p_in)/L*/
	
	b = (p_fin - p_in) / L;
	MatrixXf b2(1,1);
	b2 << 0;
	vector<MatrixXf> Ji;
	vector<MatrixXf> bi;
	ofstream data_joints_file;
  	data_joints_file.open ("data_joints_file.txt");
  	ofstream data_ee_file;
  	data_ee_file.open ("data_ee_file.txt");
	while(t <= L){
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
		bi = {b2,b};
		/* I compute the joint velocities by using the task priority matrix method*/
		MatrixXf q_dot = FlaccoPrioritySolution(Ji, bi, 0.1, 0.1);
		/* Now I solve the inverse kynematic problem given the joint velocities and the sampling time using a simple local method*/
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
