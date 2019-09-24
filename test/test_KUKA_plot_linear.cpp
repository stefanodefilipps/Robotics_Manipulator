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
	/* this is the sampling time and is of 10 milliseconds */
	float T = 0.001;
	MatrixXf p_in(3,1);
	p_in << man.dKin(q)[0],
			man.dKin(q)[1],
			man.dKin(q)[2];
	MatrixXf p_fin(3,1);
	p_fin << 0.1,
			 0.2,
			 0.2;
	float L = sqrt(pow(p_fin(0,0)-p_in(0,0),2) + pow(p_fin(1,0)-p_in(1,0),2) + pow(p_fin(2,0)-p_in(2,0),2));
	MatrixXf J;
	MatrixXf J2(1,7);
	J2 << 1.0,1.0,1.0,1.0,0.0,0.0,0.0;
	//MatrixXf J2 =MatrixXf::Zero(3,7);
	//cout << man.GeomJacAngular(q) << endl;
	//J2.block(0,0,3,4) = man.GeomJacAngular(q).block(0,0,3,4);
	cout << "PROVA" << endl;
	MatrixXf b(3,1);
	b = (p_fin - p_in) / L;
	cout << b << endl;
	MatrixXf b2(1,1);
	b2 << 0;
	/*MatrixXf b2(3,1);
	b2 << 0,
		  1,
		  0;*/
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
		for (int i = 0; i < ee_position.size(); ++i)
		{
			data_ee_file << ee_position[i];
			data_ee_file << ";";
		}
		data_ee_file << "\n";
	    J = man.jacobian(q);
	    //J2.block(0,0,3,4) = man.GeomJacAngular(q).block(0,0,3,4);
		/* I try to use the flacco algorithm*/
		Ji = {J,J2};
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

