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

using namespace std;

int main() {
	cout << "PLANAR 3R EXAMPLE:\n\n";
	Eigen::MatrixXf DH(3,4);
	DH << 0, 1, 0, 1,
		  0, 1, 0, 1,
		  0, 1, 0, 1;
	Manipulator man(DH);
	vector<float> v{0,0,0};
	cout << "Manipulator direct kinematics in nominal configuration is: "<< endl;
	cout << man.dKin(v) << endl;
	cout << "\nManipulator jacobian in nominal configuration: " << endl;
	cout << man.jacobian(v) << endl;
	
	cout << "\nType the joint variable of the configuration you're interested in: ";
	cin >> v[0] >> v[1] >> v[2];
	
	cout << "Manipulator direct kinematics in the given configuration is: "<< endl;
	cout << man.dKin(v) << endl;
	cout << "\nManipulator jacobian in the given configuration: " << endl;
	cout << man.jacobian(v) << endl;
}

