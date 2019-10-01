#include <iostream>
#include "Manipulator.h"
#include <Eigen/Dense>
#include <array>

using namespace std;

int main() {
	MatrixXf DH(3, 4);
	DH << 0, 1, 0, 1,
			0, 1, 0, 1,
			0, 1, 0, 1;
	Manipulator man{DH};

	Vector3f obst1(0, 0, 0);
	Vector3f obst2(1, 1, 0);

	cout << "______________\n" << "First obstacle in:\n";
	cout << obst1 << endl;
	cout << "______________\n" << "Second obstacle in:\n";
	cout << obst2 << endl;

	Manipulator::newObst(obst1);
	cout << "______________\n Obst1 added to the stack\n";
	Manipulator::newObst(obst2);
	cout << "______________\n Obst2 added to the stack\n";

	/*CONFIGURATIONS*/

	array<Vector3f, 4> config;
	config[0] << 0, 0, 0;
	config[1] << 0, 0, M_PI / 2;
	config[2] << 0, M_PI / 2, 0;
	config[3] << M_PI / 2, 0, 0;

	/*OBSTACLE DISTANCES*/
	for (int j = 0; j < 4; ++j) {
		cout << "---------------------" << endl << endl;
		cout << "Joint configuration:\n";
		cout << config[j] << endl;
		cout << "---------------------" << endl << endl;
		for (int i = 0; i < 2; ++i) {
			cout << "Distance vector of obst" << i + 1 << " is:\n";
			cout << man.eeDisVec(config[j], i) << endl;
			cout << "With length: " << man.eeDis(config[j], i) << endl;
		}
	}
	return 0;
}