/* This file was made by: Marco Menchetti
 *
 * Write me @: menchetti.1713013@studenti.uniroma1.it
 *
 */
 
 #include<iostream>
 #include<Eigen/Dense>
 #include"Manipulator.h"
 #include<vector>
 #include <array>
 #include<math.h>
 
 using namespace std;
 
 int main() {
 	cout << "Planar 3R example: \n";
 	Eigen::MatrixXf DH(3,4);
	DH << 	0, 1, 0, 1,
		0, 1, 0, 1,
		0, 1, 0, 1;
	Manipulator man(DH);
	Eigen::VectorXf vars(3);
	Eigen::Vector3f obsPos(0,0,0);
	cout << man.eeDisVec(obsPos,vars) << endl;
	cout << "Distance vector in particular configurations\n";
	array<Eigen::Vector3f,4> caseVars;
	caseVars[0] << 0,0,0;
	caseVars[1] << 0,0,M_PI/2;
	caseVars[2] << 0,M_PI/2,M_PI/2;
	caseVars[3] << -M_PI/2,0,0;
	 for (int i = 0; i < 4; ++i) {
		 cout << "Case " << i + 1 << ":\n" << caseVars[i] << endl << endl;
		 cout << "Distance:\n" << man.eeDisVec(obsPos,caseVars[i]) << endl << endl;
		 cout << "Whos norm is: " << man.eeDis(obsPos,caseVars[i]) << endl << endl;
		 cout << "_____________________________________" << endl;
	 }
 	return 0;
 }
 
