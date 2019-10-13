#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include "Manipulator.h"
#include "Task.h"
#include "flaccoController.h"

using namespace std;

ostream& operator<<(ostream& s, const vector<Eigen::MatrixXf>& m);
ostream& operator<<(ostream& s, const vector<int>& m);

int main() {
	Eigen::MatrixXf DH(3,4);
	DH << 0, 1, 0, 1,
		  0, 1, 0, 1,
		  0, 1, 0, 1;
	Vector3f q_in{0,0,0};
	Manipulator man{DH,q_in};           			/*Manipulator*/
	FlaccoController contr{0,0,0,q_in}; 			/*Controller*/
	Eigen::MatrixXf J1(2,2);
	Eigen::MatrixXf J2(3,3);
	Eigen::MatrixXf J3(4,4);
	Eigen::MatrixXf J4(5,5);
	vector<Eigen::MatrixXf> stack_{J1,J2,J3,J4};
	Task stack{stack_};								/*stack of task*/
	Vector3f oPos{0,1,0};
	contr.newObst(oPos);							/*Obstacle position*/
	vector<Vector3f> ctrP;							/*Control point positions: TODO in Manipulator*/
	for (int i = 0; i < 3; ++i) {
		ctrP.push_back(man.dKin(q_in,i));
	}
	cout << "Indices before task Reordering:\n" << stack.getInd() << endl;
	cout << "Stack before task Reordering:\n" << stack.getStack() << endl;
	contr.taskReorder(stack,ctrP,1.4,1);
	cout << "Indices after task Reordering:\n" << stack.getInd() << endl;
	cout << "Stack after task Reordering:\n" << stack.getStack() << endl;
	return 0;
}


ostream& operator<<(ostream& s, const vector<Eigen::MatrixXf>& m) {
	for (int i = 0; i < static_cast<int>(m.size()); ++i) {
		s << i+1 << ":\n" << m[i] << "\n";
	}
	return s;
}

ostream& operator<<(ostream& s, const vector<int>& m) {
	for (int i = 0; i < static_cast<int>(m.size()); ++i) {
		s << m[i] << "\n";
	}
	return s;
}