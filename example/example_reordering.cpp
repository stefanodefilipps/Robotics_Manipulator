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
	Eigen::Vector3f q_in{0,0,-M_PI/2};
	Manipulator man{DH,q_in};           			/*Manipulator*/
    Eigen::VectorXf ks(3);
    ks << 20.0,20.0,20.0;
    vector<Eigen::Vector3f> obstPos;
    Vector3f oPos{2,1,0};
    obstPos.push_back(oPos);
    FlaccoController contr(0,0,0,ks,obstPos,1.42,1); 			/*Controller*/
	Eigen::MatrixXf J1(2,2);
	Eigen::MatrixXf J2(3,3);
	Eigen::MatrixXf J3(4,4);
	Eigen::MatrixXf J4(5,5);
	vector<Eigen::MatrixXf> stack_{J1,J2,J3,J4};
	Task<Eigen::MatrixXf> stack{stack_};									/*stack of task*/
	vector<int> ctrP{3,2,1};							/*Control point definition*/
	man.setCtrPtsJoints(ctrP);
	vector<Vector3f> ctrPosition = man.controlPoints();	/*Control point positions*/
	cout << "Indices before task Reordering:\n" << stack.getInd() << endl;
	//cout << "Stack before task Reordering:\n" << stack.getStack() << endl;
	contr.taskReorder(stack,ctrPosition);
	cout << "Indices after task Reordering:\n" << stack.getInd() << endl;
	//cout << "Stack after task Reordering:\n" << stack.getStack() << endl;
	cout << "\n---------------------\n";
	cout << "Max distances are:\td = 1.42\tD = 1\nWhile distances were:\n";
	for (int j = 0; j < 3; ++j) {
		cout << contr.eeDis(ctrPosition[j]) << endl;
	}
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