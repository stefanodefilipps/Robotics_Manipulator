#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include "Task.h"

using namespace std;

ostream& operator<<(ostream& s, const vector<Eigen::MatrixXf>& m);
ostream& operator<<(ostream& s, const vector<int>& m);


int main() {
	Eigen::MatrixXf J1(2,2);
	Eigen::MatrixXf J2(3,3);
	Eigen::MatrixXf J3(4,4);
	Eigen::MatrixXf J4(5,5);
	vector<Eigen::MatrixXf> stack_{J1,J2,J3,J4};

	Task stack{stack_};
	cout << "\n--------------------\n" << "| BEFORE SWAP      |\n" << "--------------------\n";
	cout << "__________\n";
	cout << "stack." << "get():\n" << stack.get() << endl;
	cout << "__________\n";
	cout << "stack." << "getStack():\n" << stack.getStack() << endl;
	cout << "__________\n";
	cout << "stack." << "getIndices():\n" << stack.getInd() << endl;
	stack.swapTask(0,3);
	stack.swapTask(1,3);
	cout << "\n--------------------\n" << "|  AFTER SWAP      |\n" << "--------------------\n";
	cout << "__________\n";
	cout << "stack." << "get():\n" << stack.get() << endl;
	cout << "__________\n";
	cout << "stack." << "getStack():\n" << stack.getStack() << endl;
	cout << "__________\n";
	cout << "stack." << "getIndices():\n" << stack.getInd() << endl;
	cout << "\n--------------------\n" << "|  USING () & []   |\n" << "--------------------\n";
	cout << "Non prioritized call () of stack(0) is:\n" << stack(0) << endl ;
	cout << "Even if a swap has been made! As a matter of fact, [] = prioritized subscripting ( stack[0] ) gives:\n" << stack[0] << endl;
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
