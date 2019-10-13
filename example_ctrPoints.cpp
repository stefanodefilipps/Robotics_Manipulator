#include <iostream>
#include <Eigen/Dense>
#include "Manipulator.h"

using namespace std;

ostream& operator<<(ostream& s, const vector<float>& vect);
ostream& operator<<(ostream& s, const vector<Vector3f>& vect);


int main() {
	MatrixXf DH(4,4);
	DH <<	0, 1, 0, 1,
			0, 1, 0, 1,
			0, 1, 0, 1,
			0, 1, 0, 1;
	Vector4f q_in{0,0,0,0};

	Manipulator man{DH,q_in};

	vector<int> cPoints{4,2,1};
	man.setCtrPtsJoints(cPoints);

	cout << "The control points positions are:";
	vector<Vector3f> pos{man.controlPoints()};
	for (int i = 0; i < 3; ++i) {
		cout << "\n____________________\n" << "ctrPoint " << i+1 << ":\n";
		cout << pos[i] << endl;
	}

	return 0;
}

ostream& operator<<(ostream& s, const vector<float>& vect) {
	for (int i = 0; i < static_cast<int>(vect.size()); ++i) {
		s << vect[i] << "\n";
	}
	return s;
}

ostream& operator<<(ostream& s, const vector<Vector3f>& vect) {
	for (int i = 0; i < static_cast<int>(vect.size()); ++i) {
		s << vect[i] << "\n";
	}
	return s;
}
