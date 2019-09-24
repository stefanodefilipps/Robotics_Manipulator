/*
This file was made by:
Marco Menchetti.

Write me @ menchetti.1713013@studenti.uniroma1.it

*/

#include"Manipulator.h"

/*PUBLIC*/

MatrixXf Manipulator::jacobian(const std::vector<float>& q0, float eps) const {
	/*JACOBIAN DIMENTION ASSIGNEMENT*/
	MatrixXf J(3,static_cast<int> (q0.size()));
	
	/*COLUMN WISE ASSIGNEMENT OF JACOBIAN'S ELEMENTS*/
	for (unsigned int i{0}; i < static_cast<int> (q0.size()); i++) {
		std::vector<float> qEps{q0};
		qEps[i] = q0[i] + eps;
		J.col(i) = (dKin(qEps) - dKin(q0))/eps;
	}
	return J;
}

/*PRIVATE*/

Matrix3f Manipulator::RMat(const float var, const int i) const {
	Matrix3f R;
	Matrix4f H(HTMat(var,i));
	for(int i{0}; i<3; i++) {
		for (int j{0}; j < 3; j++) {
			R << H(i,j);
		}
	}
	return R;
}

Matrix4f Manipulator::HTMat(const float var, const int i) const {
	Matrix4f H;
    float al{DH(i,0)};
   	float  a{DH(i,1)};
   	float  d{( DH(i,3) == 1 ? DH(i,2) : var)};
   	float  th{( DH(i,3) == 1 ? var : DH(i,2))};
   	
   	/*HOMOGENEOUS TRANSFORMATION MATRIX*/
   	H << cos(th), -cos(al)*sin(th),  sin(al)*sin(th), a*cos(th),
   	     sin(th),  cos(al)*cos(th), -sin(al)*cos(th), a*sin(th),
    		   0,          sin(al),          cos(al),         d,
    	       0,                0,                0,         1;
	return H;
}

Vector4f Manipulator::dKinAlg(const std::vector<float>& vars, int upToNJoint, const int i) const {
	upToNJoint = upToNJoint == 0 ? nJoints : upToNJoint;
    Vector4f v{0,0,0,1};
	Matrix4f H;
	
	/*CHECK ON DIMENTIONS*/
	if(vars.size() != nJoints) {
		std::cout << "Error\n";
		return v;
	}
	H = HTMat(vars[i],i);
	//std::cout << "HTMat @ " << i << "-th iteration: \n" << H << std::endl;
    /*RECURSION*/
    if(i == upToNJoint - 1) {
    	return H*v;
    } else if(i < upToNJoint - 1 && i >= 0) {
    	return H*dKinAlg(vars, upToNJoint, i + 1);
    }
}