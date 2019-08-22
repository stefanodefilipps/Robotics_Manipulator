/*
This file was made by:
Marco Menchetti.

Write me @ menchetti.1713013@studenti.uniroma1.it

*/

#include"Manipulator.h"

Vector4f Manipulator::dKinAlg(const std::vector<float>& vars,const int i) const {
    Vector4f v{0,0,0,1};
	Matrix4f H;
	
	/*CHECK ON DIMENTIONS*/
	if(vars.size() != nJoints) {
		std::cout << "Error\n";
		return v;
	}
	
    float al{DH(i-1,0)};
   	float  a{DH(i-1,1)};
   	float  d{( DH(i-1,3) == 1 ? DH(i-1,2) : vars[i-1])};
   	float  th{( DH(i-1,3) == 1 ? vars[i-1] : DH(i-1,2))};
   	
   	/*HOMOGENEOUS TRANSFORMATION MATRIX*/
   	H << cos(th), -cos(al)*sin(th),  sin(al)*sin(th), a*cos(th),
   	     sin(th),  cos(al)*cos(th), -sin(al)*cos(th), a*sin(th),
    		   0,          sin(al),          cos(al),         d,
    	       0,                0,                0,         1;
    	       
    /*RECURSION*/
    if(i == nJoints) {
    	return H*v;
    } else if(i < nJoints && i >= 1) {
    	return H*dKinAlg(vars, i + 1);
    }
}

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
