/*
This file was made by:
Marco Menchetti.

Write me @ menchetti.1713013@studenti.uniroma1.it

*/

#include"Manipulator.h"

/*PUBLIC*/

MatrixXf Manipulator::jacobian(const VectorXf& q0, float eps) const {
	/*JACOBIAN DIMENTION ASSIGNEMENT*/
	MatrixXf J(3,static_cast<int> (q0.size()));
	
	/*COLUMN WISE ASSIGNEMENT OF JACOBIAN'S ELEMENTS*/
	for (unsigned int i{0}; i < static_cast<int> (q0.size()); i++) {
		VectorXf qEps{q0};
		qEps[i] = q0[i] + eps;
		J.col(i) = (dKin(qEps) - dKin(q0))/eps;
	}
	return J;
}


Vector3f Manipulator::eeDisVec(const VectorXf &vars, const int numberOfObstacle) const {
	return dKin(vars) - obstPos[numberOfObstacle];
}

float Manipulator::eeDis(const VectorXf& var, const int numberOfObstacle) const {
	Vector3f d{eeDisVec(var,numberOfObstacle)};
	return sqrt(d.transpose()*d);
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
   	float a{DH(i,1)};
   	float d{( DH(i,3) == 1 ? DH(i,2) : var)};
   	float th{( DH(i,3) == 1 ? var : DH(i,2))};
   	
   	/*HOMOGENEOUS TRANSFORMATION MATRIX*/
   	H << cos(th), -cos(al)*sin(th),  sin(al)*sin(th), a*cos(th),
   	     sin(th),  cos(al)*cos(th), -sin(al)*cos(th), a*sin(th),
    		   0,          sin(al),          cos(al),         d,
    	       0,                0,                0,         1;
	return H;
}

Vector4f Manipulator::dKinAlg(const VectorXf& vars, int upToNJoint, const int i) const {
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
    else return v;
}

/* STATIC */
bool Manipulator::isObstacle{0};
std::vector<Vector3f> Manipulator::obstPos;

void Manipulator::newObst(const Vector3f newPos) {
	if(!isObstacle) isObstacle = 1;
	obstPos.push_back(newPos);
}
