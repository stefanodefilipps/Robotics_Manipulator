/*
This file was made by:
Marco Menchetti.
Write me @ menchetti.1713013@studenti.uniroma1.it
*/

#include"Manipulator.h"

/*PUBLIC*/

VectorXf Manipulator::q;

MatrixXf Manipulator::jacobian(const VectorXf& q0,int upToJ, float eps) const { //TODO: add zOffset
    upToJ = upToJ == -1 ? nJoints : upToJ;
	/*JACOBIAN DIMENTION ASSIGNEMENT*/
	MatrixXf J(3,upToJ);
	/*COLUMN WISE ASSIGNEMENT OF JACOBIAN'S ELEMENTS*/
	for (unsigned int i{0}; i < static_cast<int> (q0.head(upToJ).size()); ++i) {
        VectorXf qEps_plus{q0}, qEps_minus{q0};
		qEps_plus[i] = q0[i] + eps;
        qEps_minus[i] = q0[i] - eps;
        J.col(i) = (dKin(qEps_plus,upToJ) - dKin(qEps_minus,upToJ))/(2*eps);
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

Vector4f Manipulator::dKinAlg(const VectorXf& vars, int upToNJoint, const int i, float xOffset) const {
	upToNJoint = upToNJoint == -1 ? nJoints : upToNJoint;
	Vector4f v{xOffset,0,0,1};
	Matrix4f H;


	/*NO CHECK ON DIMENTIONS*/
	/*
	if(vars.size() != nJoints) {
		std::cout << "Error\n";
		return v;
	}*/
	H = HTMat(vars[i],i);

    /*RECURSION*/
    if(i == upToNJoint - 1) {
    	return H*v;
    } else if(i < upToNJoint - 1 && i >= 0) {
    	return H*dKinAlg(vars, upToNJoint, i + 1);
    }
    else return v;
}

VectorXf Manipulator::update_configuration(const VectorXf& q_dot, const float T){
	q = q + q_dot * T;
	return q;
}

std::vector<Vector3f> Manipulator::controlPoints() const {
	/*TODO: modify for offset*/
	int nPoints = static_cast<int>(ctrPts.rows());
	std::vector<Vector3f> tmp(nPoints);
	for (int i = 0; i < nPoints; ++i) {
		tmp[i] = dKin(q,ctrPts(i,0),0,ctrPts(i,1));
	}
	return tmp;
}

void Manipulator::setCtrPts(const std::vector<int> joints, const std::vector<float> offset) {
    if(offset.isempty()) {
        for(int i{0}; i < l ; ++i) {
            offset.push_back(0);
        }
    }
    VectorXf _j(joints.data());
    VectorXf _o(offset.data());
    MatrixXf _ctrPts(l,2);
    _ctrPts.col(0) = _j;
    _ctrPts.col(1) = _o;
    ctrPts = _ctrPts;
}
