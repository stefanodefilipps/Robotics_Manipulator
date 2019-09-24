/*
This file was made by:
Marco Menchetti.

Write me @ menchetti.1713013@studenti.uniroma1.it

*/

#ifndef MANIPULATOR
#define MANIPULATOR

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>

using namespace Eigen;

class Manipulator {

public:

Manipulator(MatrixXf& dh): nJoints{static_cast<int>(dh.rows())},DH{dh} {
	if(dh.cols() != 4) std::cout << "Invalid amount of columns\n";
	/*NO CHECK ON THE DIMENTION YET*/
}

/* | used only to resize the number of component of the |
 * | direct kinematics algorithm                        |
 * |                                                    |
 * v                                                    v
 */
Vector3f dKin(const std::vector<float>& vars,const int upToNJoint = 0,const int i = 0) const {

	MatrixXf S(3,4);
	S << 1, 0, 0, 0,
		 0, 1, 0, 0,
		 0, 0, 1, 0;
	return S*dKinAlg(vars,upToNJoint,i);
}

MatrixXf GeomJacAngular(const std::vector<float>& vars) const{
	return GeomJacAngularPrivate(vars);
};

// TASK JACOBIAN FOR POSITIONING TASKS IN THEE 3D SPACE -> TODO: UPGRADE FOR GENERAL TASK
MatrixXf jacobian(const std::vector<float>& q0, float eps = 0.00001) const; /*DONE & WORKS*/

/*MatrixXf GeomJacAngular(MatrixXf Rs, const std::vector<float>& vars,const int i = 1) const;*/

private:
// DIRECT KINEMATICS
Vector4f dKinAlg(const std::vector<float>& vars, int upToNJoin,const int i) const; /*DONE & WORKS*/
Matrix4f HTMat(const float var, const int i) const;
Matrix3f RMat(const float var, const int i) const;

// CHARACTERISTICS
const int nJoints{0};

// KINEMATICS ONES

/* DH TABLE SETTINGS:
 * - FIRST COLUMN: TWIST ANGLE
 * - SECOND COLUMN: COMMON NORMAL LENGTH
 * - THIRD COLUMN: "NON-JOINT" CONSTANT
 *		a) IF REVOLUTE  -> DISPLACEMENT ALONG THE JOINT AXIS
 *		b) IF PRISMATIC -> ANGLE BETWEEN LINK AXES
 * - FOURTH COLUMN IS THE TYPE OF JOINT:
 *		a) IF REVOLUTE  -> "1"
 *		b) IF PRISMATIC -> "0" 
 */
MatrixXf DH; 

// TODO: DYNAMICS ONES
};

#endif
