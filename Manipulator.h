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

Manipulator(MatrixXf& dh,VectorXf& q_in,float z_offset): nJoints{static_cast<int>(dh.rows())},DH{dh},world_z_offset{z_offset} {
	if(dh.cols() != 4) std::cout << "Invalid amount of columns\n";
	q = q_in;
	/*NO CHECK ON THE DIMENTION YET*/
}

/* | used only to resize the number of component of the |
 * | direct kinematics algorithm                        |
 * |                                                    |
 * v                                                    v
 */
Vector3f dKin(const VectorXf& vars = q,const int upToNJoint = 0,const int i = 0) const {

	MatrixXf S(3,4);
	S << 1, 0, 0, 0,
		 0, 1, 0, 0,
		 0, 0, 1, 0;
    MatrixXf W(4,4);
	W << 1, 0, 0, 0,
		 0, 1, 0, 0,
		 0, 0, 1, world_z_offset,
		 0, 0, 0, 1;
	return S*W*dKinAlg(vars,upToNJoint,i);
}

VectorXf get_state() const {
	return q;
}

// TASK JACOBIAN FOR POSITIONING TASKS IN THEE 3D SPACE -> TODO: UPGRADE FOR GENERAL TASK
MatrixXf jacobian(const VectorXf& q0 = q, float eps = 0.00001) const; /*DONE & WORKS*/

VectorXf update_configuration(const VectorXf& q_dot, const float T);

private:
// DIRECT KINEMATICS
Vector4f dKinAlg(const VectorXf& vars, int upToNJoin,const int i) const; /*DONE & WORKS*/
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
static VectorXf q;
float world_z_offset; 

// TODO: DYNAMICS ONES
};

#endif
