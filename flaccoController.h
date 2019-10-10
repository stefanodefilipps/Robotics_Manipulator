#ifndef FLACCOCONTROLLER
#define FLACCOCONTROLLER

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/QR>
#include <math.h>
#include <vector>

using namespace Eigen;
using namespace std;

class FlaccoController{
public:

// OBSTACLES' POSITION KNOWN BY ANY INSTANCES OF MANIPULATOR
static std::vector<Vector3f> obstPos;

// TO ADD AN OBSTACLE TO THE STACK
static void newObst(const Vector3f newPos);

	// to the controller constructor we need to pass the terms for computing the repulsive vector magnitude:
	// aplha_ : to shape the sigmoid curve of the magnitude
	// rho_: the half length of the control point bounding box
	// v_max_: The maximum velocity in module that the repulsive vector can get
	// ks is a vector containing the diagonal element of the proportional parameters of the cartesian control scheme
	FlaccoController(float alpha_, float rho_, float v_max_, VectorXf ks){
		alpha = alpha_;
		rho = rho_;
		v_max = v_max_;
		K = ks.asDiagonal();
	}

	/*
		The following is the function that implements the controller and returns the joint velocities to command
		We pass the parameters following certain criterias:
		- Ji is the vector containng the stack of Jacobian matrices. The first task is always the end effector position task and then the control points 
		  in descending order (in our case the CP corresponding to joint 4 and then joint 1). Finally we add the other auxialiary tasks
		- bi is the stack of the tasks velocities and their ordering is the same as the stack of Jacobian
		- obstacles is a vector containg the world position of the obstacles in the environment
		- CPs id a vector containing the world position of the control points
		- p_ds is a vector containing the feedforward trajectory values for implementing a simple cartesian control scheme and thei ordering is the same as Ji
	*/

	VectorXf control(vector<MatrixXf> Ji, vector<VectorXf> bi, vector<VectorXf> obstacles, vector<VectorXf> CPs, vector<VectorXf> p_ds, float lam = 0.1, float eps = 0.1);
	MatrixXf projectJ(const MatrixXf& J, const Vector3f& pos, const int nObst = 0);
    float projectP(const Vector3f& vel,const Vector3f& pos, const int nObst = 0);
    Vector3f eeDisVec(const VectorXf &Pos, const int numberOfObstacle = 0) const;
    // I moved the function to compute the repulsive velocities in the controller instead that in the manipulator since they are only use in the controller scheme
    float eeDis(const VectorXf &Pos, const int numberOfObstacle = 0) const;
private:

static bool isObstacle; // --> to verify wether there is an obstacle or not


	MatrixXf damped_pinv(MatrixXf J,float lam, float eps);

	MatrixXf tasksPriorityMatrix(MatrixXf bF,VectorXi tasksDim,float lam,float eps);

	VectorXf FlaccoPrioritySolution(vector<MatrixXf> Ji, vector<VectorXf> bi, float lam = 0.1, float eps = 0.1);

	float repulsiveMagnitude(const VectorXf &Pos, const int numberOfObstacle = 0) const;

	Vector3f eeRepulsiveVelocity(const VectorXf &Pos, const int numberOfObstacle = 0) const;

	float alpha;
	float rho;
	float v_max;
	MatrixXf K;
};

#endif
