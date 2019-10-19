#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/QR>
#include <math.h>
#include <vector>
#include "flaccoController.h"

using namespace std;
using namespace Eigen;

VectorXf FlaccoController::control(vector<MatrixXf> Ji, vector<VectorXf> bi, vector<VectorXf> CPs, float lam, float eps){
	
	// In this function I implement a simple kinematic control of Cartisian motion con feedback for the end effector
	// I add the repulsive velocity to the ee task velocity and in this case I have only one obstacle
	// In the future here we need to implement the riordering of the task based on the distance of the control points from the obstacles and
	// when we have multiple obstacles we also need to take care of that case
	return FlaccoPrioritySolution(Ji, bi, lam, eps);
}
/*
 * To project the jacobian and the task velocity along the distance from the obstacle
 */
MatrixXf FlaccoController::projectJ(const MatrixXf& J, const Vector3f& pos, const int nObst) {
    if(J.rows() != 3) {
        std::cout << "Error in J dimentions: number of rows should be 3.\n Returning J" << std::endl;
        return J;
    }
    Vector3f n{-eeDisVec(pos,nObst)/eeDis(pos,nObst)};
    return n.transpose()*J;
}
float FlaccoController::projectP(const Vector3f& pos, const int nObst) {
    return projectJ(eeRepulsiveVelocity(pos,nObst),pos,nObst)(0);
    /*
     * Return the first (and only, due to the checks) element
     * of the Matrix returned by projectJ
     */
}

/* This is the function to compute the damped pseudoinverse
   - lam is the largest damping factor used near singularities
   - eps > 0 monitors the smallest singular value defining the range of the damping action*/

/* STATIC */
bool FlaccoController::isObstacle{0};
std::vector<Vector3f> FlaccoController::obstPos;
void FlaccoController::newObst(const Vector3f newPos) {
	if(obstPos.size() == 1) obstPos[0] = newPos;
	else obstPos.push_back(newPos);
}

MatrixXf FlaccoController::damped_pinv(MatrixXf J,float lam, float eps){
	JacobiSVD<MatrixXf> svd(J, ComputeFullU | ComputeFullV);
	int rank = svd.rank();
	VectorXf sv = svd.singularValues();
	float min_sv = sv(sv.size() - 1);
	float lam_square = 0;
	/* Now I have to choose the correct lambda square
	   if less than epsilon then I affirm that I am close to a singularity and I compute the variable damping factor
	   lam_square to use when computing the pseudoinverse*/
	if(min_sv < eps){
		lam_square = (1 - pow(min_sv / eps,2.0)) * pow(lam,2.0);
	}

	MatrixXf U = svd.matrixU();
	MatrixXf V = svd.matrixV();
	MatrixXf U_ = U.transpose();
	float sv_i = sv(0);
	/* Here I am computing the pseudoinverse given classic formula through the use of the matrices from the singular value decomposition
	   OBS: if lam is set to 0 then I have the regular pseudoinverse*/
	MatrixXf Y = (sv_i / (pow(sv_i,2.0) + pow(lam,2.0))) * V.col(0) * U_.row(0);
	for (int i = 1; i < rank; ++i){
		sv_i = sv(i);
		Y = Y + (sv_i / (pow(sv_i,2.0) + pow(lam,2.0))) * V.col(i) * U_.row(i);
	}

	return Y;
}

/* This is the function that computes the priority matrix as defined in Flacco's paper
   - bF is the R initial matrix from the QR decomposition of the augmented task matrix and has dimension mxm where m is the sum of all tasks' dimensions
   - tasksDim is a vector containing the tasks' sizes
   - lam is the largest damping factor used near singularities
   - eps > 0 monitors the smallest singular value defining the range of the damping action*/

MatrixXf FlaccoController::tasksPriorityMatrix(MatrixXf bF,VectorXi tasksDim,float lam,float eps){

	int m = bF.rows();
	int i = 0;
	int i_taskDim = 0;
	int j = 0;
	int rows = 0;
	int col = 0;
	int last = m - i;
	/* In i I keep the start row and column of the task that I am currently working on*/
	/* Now for each pivot square block in bF I execute the Gauss Jordan elimination which requires 2 steps*/
	while(i < m){
		/* In j I keep the index of the final row and column of the current task I am working on  */
		j = i + tasksDim(i_taskDim) - 1;
		rows = tasksDim(i_taskDim);
		col = tasksDim(i_taskDim);
		/* I compute the pseudoinverse of the pivot block matrix of the corresponding to the current task I am working on*/
		MatrixXf pR = damped_pinv(bF.block(i,i,rows,col),lam,eps);
		last = m-i;
		/* I execute the first step of the Gauss Jordan elimination in order to try to have an identity matrix as a pivot block*/
		bF.block(i,i,rows,last) = pR * bF.block(i,i,rows,last);
		/* I execute the second step pf the Gauss Jordan elimination in order to try to nullify the block corrisponding to same block column as the current task
		   but preceding block rows*/
		bF.block(0,i,i,last) = bF.block(0,i,i,last) - bF.block(0,i,i,col) * bF.block(i,i,rows,last);
		i = j + 1;
		i_taskDim = i_taskDim + 1;
	}
	MatrixXf F = bF.transpose();
	return F;
}

/* This is the function that actually computest the solution through the use of the task priority matrix
   - Ji is a vector containing the tasks' jacobian matrices
   - bi is a vector containing the tasks' actual velocities
   - lam is the largest damping factor used near singularities
   - eps > 0 monitors the smallest singular value defining the range of the damping action*/


VectorXf FlaccoController::FlaccoPrioritySolution(vector<MatrixXf> Ji, vector<VectorXf> bi, float lam, float eps){
	int dim_J = 0;
	int dim_b = 0;
	for(int i = 0; i < Ji.size(); ++i){
		dim_J = dim_J + Ji[i].rows();
		dim_b = dim_b + bi[i].size();
	}
	/* I first need to create the matrices for the augmented task so I stack all the Ji and bi matrices in order to get the augmented task matrix*/
	MatrixXf J(dim_J,Ji[0].cols());
	MatrixXf b(dim_b,1);
	VectorXi tasksDim(bi.size());
	int start = 0;
	for(int i = 0; i < Ji.size(); ++i){
		J.block(start,0,Ji[i].rows(),Ji[i].cols()) = Ji[i];
		b.block(start,0,bi[i].size(),1) = bi[i];
		tasksDim(i) = bi[i].rows();
		start = start + Ji[i].rows();
	}
	/* Now I need to compute the QR decomposition of J transpose in order to retrieve R the inintial matrix needed to compute the task priority matrix*/
	HouseholderQR<MatrixXf> qr(J.transpose());
	MatrixXf R = qr.matrixQR().triangularView<Upper>();
	/* R is a nxm matrix with n the number of robot's degree of freedom and m the sum of all tasks' dimensions. The last n-m rows of R are all 0s, therefore I
	   can extract a matrix R_ of dimension mxm and this is the one that I will actually use to compute the task priority matrix*/
	MatrixXf R_ = R.block(0,0,J.rows(),J.rows());
	MatrixXf Q = qr.householderQ();
	/* F is the task priority matrix*/
	MatrixXf F = tasksPriorityMatrix(R_,tasksDim,lam,eps);

	CompleteOrthogonalDecomposition<MatrixXf> cqr(R.transpose());
	MatrixXf pinv  = cqr.pseudoInverse(); 
	/* The final solution is given by the pseudoinverse of the augmented task matrix multiplyed by the priority task matrix time the task velocity augmented vector.
	   Since I already have Q and R I computed the J pseudiinverse simply by doing Q * R^(-T) and the inversion is computed through a pseudoinverse.
	   OBS: when I compute the pseudoinverse of R i need to use the nxm matrix and NOT the reduced one.*/
	VectorXf x = Q * pinv * F * b;
	return x;
}

Vector3f FlaccoController::eeDisVec(const VectorXf &Pos, const int numberOfObstacle) const {
	return Pos - obstPos[numberOfObstacle];
}

float FlaccoController::eeDis(const VectorXf &Pos, const int numberOfObstacle) const {
	Vector3f d{eeDisVec(Pos,numberOfObstacle)};
	return sqrt(d.transpose()*d);
}

float FlaccoController::repulsiveMagnitude(const VectorXf &Pos, const int numberOfObstacle) const{
	return (v_max / (1 + exp((eeDis(Pos,numberOfObstacle) * (2 / rho) -1) * alpha)));
}

Vector3f FlaccoController::eeRepulsiveVelocity(const VectorXf &Pos, const int numberOfObstacle) const{
	return repulsiveMagnitude(Pos,numberOfObstacle) * eeDisVec(Pos,numberOfObstacle) / eeDis(Pos,numberOfObstacle);
}

void FlaccoController::taskReorder(Task& stack,const std::vector<Vector3f>& contPoints, float d,float critic_d) const {
	int cycle = stack.size();
	/*DISTANCE VECTOR*/
	std::vector<float> dist(cycle,0);
	for (int i = 0; i < cycle; ++i) {
		int stackInd = stack.getInd()[i];
		if(stackInd == 1) dist[stackInd] = d+1;  /* index 1 means a peculiar task
												  * for which we add a fictitious distance
												  */
		else {
			float temp_d = eeDis(contPoints[stackInd == 0 ? stackInd : stackInd - 1]); // ctrP has 1 element less than stack
			dist[stackInd] = temp_d;
		}
	}
	/*SWAPPING*/
	int criticity{0};
	for (int i = 0; i < cycle - 1; ++i) {
		float di = dist[i];
		float dj = dist[i+1];
		if(dj < di) {
			if(dj <= d && dj > critic_d && i > criticity){
				stack.swapTask(i,i+1); dist[i] = dj; dist[i+1] = di;
				i -= 2;
			} else if(dj <= critic_d) {
				stack.swapTask(i,i+1); dist[i] = dj; dist[i+1] = di; 	//swap task and distances so that in next step
																	 	// won't be computed again

				if(i>criticity) ++criticity;	// dimension of the critic distance vector increase if a critic
												// swap is being done from outside the sub-vector

				i == 0 ? i-=1 : i -= 2; //can't accede to dist[-1] in the next step if i == 0
			}
		}
	}
}

