#ifndef FLACCO_IMPL
#define FLACCO_IMPL

#include <iostream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/QR>
#include <math.h>
#include <vector>

using namespace Eigen;


MatrixXf damped_pinv(MatrixXf J,float lam, float eps);

MatrixXf tasksPriorityMatrix(MatrixXf bF,VectorXi tasksDim,float lam,float eps);

MatrixXf FlaccoPrioritySolution(std::vector<MatrixXf> Ji, std::vector<MatrixXf> bi, float lam, float eps);

#endif