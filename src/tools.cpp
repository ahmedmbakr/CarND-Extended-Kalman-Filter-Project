#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

//RMSE = sqrt( (1 / n) * sum-of((estimiations[i] - ground_truth[i])^2) )
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) 
{
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if (estimations.size() != ground_truth.size()
		|| estimations.size() == 0) 
	{
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for (unsigned int i = 0; i < estimations.size(); ++i) {

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array() * residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse / estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

//The Jacobian matrix is used in the computation of the radar update stage
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	MatrixXd Hj(3, 4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//check division by zero
	if (px == 0 || py == 0) {
		cout << "px or py can not be zero\n";
		return Hj;
	}

	//compute the Jacobian matrix
	float px2PlusPy2 = px * px + py * py;
	float px2PlusPy2_pow_1Point5 = pow(px2PlusPy2, 1.5);
	float px2PlusPy2_sqrt = sqrt(px2PlusPy2);
	float hj_0_0 = px / px2PlusPy2_sqrt;
	float hj_0_1 = py / px2PlusPy2_sqrt;
	float hj_1_0 = -py / px2PlusPy2;
	float hj_1_1 = px / px2PlusPy2;
	float hj_2_0 = py * (vx * py - vy * px) / px2PlusPy2_pow_1Point5;
	float hj_2_1 = px * (vy * px - vx * py) / px2PlusPy2_pow_1Point5;
	float hj_2_2 = hj_0_0;
	float hj_2_3 = hj_0_1;

	Hj << hj_0_0, hj_0_1, 0, 0,
		hj_1_0, hj_1_1, 0, 0,
		hj_2_0, hj_2_1, hj_2_2, hj_2_3;
	return Hj;
}
