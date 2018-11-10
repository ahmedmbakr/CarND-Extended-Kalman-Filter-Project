#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new estimate
	VectorXd new_x = x_ + (K * y);
	if (isReasonableNewX(new_x)) {
		x_ = new_x;
		long x_size = x_.size();
		MatrixXd I = MatrixXd::Identity(x_size, x_size);
		P_ = (I - K * H_) * P_;
	}
}


bool KalmanFilter::isReasonableNewX(const VectorXd & newX)
{
	static int number_of_readings = 0;
	//ignore the first 4 calls to this function
	if(number_of_readings < 4)
	{
		number_of_readings++;
		return true;
	}
	const double new_x_val = newX(0);
	const double new_y_val = newX(1);
	const double new_xvel_val = newX(2);
	const double new_yvel_val = newX(3);

	const double curr_x_val = x_(0);
	const double curr_y_val = x_(1);
	const double curr_xvel_val = x_(2);
	const double curr_yvel_val = x_(3);

	if (abs(new_x_val - curr_x_val) > 0.3)return false;
	if (abs(new_y_val - curr_y_val) > 0.3)return false;
	/*if (abs(new_xvel_val - curr_xvel_val) > 0.6)return false;
	if (abs(new_yvel_val - curr_yvel_val) > 0.6)return false;*/

	return true;
}

// This function is used to update Radar measurements.
void KalmanFilter::UpdateEKF(const VectorXd &z) { 
	float x_pred = this->x_(0);
	float y_pred = this->x_(1);
	float vx = this->x_(2);
	float vy = this->x_(3);

	float rho = sqrt(x_pred * x_pred + y_pred * y_pred);
	const float epslon = 1e-2;
	float theta = atan2(y_pred, x_pred);
	float ro_dot = (x_pred * vx + y_pred * vy) / rho;
	
	VectorXd z_pred = VectorXd(3);
	z_pred << rho, theta, ro_dot;
	VectorXd y = z - z_pred;

	MatrixXd Ht = this->H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	//new state
	VectorXd new_x = x_ + (K * y);
	if (isReasonableNewX(new_x)) {
		x_ = new_x;
		long x_size = x_.size();
		MatrixXd I = MatrixXd::Identity(x_size, x_size);
		P_ = (I - K * H_) * P_;
	}
}
