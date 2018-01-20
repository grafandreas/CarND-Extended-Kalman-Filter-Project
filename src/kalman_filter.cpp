#include "kalman_filter.h"
#include <assert.h>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {


  assert(P_in.rows() == F_in.rows());
  assert(P_in.rows() == Q_in.rows());
  assert(P_in.cols() == F_in.cols());
  assert(P_in.cols() == Q_in.cols());

  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  auto x_dash = F_* x_;
  x_ = x_dash;

  auto P_dash = F_* P_ * F_.transpose()+Q_;
  P_ = P_dash;
}

void KalmanFilter::Update(const VectorXd &z) {
	VectorXd y = z - H_ * x_;
	const MatrixXd S = H_ * P_ * H_.transpose() + R_;
	const MatrixXd K = P_ * H_.transpose() * S.inverse();

	MatrixXd I = MatrixXd::Identity(P_.rows(), P_.cols());

	auto x_dash = x_ + K * y;
	x_ = x_dash;
	auto P_dash = (I - K * H_) * P_;
	P_ = P_dash;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
}
