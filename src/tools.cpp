#include <iostream>
#include "tools.h"
#include <math.h>
#include "Eigen/Dense"
#include <assert.h>
using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {

	assert(estimations.size() == ground_truth.size());

	VectorXd rmse = VectorXd(4);
	rmse << 0.0,0.0,0.0,0.0;

	for(int i = 0; i < estimations.size(); i++) {
		VectorXd delta = estimations[i] - ground_truth[i];
		VectorXd sq = delta.array() * delta.array();

		rmse+=sq;
	}
	rmse = rmse / estimations.size();
	rmse = rmse.array().sqrt();
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
}

MatrixXd Tools::polar2Cart(const VectorXd& polarC) {
	auto res = VectorXd(4);
	res << polarC(0) * cos(polarC(1)), polarC(0) * sin(polarC(1)),
			polarC(2) * cos(polarC(1)), polarC(2) * sin(polarC(1));
	return res;
}
