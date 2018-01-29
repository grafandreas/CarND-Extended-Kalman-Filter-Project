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

	assert(rmse(0) < 0.5);
	assert(rmse(1) < 0.5);
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);
  MatrixXd Hj(3, 4);
  // Fix numeric problems
  px = FLOAT_FIX(px);
  py = FLOAT_FIX(py);

  // Thi stuff is used repeatedly
  float c1 = px * px + py * py;
  // Fix numeric problems

  c1 = FLOAT_FIX(c1);
  float c2 = sqrt(c1);
//  c2 = FLOAT_FIX(c2);
  float c3 = (c1 * c2);
//  c3 = FLOAT_FIX(c3);

  // Jacobian matrix
  Hj << (px/c2), (py/c2), 0, 0,
       -(py/c1), (px/c1), 0, 0,
        py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 3; j++)
      if (isnan(Hj.coeff(i, j))) {
//        cout << "Nan " << i << "," << j << endl;
//        cout << c1 << "," << c2 << "," << c3 << endl;
      }

  assert(!Hj.hasNaN());

  return Hj;
}

MatrixXd Tools::polar2Cart(const VectorXd& polarC) {
	auto res = VectorXd(4);
	res << polarC(0) * cos(polarC(1)), polarC(0) * sin(polarC(1)),
			polarC(2) * cos(polarC(1)), polarC(2) * sin(polarC(1));
	return res;
}
