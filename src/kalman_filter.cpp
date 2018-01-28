#include "kalman_filter.h"
#include "tools.h"
#include <assert.h>
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;
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
  Tools tool;
  cout << "<h3>Predict</h3>" << endl;
  VectorXd x_dash = F_* x_;
  MatrixXd P_dash = F_* P_ * (F_.transpose())+Q_;

#ifdef DEBUG
  cout << "<table><tr><td>P_</td><td>P'</td><td>x_</td><td>x'</td><td>Q_</td><td>F_</td></tr>" <<endl;
       cout << "<tr><td>" << P_.format(tool.htmlFormat) << "</td>" << endl;
       cout << "<td>" << P_dash.format(tool.htmlFormat) << "</td>" << endl;
       cout << "<td>" << x_.format(tool.htmlFormat) << "</td>" << endl;
       cout << "<td>" << x_dash.format(tool.htmlFormat) << "</td>" << endl;
       cout << "<td>" << Q_.format(tool.htmlFormat) << "</td>" << endl;
       cout << "<td>" << F_.format(tool.htmlFormat) << "</td>" << endl;
       cout << "</tr></table>" << endl;

  cout << "<table><tr><td>F_</td><td>F_* P_ </td><td>F_* P_ * (F_.transpose())</td><td>F_* P_ * (F_.transpose())+Q_</td></tr>" <<endl;
  cout << "<tr><td>" << F_.format(tool.htmlFormat) << "</td>" << endl;
  cout << "<td>" << (F_*P_).format(tool.htmlFormat) << "</td>" << endl;
  cout << "<td>" << (F_*P_* (F_.transpose())).format(tool.htmlFormat) << "</td>" << endl;
  cout << "<td>" << (F_* P_ * (F_.transpose())+Q_).format(tool.htmlFormat) << "</td>" << endl;
  cout << "</tr></table>" << endl;
#endif
  x_ = x_dash;
  P_ = P_dash;





}

void KalmanFilter::Update(const VectorXd &z) {

  cout << "<h3>Update</h3>" << endl;

	VectorXd y = z - H_ * x_;
	updateCommon(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  double rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  double theta = atan2(x_(1) , x_(0));
  double rho_dot = (x_(0)*x_(2) + x_(1)*x_(3)) / rho;
  VectorXd h = VectorXd(3);
  h << rho, theta, rho_dot;

  VectorXd y = z - h;


  for(; y(1) < -M_PI; y(1) += 2*M_PI);
  for(; y(1) > M_PI;  y(1) -= 2*M_PI);

  cout << "y(1)" <<y(1) << endl;
  assert(y(1) >= ( -1 * M_PI));
  assert(y(1) <=  M_PI);
  updateCommon(y);
}

void KalmanFilter::updateCommon(const VectorXd &y) {
  assert(!P_.hasNaN());
  assert(!H_.hasNaN());
  const MatrixXd P_times_H_transpo = P_ * H_.transpose();
  assert(!P_times_H_transpo.hasNaN());
  const MatrixXd S = H_ * P_times_H_transpo + R_;
  assert(!S.hasNaN());
  const MatrixXd K = P_times_H_transpo * S.inverse();
  assert(!K.hasNaN());
  MatrixXd I = MatrixXd::Identity(P_.rows(), P_.cols());
  assert(!I.hasNaN());
  VectorXd x_dash = x_ + K * y;
  cout << "DBG" << endl;
//		cout << "S " << S << endl;
//		cout << "Si " << S.inverse() << endl;
  Tools tool;
#ifdef DEBUG
  cout << "<table><tr><td>P_</td><td>H_</td><td>R_</td><td>K_</td><td>x_</td><td>x_dash</td><td>y</td><td>PH_t</td><td>Si</td><td>S</td></tr>" <<endl;
  cout << "<tr><td>" << P_.format(tool.htmlFormat) << "</td>" << endl;
  cout << "<td>" << H_.format(tool.htmlFormat) << "</td>" << endl;
  cout << "<td>" << R_.format(tool.htmlFormat) << "</td>" << endl;
  cout << "<td>" << K.format(tool.htmlFormat) << "</td>" << endl;
  cout << "<td>" << x_.format(tool.htmlFormat) << "</td>" << endl;
  cout << "<td>" << x_dash.format(tool.htmlFormat) << "</td>" << endl;
  cout << "<td>" << y.format(tool.htmlFormat) << "</td>" << endl;
  cout << "<td>" << P_times_H_transpo.format(tool.htmlFormat) << "</td>" << endl;
  cout << "<td>" << S.inverse().format(tool.htmlFormat) << "</td>" << endl;
  cout << "<td>" << S.format(tool.htmlFormat) << "</td>" << endl;
  cout << "</tr></table>" << endl;


  cout << "H_" << H_ << endl;
  cout << "R_" << R_ << endl;
  cout << "x_" << x_ << endl;
  cout << "K" << K << endl;
  cout << "y" << y << endl;
#endif

  assert(!x_dash.hasNaN());
  x_ = x_dash;
  MatrixXd P_dash = (I - K * H_) * P_;
  P_ = P_dash;
}
