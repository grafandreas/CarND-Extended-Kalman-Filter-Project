#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

#define FLOAT_FIX(x) ((fabs(x)) < 0.00001 ? 0.00001 : (x))

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  Eigen::IOFormat htmlFormat = Eigen::IOFormat(Eigen::FullPrecision,Eigen::DontAlignCols,"</td><td>","\n","<tr><td>","</td></tr>","<table border='1'>", "</table>");
  /**
  * A helper method to calculate RMSE.
  */
  VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

  /**
  * A helper method to calculate Jacobians.
  */
  MatrixXd CalculateJacobian(const VectorXd& x_state);

  MatrixXd polar2Cart(const VectorXd& polarC);

};

#endif /* TOOLS_H_ */
