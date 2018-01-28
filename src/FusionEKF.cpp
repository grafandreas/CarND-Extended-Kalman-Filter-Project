#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  VectorXd x_(4);
  	x_ << 0.0,0.0,1.0,1.0;
  	MatrixXd F_ = MatrixXd::Identity(4,4);
  	F_(0,2) =0.5;
  	F_(1,3) = 0.5;

  	MatrixXd P_ = MatrixXd::Zero(4,4);
  	MatrixXd Q_ = MatrixXd::Zero(4,4);

  	MatrixXd dummy_= MatrixXd::Constant(4,4,0.0) ;

  	H_laser_ << 1,0,0,0,
  	     0,1,0,0;

  	// Check initialization
  	ekf_.Init(x_,P_,F_,dummy_,dummy_,Q_);


}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
     TODO:
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
     * Remember: you'll need to convert radar from polar to cartesian coordinates.
     */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      Tools t;
      VectorXd c = t.polar2Cart(measurement_pack.raw_measurements_);
      ekf_.x_ << c[0], c[1], 0.0, 0.0;
    } else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack
          .raw_measurements_[1], 0,  // Should 2 and 3 should be 0 anyway, but use constant so
      0;
    }

    // We need to init timestamp, too otherwise
    // first real measurement will be really distorted
    //
    previous_timestamp_ = measurement_pack.timestamp_;
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ <<  1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1000, 0,
                 0, 0, 0, 1000;


    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

//  if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
//    return;
//  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
   * Update the state transition matrix F according to the new elapsed time.
   - Time is measured in seconds.
   * Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */

  auto delta_t = ((measurement_pack.timestamp_ - previous_timestamp_)
      / 1000000.0);
  previous_timestamp_ = measurement_pack.timestamp_;
  std::cout << "delta_t" << delta_t << endl;

  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ <<  1, 0, delta_t, 0,
              0, 1, 0, delta_t,
              0, 0, 1, 0,
              0, 0, 0, 1;
   // Noise covariance matrix computation
   // Noise values from the task

  ekf_.Q_ = calcQ(delta_t);

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
   * Use the sensor type to perform the update step.
   * Update the state and covariance matrices.
   */
  cout << "Meas: " << measurement_pack.raw_measurements_ << endl;
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    cout << "RADAR" << endl;
	  ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
	  assert(!ekf_.H_.hasNaN());
	  	ekf_.R_ = R_radar_;
	  ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    cout << "LASER" << endl;
    ekf_.H_ = H_laser_;
    assert(!ekf_.H_.hasNaN());
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;




}

MatrixXd FusionEKF::calcQ(const double delta_t){

  const double dt2 = delta_t * delta_t;
  const double dt3 = delta_t * dt2;
  const double dt4 = delta_t * dt3;

  const double r11 = dt4 * noise_ax / 4;
  const double r13 = dt3 * noise_ax / 2;
  const double r22 = dt4 * noise_ay / 4;
  const double r24 = dt3 * noise_ay / 2;
  const double r31 = dt3 * noise_ax / 2;
  const double r33 = dt2 * noise_ax;
  const double r42 = dt3 * noise_ay / 2;
  const double r44 = dt2 * noise_ay;

  MatrixXd Q = MatrixXd::Zero(4,4);

  Q << r11, 0.0, r13, 0.0,
             0.0, r22, 0.0, r24,
             r31, 0.0, r33, 0.0,
             0.0, r42, 0.0, r44;

  return Q;
}

