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
  R_laser_ << 0.0225, 0, 0, 0.0225;

  R_radar_ = MatrixXd(3, 3);
  R_radar_ << 0.09, 0, 0, 0, 0.0009, 0, 0, 0, 0.09;

  H_laser_ = MatrixXd(2, 4);
  H_laser_ << 1, 0, 0, 0, 0, 1, 0, 0;
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


    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      ekf_.x_ = VectorXd(4);
      ekf_.x_ << measurement_pack.raw_measurements_, 0, 0;
      ekf_.P_ = MatrixXd(4, 4);
      // Position uncertainty is given by measurement uncertainty
      auto p = R_laser_(0,0);
      // We don't know anything about velocity - assume that's uncertainty of 10.
      auto p_max = 10.;
      ekf_.P_ << p, 0, 0, 0, 0, p, 0, 0, 0, 0, p_max, 0, 0, 0, 0, p_max;
      previous_timestamp_ = measurement_pack.timestamp_;
      is_initialized_ = true;
    }

  }
  else {

    /*****************************************************************************
     *  Prediction
     ****************************************************************************/

    auto dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1e6;
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, dt, 0,
               0, 1, 0, dt,
               0, 0, 1, 0,
               0, 0, 0, 1;

    double noise_ax = 9;
    double noise_ay = 9;
    MatrixXd G(4,2);
    G << dt*dt/2, 0, 0, dt*dt/2, dt, 0, 0, dt;
    MatrixXd Qv(2,2);
    Qv << noise_ax, 0, 0, noise_ay;
    ekf_.Q_ = G * Qv * G.transpose();

    ekf_.Predict();
    previous_timestamp_ = measurement_pack.timestamp_;

    /*****************************************************************************
     *  Update
     ****************************************************************************/

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Radar updates
      ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
      ekf_.R_ = R_radar_;
      ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    } else {
      // Laser updates
      ekf_.H_ = H_laser_;
      ekf_.R_ = R_laser_;
      ekf_.Update(measurement_pack.raw_measurements_);
    }

  }

  // print the output
  //cout << "x_ = " << ekf_.x_.transpose() << endl;
  //cout << "P_ = " << endl << ekf_.P_ << endl;
}
