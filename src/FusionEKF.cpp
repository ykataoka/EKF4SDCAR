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
  H_laser_ = MatrixXd(2, 4);
  
  R_radar_ = MatrixXd(3, 3);
  Hj_ = MatrixXd(3, 4);  // Jacobian for radar
  

  /**
    * Initializing
    * Set the process and measurement noises
  */
  
  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;
  // H_laser_ 
  H_laser_ << 1, 0, 0, 0,
    0, 1, 0, 0;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  // Hj_ (Jacobian Matrix) for Radar
  ekf_.x_ = VectorXd(4);
  float px = ekf_.x_(0);
  float py = ekf_.x_(1);
  float vx = ekf_.x_(2);
  float vy = ekf_.x_(3);  
  float c1 = px*px+py*py;
  if(c1 < 0.0001){
    c1 = 0.0001;
  }
  float c2 = sqrt(c1);
  float c3 = c1*c2;

  Hj_ << (px/c2), (py/c2), 0, 0,
    -(py/c1), (px/c1), 0, 0,
    py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

}

/**
* Destructor
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
     * Initialize the state ekf_.x_ with the first measurement.
     * Create the covariance matrix.
    */

    // first measurement
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    // RADAR
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      ekf_.x_(0) = measurement_pack.raw_measurements_(0);
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);
    }

    // LASER
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {      
      /**
      Initialize state.
      * need to convert radar from polar to cartesian coordinates.
      * px = rho * sin(phi)
      * py = rho * cos(phi)
      * vx = rho_dot sin(phi) + rho cos(phi) phi_dot
      * vy = rho_dot cos(phi) - rho sin(phi) phi_dot
      * but, phi_dot cannot be observed, so just skip vx and vy for now.
      */
      float rho = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      ekf_.x_(0) = rho * sin(phi);
      ekf_.x_(1) = rho * cos(phi);
    }

    // state covariance matrix
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1000, 0,
      0, 0, 0, 1000;

    // state transition matrix (updated later depending on dt)
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1, 0,
      0, 0, 0, 1;
    
    // done initializing, no need to predict or update
    previous_timestamp_ = measurement_pack.timestamp_;
    is_initialized_ = true;

    return;
  }

  /*****************************************************************************
   *  Prediction (State Estimation using Vehicle Dynamics)
   ****************************************************************************/

  // update the state transition matrix F according to the new elapsed time.
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  previous_timestamp_ = measurement_pack.timestamp_;
  
  // update the process noise covariance matrix (acceleration variation)
  float noise_ax = 9;
  float noise_ay = 9;
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
    0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
    dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
    0, dt_3/2*noise_ay, 0, dt_2*noise_ay;
  
  ekf_.Predict();
  
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  // CASE : RADAR (Extended kalman Filter)
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {

    // Radar updates
    ekf_.R_ = MatrixXd(3, 3);
    ekf_.H_ = MatrixXd(3, 4);

    // Update Jacobian
    float px = ekf_.x_(0);
    float py = ekf_.x_(1);
    float vx = ekf_.x_(2);
    float vy = ekf_.x_(3);
    float c1 = px*px+py*py;
    if(c1 < 0.1){
      c1 = 0.1;
    }
    float c2 = sqrt(c1);
    float c3 = c1*c2;
  
    // Hj_ (Jacobian Matrix) for Radar (Linearization around the current state)
    Hj_ << (px/c2), (py/c2), 0, 0,
      -(py/c1), (px/c1), 0, 0,
      py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

    // Replace the Variables
    ekf_.R_ = R_radar_;
    ekf_.H_ = Hj_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    
  // CASE : LASER (Standsard kalman Filter)
  }else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
    // Laser updates (Starndard : ekf_.Update())
    ekf_.R_ = MatrixXd(2, 2);
    ekf_.H_ = MatrixXd(2, 4);
    
    // Replace the Variables
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
