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

    // =======================
    // initializing matrices
    // =======================
    R_laser_ = MatrixXd(2, 2);
    R_radar_ = MatrixXd(3, 3);
    H_laser_ = MatrixXd(2, 4);
    Hj_ = MatrixXd(3, 4);

    // =====================================
    // measurement covariance matrix - Laser
    // =====================================
    R_laser_ << 0.0225, 0,
                0, 0.0225;

    // =====================================
    // measurement covariance matrix - Radar
    // =====================================
    R_radar_ << 0.09, 0     , 0,
                0   , 0.0009, 0,
                0   , 0     , 0.09;

    /**
     * =============================================
       * Initializing the FusionEKF.
       * Setting the process and measurement noises
     * =============================================
    **/
    // ==========================
    // state covariance matrix P
    // ==========================
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0   , 0,
               0, 1, 0   , 0,
               0, 0, 1000, 0,
               0, 0, 0   , 1000;

    // ===================================
    // The initial transition matrix F_
    // ===================================
    ekf_.F_ = MatrixXd(4, 4);
    ekf_.F_ << 1, 0, 1, 0,
               0, 1, 0, 1,
               0, 0, 1, 0,
               0, 0, 0, 1;

    // ==========================
    // Measurement Matrix - Laser
    // ==========================
    H_laser_ = MatrixXd(2, 4);
    H_laser_ << 1, 0, 0, 0,
                0, 1, 0, 0;

    // ==========================
    // Measurement Matrix - Radar
    // ==========================
    Hj_ = MatrixXd::Zero(3, 4);
}

/**
* Destructor.
**/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack)
{
    float dt, dt_2, dt_3, dt_4;
    // =============================
    //       Initialization
    // =============================
    if (!is_initialized_)
    {
        /**
         * ================================================================================
            * Initializing the state ekf_.x_ with the first measurement.
            * Creating the covariance matrix.
            * Remember: you'll need to convert radar from polar to cartesian coordinates.
         * =================================================================================
         **/
        // first measurement
        cout << "EKF: " << endl;
        ekf_.x_ = VectorXd(4);
        ekf_.x_ << 1, 1, 1, 1;

        previous_timestamp_ = measurement_pack.timestamp_;
        if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
        {
            /**
             * ========================================================================
               Convert radar from polar to cartesian coordinates and initialize state.
             * ========================================================================
            **/
            float ro = measurement_pack.raw_measurements_(0);
            float phi = measurement_pack.raw_measurements_(1);
            float ro_dot = measurement_pack.raw_measurements_(2);
            ekf_.x_(0) = ro     * cos(phi);
            ekf_.x_(1) = ro     * sin(phi);
            ekf_.x_(2) = ro_dot * cos(phi);
            ekf_.x_(3) = ro_dot * sin(phi);
        }
        else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
        {
            /**
             * ======================
                Initialize state.
             * ======================
            **/
            ekf_.x_(0) = measurement_pack.raw_measurements_(0);
            ekf_.x_(1) = measurement_pack.raw_measurements_(1);
            ekf_.x_(2) = 0;
            ekf_.x_(3) = 0;
        }
        // ==================================================
        // done initializing, no need to predict or update
        // ==================================================
        is_initialized_ = true;
        return;
    }

    // ===============
    //   Prediction
    // ===============

    /**
     * =======================================================
       * Updating the state transition matrix F according to the new elapsed time.
          - Time is measured in seconds.
       * Updating the process noise covariance matrix.
       * Using noise_ax = 9 and noise_ay = 9 for your Q matrix.
      * =======================================================
     **/

    //set the acceleration noise components
    const float noise_ax = 9;
    const float noise_ay = 9;

    dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
    previous_timestamp_ = measurement_pack.timestamp_;

    dt_2 = dt * dt;
    dt_3 = dt_2 * dt;
    dt_4 = dt_3 * dt;

    //Modify the F matrix so that the time is integrated
    ekf_.F_(0, 2) = dt;
    ekf_.F_(1, 3) = dt;

    //set the process covariance matrix Q
    ekf_.Q_ = MatrixXd(4, 4);
    ekf_.Q_ <<  dt_4/4*noise_ax, 0              , dt_3/2*noise_ax, 0              ,
                0              , dt_4/4*noise_ay, 0              , dt_3/2*noise_ay,
                dt_3/2*noise_ax, 0              , dt_2*noise_ax  , 0              ,
                0              , dt_3/2*noise_ay, 0              , dt_2*noise_ay;

    ekf_.Predict();

    // ==============
    //    Update
    // ==============

    /**
     * ========================================================
       * Using the sensor type to perform the update step.
       * Updating the state and covariance matrices.
     * ========================================================
     **/

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
    {
        // ==============
        // Radar updates
        // ==============
        Tools tools;
        Hj_ = tools.CalculateJacobian(ekf_.x_);
        ekf_.H_ = Hj_;
        ekf_.R_ = R_radar_;
        ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    }
    else
    {
        // ==============
        // Laser updates
        // ==============
        ekf_.H_ = H_laser_;
        ekf_.R_ = R_laser_;
        ekf_.Update(measurement_pack.raw_measurements_);
    }

    // print the output
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
}
