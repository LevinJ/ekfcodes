/*
 * fusion_ekf.cpp
 *
 *  Created on: Nov 3, 2016
 *      Author: MBRDNA
 */

#include "fusion_ekf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor
 */
FusionEKF::FusionEKF() {
	is_initialized_ = false;
	previous_timestamp_ = 0;

	R_laser_ = MatrixXd(2, 2);
	R_radar_ = MatrixXd(3, 3);
	H_laser_ = MatrixXd(2, 4);
	Hj_ = MatrixXd(3, 4);

	// measurement covariance matrix - laser
		R_laser_ << 0.0225, 0,
								0, 0.0225;

		// measurement covariance matrix - radar
		R_radar_ << 0.09, 0, 0,
		        0, 0.0009, 0,
				0, 0, 0.09;

	H_laser_ << 1, 0, 0, 0,
							0, 1, 0, 0;

	ekf_.F_ = MatrixXd(4, 4);

	// for state state transition matrix F, delta_t is initialized with 1 (1 sec)
	// it is going to be updated later in the ProcessMeasurement based on
	// measurement timestamps
	ekf_.F_ << 1, 0, 1, 0,
						 0, 1, 0, 1,
						 0, 0, 1, 0,
						 0, 0, 0, 1;
}

FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {

	/*****************************************************************************
	 *  Prediction
	 ****************************************************************************/
	if (!is_initialized_) {
		// first measurement
		cout << "EKF: " << endl;
		ekf_.x_ = VectorXd(4);
		ekf_.x_ << 0, 0, 0, 0;

		if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
		{
			ekf_.x_(0) = measurement_pack.raw_measurements_(0) * cos(measurement_pack.raw_measurements_(1));
			ekf_.x_(1) = measurement_pack.raw_measurements_(0) * sin(measurement_pack.raw_measurements_(1));
		}
		else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER)
		{
			ekf_.x_(0) = measurement_pack.raw_measurements_(0);
			ekf_.x_(1) = measurement_pack.raw_measurements_(1);
		}

		ekf_.P_ = MatrixXd(4, 4);
		ekf_.P_ << 1, 0, 0, 0,
							 0, 1, 0, 0,
							 0, 0, 1000, 0,
							 0, 0, 0, 1000;

		previous_timestamp_ = measurement_pack.timestamp_;
		is_initialized_ = true;

		// done initializing, no need to predict or update
		return;
	}

	/*****************************************************************************
	 *  Prediction
	 ****************************************************************************/
	// compute the time elapsed between the current and previous measurements
	// dt - expressed in seconds
	float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
	float dt_2 = dt * dt;
	float dt_3 = dt_2 * dt;
	float dt_4 = dt_3 * dt;
	previous_timestamp_ = measurement_pack.timestamp_;

	//Modify the F matrix so that the time is integrated
	ekf_.F_(0, 2) = dt;
	ekf_.F_(1, 3) = dt;

	//set the process noise covariance matrix
	float sigma_ax = 9.;
	float sigma_ay = 9.;
	ekf_.Q_ = MatrixXd(4, 4);
	ekf_.Q_ << dt_4 / 4 * sigma_ax, 0, dt_3 / 2 * sigma_ax, 0,
						 0, dt_4 / 4 * sigma_ay, 0, dt_3 / 2 * sigma_ay,
						 dt_3 / 2 * sigma_ax, 0, dt_2 * sigma_ax, 0,
						 0, dt_3 / 2 * sigma_ay, 0, dt_2 * sigma_ay;

	ekf_.Predict();

	/*****************************************************************************
	 *  Update
	 ****************************************************************************/
	/* TODO:
	 * 1. Define H, R for each sensor
	 * 2. Change H, R, based on the sensor type
	 * 3. Apply KF
	 * 4. Define timestamps
	 */

	if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR)
	{
		// recover state parameters
		float x = ekf_.x_(0);
		float y = ekf_.x_(1);
		float vx = ekf_.x_(2);
		float vy = ekf_.x_(3);

		if (x == 0 && y == 0)
		{
			// avoid NaN values, just predict the current state
			return;
		}

		MatrixXd Hj = tools.CalculateJacobian(ekf_.x_);

		ekf_.H_ = Hj;
		ekf_.R_ = R_radar_;

		// update manually with nonlinear measurement function
		// radar uses polar coordinates
		float ro = sqrt(x * x + y * y);
		float theta = atan2(y, x);
		float ro_dot = (x * vx + y * vy) / ro;
		VectorXd z_pred(3);
		z_pred << ro, theta, ro_dot;

		ekf_.UpdateWithAlreadyPredictedMeasurements(measurement_pack.raw_measurements_,z_pred);
	}
	else
	{
		ekf_.H_ = H_laser_;
		ekf_.R_ = R_laser_;
		ekf_.Update(measurement_pack.raw_measurements_);
	}

	// print the output
//	cout << "x_= " << ekf_.x_ << endl;
//	cout << "P_= " << ekf_.P_ << endl;
}
