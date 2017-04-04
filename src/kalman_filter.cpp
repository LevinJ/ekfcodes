/**
 * kalman_filter.cpp
 *
 *  Created on: Nov 4, 2016
 *      Author: MBRDNA
 */

#include "kalman_filter.h"

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
												MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
	x_ = x_in;
	P_ = P_in;
	F_ = F_in;
	H_ = H_in;
	R_ = R_in;
	Q_ = Q_in;
}

void KalmanFilter::Predict() {
	//predict the state
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	//Quiz: the difference between Qx and Qv
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
	//Error y
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;

	//Kalman Gain
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;

	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	// update
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);

	P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateWithAlreadyPredictedMeasurements(const VectorXd &z, const VectorXd &z_pred) {
	// Error y
	// VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;

	// angle normalization
	while (y(1)> M_PI) y(1)-=2.*M_PI;
	while (y(1)<-M_PI) y(1)+=2.*M_PI;

	// Kalman Gain
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;

	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	// update
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);

	P_ = (I - K * H_) * P_;
}

void KalmanFilter::Filter() {}