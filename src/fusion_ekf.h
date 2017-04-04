/*
 * fusion_ekf.h
 *
 *  Created on: Nov 3, 2016
 *      Author: MBRDNA
 */

#ifndef FUSION_KF_H_
#define FUSION_KF_H_

#include "measurement_package.h"
#include <vector>
#include <string>
#include <fstream>
#include "kalman_filter.h"
#include "tools.h"

class FusionEKF {
public:
	FusionEKF();
	virtual ~FusionEKF();
	void ProcessMeasurement(const MeasurementPackage &measurement_pack);
	MatrixXd CalculateJacobian(const VectorXd &x_state);
	KalmanFilter ekf_;

private:
	// check whether the tracking toolbox was initiallized or not (first measurement)
	bool is_initialized_;
	// previous timestamp
	long previous_timestamp_;
	// tool object used to compute Jacobian and RMSE
	Tools tools;
	MatrixXd R_laser_;
	MatrixXd R_radar_;
	MatrixXd H_laser_;
	MatrixXd Hj_;
};

#endif /* FUSION_KF_H_ */
