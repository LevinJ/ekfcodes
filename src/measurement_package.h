/*
 * measurement_package.h
 *
 *  Created on: Nov 4, 2016
 *      Author: MBRDNA
 */

#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

class MeasurementPackage {
public:
	long timestamp_;

	enum SensorType{
		LASER,
		RADAR
	} sensor_type_;

	Eigen::VectorXd raw_measurements_;

};

#endif /* MEASUREMENT_PACKAGE_H_ */
