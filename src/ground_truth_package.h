/*
 * ground_truth_package.h
 *
 *  Created on: Nov 4, 2016
 *      Author: MBRDNA
 */

#ifndef GROUND_TRUTH_PACKAGE_H_
#define GROUND_TRUTH_PACKAGE_H_

#include "Eigen/Dense"

class GroundTruthPackage {
public:
	long timestamp_;

	enum SensorType{
		LASER,
		RADAR
	} sensor_type_;

	Eigen::VectorXd gt_values_;

};

#endif /* GROUND_TRUTH_PACKAGE_H_ */
