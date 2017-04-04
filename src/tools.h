/*
 * tools.h
 *
 *  Created on: Feb 15, 2017
 *      Author: MBRDNA
 */

#ifndef TOOLS_H_
#define TOOLS_H_
#include <vector>
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

class Tools {
public:
	Tools();
	virtual ~Tools();
	VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);
	MatrixXd CalculateJacobian(const VectorXd& x_state);
};

#endif /* TOOLS_H_ */
