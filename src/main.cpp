/**
 * main.cpp
 *
 *  Created on: Feb 15, 2017
 *      Author: MBRDNA
 */

#include <iostream>
#include "Eigen/Dense"
#include <vector>
#include "fusion_ekf.h"
#include "measurement_package.h"
#include "ground_truth_package.h"
#include "tools.h"
#include <fstream>
#include <sstream>
#include <stdlib.h>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

int main(int argc, char* argv[]) {

	string usage_instructions = "Usage instructions: ./ExtendedKF path/to/input.txt output.txt";
	bool has_valid_args = false;

	// make sure the user has provided input and output files
	if (argc == 1)
	{
		cerr << usage_instructions << endl;
	}

	if (argc == 2)
	{
		cerr << "Please include an output file.\n" << usage_instructions << endl;
	}

	if (argc == 3)
	{
		has_valid_args = true;
	}

	if (argc > 3)
	{
		cerr << "Too many arguments.\n" << usage_instructions << endl;
	}

	if (!has_valid_args)
	{
		exit(EXIT_FAILURE);
	}

	/*****************************************************************************
	 *	Local configuration																 *
	 ****************************************************************************/

	// true = generate measurement packages together with ground truth
	// bool use_gt = true;

	/*****************************************************************************
	 *	Set Measurements															 *
	 ****************************************************************************/
	vector<MeasurementPackage> measurement_pack_list;

	// test list with measurement packages
	vector<GroundTruthPackage> gt_pack_list;

	// get input file
	string in_file_name_ = argv[1];
	ifstream in_file(in_file_name_.c_str(), std::ifstream::in);

	// prep output file
	string out_file_name_ = argv[2];
	std::ofstream out_file_(out_file_name_.c_str(), std::ofstream::out);

	if (!in_file.is_open())
	{
		cerr << "Cannot open input file: " << in_file_name_ << endl;
		exit(EXIT_FAILURE);
	}

	if (!out_file_.is_open())
	{
		cerr << "Cannot Open output file: " << out_file_name_ << endl;
		exit(EXIT_FAILURE);
	}

	string line;

	// run the filter on each measurement!
	while (getline(in_file, line)) {

		string sensor_type;
		MeasurementPackage meas_package;
		GroundTruthPackage gt_package;

		istringstream iss(line);
		iss >> sensor_type;	//reads first element from the current line
		long timestamp;
		if (sensor_type.compare("L") == 0) {
			// laser measurement

			// read measurements
			meas_package.sensor_type_ = MeasurementPackage::LASER;
			meas_package.raw_measurements_ = VectorXd(2);
			float x;
			float y;
			iss >> x;
			iss >> y;
			meas_package.raw_measurements_ << x, y;
			iss >> timestamp;
			meas_package.timestamp_ = timestamp;
			measurement_pack_list.push_back(meas_package);

		} else if (sensor_type.compare("R") == 0) {
			// radar measurement

			// read measurements
			meas_package.sensor_type_ = MeasurementPackage::RADAR;
			meas_package.raw_measurements_ = VectorXd(3);
			float ro;
			float theta;
			float ro_dot;
			iss >> ro;
			iss >> theta;
			iss >> ro_dot;
			meas_package.raw_measurements_ << ro, theta, ro_dot;
			iss >> timestamp;
			meas_package.timestamp_ = timestamp;
			measurement_pack_list.push_back(meas_package);
		}

		// read ground truth
		float x_gt;
		float y_gt;
		float vx_gt;
		float vy_gt;
		iss >> x_gt;
		iss >> y_gt;
		iss >> vx_gt;
		iss >> vy_gt;
		gt_package.gt_values_ = VectorXd(4);
		gt_package.gt_values_ << x_gt, y_gt, vx_gt, vy_gt;
		gt_pack_list.push_back(gt_package);
	}

	// Create a FusionEKF instance
	FusionEKF fusion_ekf;

	// accumulate state estimations and corresponding ground truths into separate
	// lists used to compute the RMSE
	vector<VectorXd> estimations;
	vector<VectorXd> ground_truth;

	//Call the EKF-based fusion
	size_t N = measurement_pack_list.size();
	for (size_t k = 0; k < N; ++k) {
		// start filtering from the second frame (the speed is unknown in the first
		// frame)
		fusion_ekf.ProcessMeasurement(measurement_pack_list[k]);

		// output the estimation
		out_file_ << fusion_ekf.ekf_.x_(0) << "\t";
		out_file_ << fusion_ekf.ekf_.x_(1) << "\t";
		out_file_ << fusion_ekf.ekf_.x_(2) << "\t";
		out_file_ << fusion_ekf.ekf_.x_(3) << "\t";

		// output the measurements
		if (measurement_pack_list[k].sensor_type_
				== MeasurementPackage::LASER) {
			// output the estimation
			out_file_ << measurement_pack_list[k].raw_measurements_(0) << "\t";
			out_file_ << measurement_pack_list[k].raw_measurements_(1) << "\t";
		} else if (measurement_pack_list[k].sensor_type_
				== MeasurementPackage::RADAR) {
			// output the estimation in the cartesian coordinates
			float ro = measurement_pack_list[k].raw_measurements_(0);
			float phi = measurement_pack_list[k].raw_measurements_(1);
			out_file_ << ro * cos(phi) << "\t";
			out_file_ << ro * sin(phi) << "\t";
		}

		// output the gt packages
		out_file_ << gt_pack_list[k].gt_values_(0) << "\t";
		out_file_ << gt_pack_list[k].gt_values_(1) << "\t";
		out_file_ << gt_pack_list[k].gt_values_(2) << "\t";
		out_file_ << gt_pack_list[k].gt_values_(3) << "\n";


		estimations.push_back(fusion_ekf.ekf_.x_);
		ground_truth.push_back(gt_pack_list[k].gt_values_);
	}

	// compute the accuracy (RMSE)
	Tools tools;
	VectorXd rmse = tools.CalculateRMSE(estimations, ground_truth);
	cout << "Accuracy - RMSE:\n" << rmse << endl;

	// close files
	if (out_file_.is_open())
	{
		out_file_.close();
	}

	if (in_file.is_open())
	{
		in_file.close();
	}

	return 0;
}
