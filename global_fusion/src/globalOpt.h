/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#pragma once
#include <vector>
#include <map>
#include <iostream>
#include <mutex>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include "LocalCartesian.hpp"
#include "tic_toc.h"

using namespace std;

class GlobalOptimization
{
public:
	GlobalOptimization();
	~GlobalOptimization();
	void inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy);
	void inputUWB(double t, double latitude, double longitude, double altitude, double posAccuracy);
	void inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ);
	void inputPoseGraphPath(const nav_msgs::Path::ConstPtr &path_msg);
	void getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ);
	void optimize_para_ex();
	nav_msgs::Path global_path;
	nav_msgs::Path gps_path;
	nav_msgs::Path uwb_path;

private:
	void GPS2XYZ(double latitude, double longitude, double altitude, double* xyz);
	void UWB2XYZ(double latitude, double longitude, double altitude, double* xyz);
	void optimize();
	void updateGlobalPath();

	// format t, tx,ty,tz,qw,qx,qy,qz
	map<double, vector<double>> localPoseMap;
	map<double, vector<double>> globalPoseMap;
	map<double, vector<double>> GPSPositionMap;
	bool initGPS;
	bool initUWB;
	bool newGPS;
	bool newUWB;
	GeographicLib::LocalCartesian geoConverter;
	GeographicLib::LocalCartesian geoConverter1;
	std::mutex mPoseMap;
	Eigen::Matrix4d WGPS_T_WVIO;
	Eigen::Vector3d ex_vio_gps;
	Eigen::Vector3d lastP;
	Eigen::Quaterniond lastQ;
	std::thread threadOpt;

};