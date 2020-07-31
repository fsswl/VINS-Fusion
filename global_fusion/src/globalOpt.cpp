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

#include "globalOpt.h"
#include "Factors.h"
#include "ros/ros.h"
#include "tf/tf.h"

#define TRANSFORM_POSE_GRAPH_PATH

extern ros::Publisher pub_gps_path;

GlobalOptimization::GlobalOptimization()
{
	initGPS = false;
    initUWB = false;
    newGPS = false;
    ex_vio_gps = Eigen::Vector3d(2.32996, 0.30695, -0.538865);
#ifndef TRANSFORM_POSE_GRAPH_PATH
	WGPS_T_WVIO = Eigen::Matrix4d::Identity();
#else
    WGPS_T_WVIO <<  0.487693,   -0.873011,  -0.0028463,    0.877877,
                    0.873015,    0.487693, 0.000710993,   -0.348232,
                    0.000767416, -0.00283161,    0.999996,           0,
                    0.,         0.,         0.,          1.;
#endif

    //threadOpt = std::thread(&GlobalOptimization::optimize, this);
}

GlobalOptimization::~GlobalOptimization()
{
    threadOpt.detach();
}

void GlobalOptimization::GPS2XYZ(double latitude, double longitude, double altitude, double* xyz)
{
    if(!initGPS)
    {
        geoConverter.Reset(latitude, longitude, altitude);
        initGPS = true;
    }
    geoConverter.Forward(latitude, longitude, altitude, xyz[0], xyz[1], xyz[2]);
    //printf("la: %f lo: %f al: %f\n", latitude, longitude, altitude);
    //printf("gps x: %f y: %f z: %f\n", xyz[0], xyz[1], xyz[2]);
}

void GlobalOptimization::UWB2XYZ(double latitude, double longitude, double altitude, double* xyz)
{
    if(!initUWB)
    {
        geoConverter1.Reset(latitude, longitude, altitude);
        initUWB = true;
    }
    geoConverter1.Forward(latitude, longitude, altitude, xyz[0], xyz[1], xyz[2]);
    //printf("la: %f lo: %f al: %f\n", latitude, longitude, altitude);
    //printf("gps x: %f y: %f z: %f\n", xyz[0], xyz[1], xyz[2]);
}

void GlobalOptimization::inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ)
{
	mPoseMap.lock();
    vector<double> localPose{OdomP.x(), OdomP.y(), OdomP.z(), 
    					     OdomQ.w(), OdomQ.x(), OdomQ.y(), OdomQ.z()};
    localPoseMap[t] = localPose;


    Eigen::Quaterniond globalQ;
    globalQ = WGPS_T_WVIO.block<3, 3>(0, 0) * OdomQ;
    Eigen::Vector3d globalP = WGPS_T_WVIO.block<3, 3>(0, 0) * OdomP + WGPS_T_WVIO.block<3, 1>(0, 3);
    vector<double> globalPose{globalP.x(), globalP.y(), globalP.z(),
                              globalQ.w(), globalQ.x(), globalQ.y(), globalQ.z()};
    globalPoseMap[t] = globalPose;
    lastP = globalP;
    lastQ = globalQ;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(t);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = lastP.x();
    pose_stamped.pose.position.y = lastP.y();
    pose_stamped.pose.position.z = lastP.z();
    pose_stamped.pose.orientation.x = lastQ.x();
    pose_stamped.pose.orientation.y = lastQ.y();
    pose_stamped.pose.orientation.z = lastQ.z();
    pose_stamped.pose.orientation.w = lastQ.w();
    global_path.header = pose_stamped.header;
    global_path.poses.push_back(pose_stamped);

    mPoseMap.unlock();
}

void GlobalOptimization::inputPoseGraphPath(const nav_msgs::Path::ConstPtr &path_msg)
{
    Eigen::Quaterniond globalQ;
    Eigen::Vector3d globalP;
    double t;

    mPoseMap.lock();

    /*localPoseMap.clear();
    globalPoseMap.clear();
    global_path.poses.clear();

    for (unsigned int i = 0; i < path_msg->poses.size(); i++)
    {
        t = path_msg->poses[i].header.stamp.toSec();
        Eigen::Vector3d vio_t(path_msg->poses[i].pose.position.x, path_msg->poses[i].pose.position.y, path_msg->poses[i].pose.position.z);
        Eigen::Quaterniond vio_q;
        vio_q.w() = path_msg->poses[i].pose.orientation.w;
        vio_q.x() = path_msg->poses[i].pose.orientation.x;
        vio_q.y() = path_msg->poses[i].pose.orientation.y;
        vio_q.z() = path_msg->poses[i].pose.orientation.z;
        vector<double> localPose{vio_t.x(), vio_t.y(), vio_t.z(),
                                 vio_q.w(), vio_q.x(), vio_q.y(), vio_q.z()};
        localPoseMap[t] = localPose;

        globalQ = WGPS_T_WVIO.block<3, 3>(0, 0) * vio_q;
        globalP = WGPS_T_WVIO.block<3, 3>(0, 0) * vio_t + WGPS_T_WVIO.block<3, 1>(0, 3);
        vector<double> globalPose{globalP.x(), globalP.y(), globalP.z(),
                                  globalQ.w(), globalQ.x(), globalQ.y(), globalQ.z()};
        globalPoseMap[t] = globalPose;

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(t);
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = globalP.x();
        pose_stamped.pose.position.y = globalP.y();
        pose_stamped.pose.position.z = globalP.z();
        pose_stamped.pose.orientation.x = globalQ.x();
        pose_stamped.pose.orientation.y = globalQ.y();
        pose_stamped.pose.orientation.z = globalQ.z();
        pose_stamped.pose.orientation.w = globalQ.w();
        global_path.header = pose_stamped.header;
        global_path.poses.push_back(pose_stamped);
    }*/

    unsigned int i = path_msg->poses.size() - 1;
    t = path_msg->poses[i].header.stamp.toSec();
    Eigen::Vector3d vio_t(path_msg->poses[i].pose.position.x, path_msg->poses[i].pose.position.y, path_msg->poses[i].pose.position.z);
    Eigen::Quaterniond vio_q;
    vio_q.w() = path_msg->poses[i].pose.orientation.w;
    vio_q.x() = path_msg->poses[i].pose.orientation.x;
    vio_q.y() = path_msg->poses[i].pose.orientation.y;
    vio_q.z() = path_msg->poses[i].pose.orientation.z;
    Eigen::Vector3d OdomP_gps = vio_q.toRotationMatrix() * ex_vio_gps + vio_t;

    vector<double> localPose{vio_t.x(), vio_t.y(), vio_t.z(),
                         vio_q.w(), vio_q.x(), vio_q.y(), vio_q.z()};
    localPoseMap[t] = localPose;

    globalQ = WGPS_T_WVIO.block<3, 3>(0, 0) * vio_q;
#ifndef TRANSFORM_POSE_GRAPH_PATH
    globalP = WGPS_T_WVIO.block<3, 3>(0, 0) * vio_t + WGPS_T_WVIO.block<3, 1>(0, 3);
#else
    globalP = WGPS_T_WVIO.block<3, 3>(0, 0) * OdomP_gps + WGPS_T_WVIO.block<3, 1>(0, 3);
#endif
    vector<double> globalPose{globalP.x(), globalP.y(), globalP.z(),
                              globalQ.w(), globalQ.x(), globalQ.y(), globalQ.z()};
    globalPoseMap[t] = globalPose;

    lastP = globalP;
    lastQ = globalQ;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(t);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = lastP.x();
    pose_stamped.pose.position.y = lastP.y();
    pose_stamped.pose.position.z = lastP.z();
    pose_stamped.pose.orientation.x = lastQ.x();
    pose_stamped.pose.orientation.y = lastQ.y();
    pose_stamped.pose.orientation.z = lastQ.z();
    pose_stamped.pose.orientation.w = lastQ.w();
    global_path.header = pose_stamped.header;
    global_path.poses.push_back(pose_stamped);

    mPoseMap.unlock();
}

void GlobalOptimization::getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ)
{
    odomP = lastP;
    odomQ = lastQ;
}

void GlobalOptimization::inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy)
{
	double xyz[3];
	GPS2XYZ(latitude, longitude, altitude, xyz);
	vector<double> tmp{xyz[0], xyz[1], xyz[2], posAccuracy};
    //printf("new gps: t: %f x: %f y: %f z:%f \n", t, tmp[0], tmp[1], tmp[2]);
	//GPSPositionMap[t] = tmp;
    //newGPS = true;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(t);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = xyz[0];
    pose_stamped.pose.position.y = xyz[1];
    pose_stamped.pose.position.z = xyz[2];
    geometry_msgs::Quaternion ori = tf::createQuaternionMsgFromYaw(0);
    pose_stamped.pose.orientation = ori;
    gps_path.header = pose_stamped.header;
    gps_path.poses.push_back(pose_stamped);
}

void GlobalOptimization::inputUWB(double t, double latitude, double longitude, double altitude, double posAccuracy)
{
    double xyz[3];
    UWB2XYZ(latitude, longitude, altitude, xyz);
    vector<double> tmp{xyz[0], xyz[1], xyz[2], posAccuracy};
    //printf("new gps: t: %f x: %f y: %f z:%f \n", t, tmp[0], tmp[1], tmp[2]);
    GPSPositionMap[t] = tmp;
    newGPS = true;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(t);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = xyz[0];
    pose_stamped.pose.position.y = xyz[1];
    pose_stamped.pose.position.z = xyz[2];
    geometry_msgs::Quaternion ori = tf::createQuaternionMsgFromYaw(0);
    pose_stamped.pose.orientation = ori;
    uwb_path.header = pose_stamped.header;
    uwb_path.poses.push_back(pose_stamped);
}

void GlobalOptimization::optimize()
{
    while(true)
    {
        if(newGPS)
        {
            newGPS = false;
            printf("global optimization\n");
            TicToc globalOptimizationTime;

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //options.minimizer_progress_to_stdout = true;
            //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
            options.max_num_iterations = 5;
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(1.0);
            ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();

            //add param
            mPoseMap.lock();
            int length = localPoseMap.size();
            // w^t_i   w^q_i
            double t_array[length][3];
            double q_array[length][4];
            map<double, vector<double>>::iterator iter;
            iter = globalPoseMap.begin();
            for (int i = 0; i < length; i++, iter++)
            {
                t_array[i][0] = iter->second[0];
                t_array[i][1] = iter->second[1];
                t_array[i][2] = iter->second[2];
                q_array[i][0] = iter->second[3];
                q_array[i][1] = iter->second[4];
                q_array[i][2] = iter->second[5];
                q_array[i][3] = iter->second[6];
                problem.AddParameterBlock(q_array[i], 4, local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);
            }

            map<double, vector<double>>::iterator iterVIO, iterVIONext, iterGPS;
            int i = 0;
            for (iterVIO = localPoseMap.begin(); iterVIO != localPoseMap.end(); iterVIO++, i++)
            {
                //vio factor
                iterVIONext = iterVIO;
                iterVIONext++;
                if(iterVIONext != localPoseMap.end())
                {
                    Eigen::Matrix4d wTi = Eigen::Matrix4d::Identity();
                    Eigen::Matrix4d wTj = Eigen::Matrix4d::Identity();
                    wTi.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIO->second[3], iterVIO->second[4], 
                                                               iterVIO->second[5], iterVIO->second[6]).toRotationMatrix();
                    wTi.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIO->second[0], iterVIO->second[1], iterVIO->second[2]);
                    wTj.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIONext->second[3], iterVIONext->second[4], 
                                                               iterVIONext->second[5], iterVIONext->second[6]).toRotationMatrix();
                    wTj.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIONext->second[0], iterVIONext->second[1], iterVIONext->second[2]);
                    Eigen::Matrix4d iTj = wTi.inverse() * wTj;
                    Eigen::Quaterniond iQj;
                    iQj = iTj.block<3, 3>(0, 0);
                    Eigen::Vector3d iPj = iTj.block<3, 1>(0, 3);

                    ceres::CostFunction* vio_function = RelativeRTError::Create(iPj.x(), iPj.y(), iPj.z(),
                                                                                iQj.w(), iQj.x(), iQj.y(), iQj.z(),
                                                                                0.1, 0.01);
                    problem.AddResidualBlock(vio_function, NULL, q_array[i], t_array[i], q_array[i+1], t_array[i+1]);

                    /*
                    double **para = new double *[4];
                    para[0] = q_array[i];
                    para[1] = t_array[i];
                    para[3] = q_array[i+1];
                    para[4] = t_array[i+1];

                    double *tmp_r = new double[6];
                    double **jaco = new double *[4];
                    jaco[0] = new double[6 * 4];
                    jaco[1] = new double[6 * 3];
                    jaco[2] = new double[6 * 4];
                    jaco[3] = new double[6 * 3];
                    vio_function->Evaluate(para, tmp_r, jaco);

                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 1>>(tmp_r).transpose() << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>>(jaco[0]) << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>>(jaco[1]) << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 4, Eigen::RowMajor>>(jaco[2]) << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>>(jaco[3]) << std::endl
                        << std::endl;
                    */

                }
                //gps factor
                double t = iterVIO->first;
                iterGPS = GPSPositionMap.find(t);
                if (iterGPS != GPSPositionMap.end())
                {
                    ceres::CostFunction* gps_function = TError::Create(iterGPS->second[0], iterGPS->second[1], 
                                                                       iterGPS->second[2], iterGPS->second[3]);
                    //printf("inverse weight %f \n", iterGPS->second[3]);
                    problem.AddResidualBlock(gps_function, loss_function, t_array[i]);

                    /*
                    double **para = new double *[1];
                    para[0] = t_array[i];

                    double *tmp_r = new double[3];
                    double **jaco = new double *[1];
                    jaco[0] = new double[3 * 3];
                    gps_function->Evaluate(para, tmp_r, jaco);

                    std::cout << Eigen::Map<Eigen::Matrix<double, 3, 1>>(tmp_r).transpose() << std::endl
                        << std::endl;
                    std::cout << Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(jaco[0]) << std::endl
                        << std::endl;
                    */
                }

            }
            //mPoseMap.unlock();
            ceres::Solve(options, &problem, &summary);
            //std::cout << summary.BriefReport() << "\n";

            // update global pose
            //mPoseMap.lock();
            iter = globalPoseMap.begin();
            for (int i = 0; i < length; i++, iter++)
            {
            	vector<double> globalPose{t_array[i][0], t_array[i][1], t_array[i][2],
            							  q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]};
            	iter->second = globalPose;
            	if(i == length - 1)
            	{
            	    Eigen::Matrix4d WVIO_T_body = Eigen::Matrix4d::Identity(); 
            	    Eigen::Matrix4d WGPS_T_body = Eigen::Matrix4d::Identity();
            	    double t = iter->first;
            	    WVIO_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(localPoseMap[t][3], localPoseMap[t][4], 
            	                                                       localPoseMap[t][5], localPoseMap[t][6]).toRotationMatrix();
            	    WVIO_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(localPoseMap[t][0], localPoseMap[t][1], localPoseMap[t][2]);
            	    WGPS_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(globalPose[3], globalPose[4], 
            	                                                        globalPose[5], globalPose[6]).toRotationMatrix();
            	    WGPS_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(globalPose[0], globalPose[1], globalPose[2]);
            	    WGPS_T_WVIO = WGPS_T_body * WVIO_T_body.inverse();
            	}
            }
            updateGlobalPath();
            std::cout << "WGPS_T_WVIO: " << WGPS_T_WVIO << std::endl;
            //printf("global time %f \n", globalOptimizationTime.toc());
            mPoseMap.unlock();
        }
        std::chrono::milliseconds dura(100);
        std::this_thread::sleep_for(dura);
    }
	return;
}

void GlobalOptimization::optimize_para_ex()
{
    printf("para ex optimization\n");
    TicToc globalOptimizationTime;

    ceres::Problem problem;
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
    //options.minimizer_progress_to_stdout = true;
    //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
    //options.max_num_iterations = 100;
    ceres::Solver::Summary summary;
    ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();

    //add param
    mPoseMap.lock();

    double t_result[3];
    double q_result[4];
    double t_ex_result[3];

    t_result[0] = 0;
    t_result[1] = 0;
    //t_result[2] = 0;
    q_result[0] = 1;
    q_result[1] = 0;
    q_result[2] = 0;
    q_result[3] = 0;
    t_ex_result[0] = 0;
    t_ex_result[1] = 0;
    t_ex_result[2] = 0;
    problem.AddParameterBlock(q_result, 4, local_parameterization);
    problem.AddParameterBlock(t_result, 2);
    problem.AddParameterBlock(t_ex_result, 3);
    //problem.SetParameterBlockConstant(&t_result[2]);

    map<double, vector<double>>::iterator iterVIO, iterGPS;
    int i = 0;
    for (iterVIO = localPoseMap.begin(), iterGPS = GPSPositionMap.begin(); (iterVIO != localPoseMap.end()) && (iterGPS != GPSPositionMap.end()); iterVIO++, iterGPS++, i++)
    {
        Eigen::Vector3d p_vio = Eigen::Vector3d(iterVIO->second[0], iterVIO->second[1], iterVIO->second[2]);
        Eigen::Quaterniond q_vio = Eigen::Quaterniond(iterVIO->second[3], iterVIO->second[4], iterVIO->second[5], iterVIO->second[6]);
        Eigen::Vector3d p_gps = Eigen::Vector3d(iterGPS->second[0], iterGPS->second[1], iterGPS->second[2]);

        ceres::CostFunction* para_ex_function = exRelativeRTError::Create(p_vio.x(), p_vio.y(), p_vio.z(),
                                                                    q_vio.w(), q_vio.x(), q_vio.y(), q_vio.z(),
                                                                    p_gps.x(), p_gps.y(), p_gps.z(), 1);
        problem.AddResidualBlock(para_ex_function, NULL, q_result, t_result, t_ex_result);
    }
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << std::endl;
    Eigen::Matrix4d qt_result = Eigen::Matrix4d::Identity();
    qt_result.block<3, 3>(0, 0) = Eigen::Quaterniond(q_result[0], q_result[1], q_result[2], q_result[3]).toRotationMatrix();
    qt_result.block<3, 1>(0, 3) = Eigen::Vector3d(t_result[0], t_result[1], 0.);
    std::cout << "t_result: " << t_result[0] << ", " << t_result[1] << ", " << "0." << ", " << std::endl;
    std::cout << "q_result: " << q_result[0] << ", " << q_result[1] << ", " << q_result[2] << ", " << q_result[3] << ", " << std::endl;
    std::cout << "qt_martix: " << qt_result << std::endl;
    std::cout << "t_ex_result: " << t_ex_result[0] << ", " << t_ex_result[1] << ", " << t_ex_result[2] << ", " << std::endl;
    //printf("global time %f \n", globalOptimizationTime.toc());
    mPoseMap.unlock();

    return;
}

void GlobalOptimization::updateGlobalPath()
{
    global_path.poses.clear();
    map<double, vector<double>>::iterator iter;
    for (iter = globalPoseMap.begin(); iter != globalPoseMap.end(); iter++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(iter->first);
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = iter->second[0];
        pose_stamped.pose.position.y = iter->second[1];
        pose_stamped.pose.position.z = iter->second[2];
        pose_stamped.pose.orientation.w = iter->second[3];
        pose_stamped.pose.orientation.x = iter->second[4];
        pose_stamped.pose.orientation.y = iter->second[5];
        pose_stamped.pose.orientation.z = iter->second[6];
        global_path.poses.push_back(pose_stamped);
    }
}