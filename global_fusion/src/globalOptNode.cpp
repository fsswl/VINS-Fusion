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

#include "ros/ros.h"
#include "globalOpt.h"
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nmea_msgs/Gprmc.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <stdio.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <queue>
#include <mutex>
#include "tf/tf.h"
#include <math.h>

GlobalOptimization globalEstimator;
ros::Publisher pub_global_odometry, pub_global_path, pub_car, pub_gps_path, pub_uwb_path;
nav_msgs::Path *global_path;
nav_msgs::Path *gps_path;
nav_msgs::Path *uwb_path;
double last_vio_t = -1;
std::queue<sensor_msgs::NavSatFixConstPtr> gpsQueue;
std::queue<nmea_msgs::GprmcPtr> uwbQueue;
std::mutex m_buf;
std::mutex u_buf;
ros::Time gps_cur_time = ros::Time(0);
ros::Duration uwb_offset_time = ros::Duration(0);
double gps_t_start = 0.0;

void publish_car_model(double t, Eigen::Vector3d t_w_car, Eigen::Quaterniond q_w_car)
{
    visualization_msgs::MarkerArray markerArray_msg;
    visualization_msgs::Marker car_mesh;
    car_mesh.header.stamp = ros::Time(t);
    car_mesh.header.frame_id = "world";
    car_mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    car_mesh.action = visualization_msgs::Marker::ADD;
    car_mesh.id = 0;

    car_mesh.mesh_resource = "package://global_fusion/models/car.dae";

    Eigen::Matrix3d rot;
    rot << 1, 0, 0, 0, 1, 0, 0, 0, 1;//0 , 0, -1, 0, -1, 0, -1, 0, 0;
    
    Eigen::Quaterniond Q;
    Q = q_w_car * rot; 
    car_mesh.pose.position.x    = t_w_car.x();
    car_mesh.pose.position.y    = t_w_car.y();
    car_mesh.pose.position.z    = t_w_car.z();
    car_mesh.pose.orientation.w = Q.w();
    car_mesh.pose.orientation.x = Q.x();
    car_mesh.pose.orientation.y = Q.y();
    car_mesh.pose.orientation.z = Q.z();

    car_mesh.color.a = 1.0;
    car_mesh.color.r = 1.0;
    car_mesh.color.g = 0.0;
    car_mesh.color.b = 0.0;

    float major_scale = 1.0;

    car_mesh.scale.x = major_scale;
    car_mesh.scale.y = major_scale;
    car_mesh.scale.z = major_scale;
    markerArray_msg.markers.push_back(car_mesh);
    pub_car.publish(markerArray_msg);
}

void UWB_callback(const nmea_msgs::GprmcPtr &GPS_msg)
{
    //printf("gps_callback! \n");
    if (gps_cur_time.toSec() > 0.)
    {
        if (uwb_offset_time.toSec() == 0.)
        {
            uwb_offset_time = ros::Time::now() - gps_cur_time;
            std::cout << "offset time: " << uwb_offset_time << std::endl;
        }
        else
        {
            GPS_msg->header.stamp = ros::Time::now() - uwb_offset_time;
            u_buf.lock();
            uwbQueue.push(GPS_msg);
            u_buf.unlock();
        }
    }
}

void GPS_callback(const sensor_msgs::NavSatFixConstPtr &GPS_msg)
{
    //printf("gps_callback! \n");
    gps_cur_time = GPS_msg->header.stamp;
    m_buf.lock();
    gpsQueue.push(GPS_msg);
    m_buf.unlock();
}

void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //printf("vio_callback! \n");
    double t = pose_msg->header.stamp.toSec();
    last_vio_t = t;
    Eigen::Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Eigen::Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;
    globalEstimator.inputOdom(t, vio_t, vio_q);

    u_buf.lock();
    while(!uwbQueue.empty())
    {
        nmea_msgs::GprmcPtr GPS_msg = uwbQueue.front();
        double uwb_t = GPS_msg->header.stamp.toSec();
        if(uwb_t >= t - 1 && uwb_t <= t + 1)
        {
            double lat_ms = GPS_msg->lat - floor(GPS_msg->lat / 100) * 100;
            double lon_ms = GPS_msg->lon - floor(GPS_msg->lon / 100) * 100;
            double latitude = floor(GPS_msg->lat / 100) + lat_ms / 60;
            double longitude = floor(GPS_msg->lon / 100) + lon_ms / 60;
            //printf("latms: %lf, lonms: %lf, latm: %lf, lonm: %lf, lats: %lf, lons: %lf, lat: %lf, lon: %lf\n",lat_ms, lon_ms, lat_m, lon_m, lat_s, lon_s, latitude, longitude);
            double altitude = 15;
            double pos_accuracy = 1;
            globalEstimator.inputUWB(t, latitude, longitude, altitude, pos_accuracy);
            uwbQueue.pop();
            break;
        }
        else if(uwb_t < t - 1)
            uwbQueue.pop();
        else if(uwb_t > t + 1)
            break;
    }
    u_buf.unlock();

    m_buf.lock();
    while(!gpsQueue.empty())
    {
        sensor_msgs::NavSatFixConstPtr GPS_msg = gpsQueue.front();

        double gps_t = GPS_msg->header.stamp.toSec();
        //printf("vio t: %f, gps t: %f \n", t, gps_t);
        // 10ms sync tolerance
        if(gps_t >= t - 1 && gps_t <= t + 1)
        {
            //printf("receive GPS with timestamp %f\n", GPS_msg->header.stamp.toSec());
            double latitude = GPS_msg->latitude;
            double longitude = GPS_msg->longitude;
            double altitude = GPS_msg->altitude;
            double pos_accuracy = GPS_msg->position_covariance[0];
            pos_accuracy = 3;
            //int numSats = GPS_msg->status.service;

            if(pos_accuracy <= 0)
                pos_accuracy = 1;
            //printf("receive covariance %lf \n", pos_accuracy);
            //if(GPS_msg->status.status > 8)
            printf("inputGPS \n");
            globalEstimator.inputGPS(t, latitude, longitude, altitude, pos_accuracy);
            gpsQueue.pop();
            break;
        }
        else if(gps_t < t - 1)
            gpsQueue.pop();
        else if(gps_t > t + 1)
            break;
    }
    m_buf.unlock();

    Eigen::Vector3d global_t;
    Eigen:: Quaterniond global_q;
    globalEstimator.getGlobalOdom(global_t, global_q);

    nav_msgs::Odometry odometry;

    odometry.header = pose_msg->header;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    odometry.pose.pose.position.x = global_t.x();
    odometry.pose.pose.position.y = global_t.y();
    odometry.pose.pose.position.z = global_t.z();
    odometry.pose.pose.orientation.x = global_q.x();
    odometry.pose.pose.orientation.y = global_q.y();
    odometry.pose.pose.orientation.z = global_q.z();
    odometry.pose.pose.orientation.w = global_q.w();
    pub_global_odometry.publish(odometry);
    pub_global_path.publish(*global_path);
    pub_gps_path.publish(*gps_path);
    pub_uwb_path.publish(*uwb_path);
    publish_car_model(t, global_t, global_q);


    // write result to file
    std::ofstream foutC("/home/tony-ws1/output/vio_global.csv", ios::app);
    foutC.setf(ios::fixed, ios::floatfield);
    foutC.precision(0);
    foutC << pose_msg->header.stamp.toSec() * 1e9 << ",";
    foutC.precision(5);
    foutC << global_t.x() << ","
            << global_t.y() << ","
            << global_t.z() << ","
            << global_q.w() << ","
            << global_q.x() << ","
            << global_q.y() << ","
            << global_q.z() << endl;
    foutC.close();
}

void pose_graph_path_callback(const nav_msgs::Path::ConstPtr &path_msg)
{
    //printf("vio_callback! \n");
    double t = path_msg->header.stamp.toSec();
    last_vio_t = t;

    globalEstimator.inputPoseGraphPath(path_msg);

    u_buf.lock();
    while(!uwbQueue.empty())
    {
        nmea_msgs::GprmcPtr GPS_msg = uwbQueue.front();
        double uwb_t = GPS_msg->header.stamp.toSec();
        printf("vio t: %f, uwb t: %f \n", t, uwb_t);
        if(uwb_t >= t - 1 && uwb_t <= t + 1)
        {
            printf("fuck vio t: %f, uwb t: %f \n", t, uwb_t);
            double lat_ms = GPS_msg->lat - floor(GPS_msg->lat / 100) * 100;
            double lon_ms = GPS_msg->lon - floor(GPS_msg->lon / 100) * 100;
            double latitude = floor(GPS_msg->lat / 100) + lat_ms / 60;
            double longitude = floor(GPS_msg->lon / 100) + lon_ms / 60;
            //printf("latms: %lf, lonms: %lf, latm: %lf, lonm: %lf, lats: %lf, lons: %lf, lat: %lf, lon: %lf\n",lat_ms, lon_ms, lat_m, lon_m, lat_s, lon_s, latitude, longitude);
            double altitude = 15;
            double pos_accuracy = 1;
            globalEstimator.inputUWB(t, latitude, longitude, altitude, pos_accuracy);
            uwbQueue.pop();
            break;
        }
        else if(uwb_t < t - 1)
            uwbQueue.pop();
        else if(uwb_t > t + 1)
            break;
    }
    u_buf.unlock();

    m_buf.lock();
    while(!gpsQueue.empty())
    {
        sensor_msgs::NavSatFixConstPtr GPS_msg = gpsQueue.front();

        double gps_t = GPS_msg->header.stamp.toSec();
        printf("vio t: %f, gps t: %f \n", t, gps_t);
        // 10ms sync tolerance
        if(gps_t >= t - 1 && gps_t <= t + 1)
        {
            //printf("receive GPS with timestamp %f\n", GPS_msg->header.stamp.toSec());
            if(gps_t_start == 0.0)
                gps_t_start = gps_t;
            double delta_t = gps_t - gps_t_start;
            printf("fuck vio t: %f, gps t: %f, delta_t: %f \n", t, gps_t, delta_t);
            double latitude = GPS_msg->latitude;
            double longitude = GPS_msg->longitude;
            double altitude = GPS_msg->altitude;
            double pos_accuracy = GPS_msg->position_covariance[0];
            //pos_accuracy = 1 + delta_t * 0.01;
            //int numSats = GPS_msg->status.service;

            if(pos_accuracy <= 0)
                pos_accuracy = 1;
            //printf("receive covariance %lf \n", pos_accuracy);
            //if(GPS_msg->status.status > 8)
            printf("inputGPS \n");
            globalEstimator.inputGPS(t, latitude, longitude, altitude, pos_accuracy);
            gpsQueue.pop();
            break;
        }
        else if(gps_t < t - 1)
            gpsQueue.pop();
        else if(gps_t > t + 1)
            break;
    }
    m_buf.unlock();

    Eigen::Vector3d global_t;
    Eigen:: Quaterniond global_q;
    globalEstimator.getGlobalOdom(global_t, global_q);

    nav_msgs::Odometry odometry;

    odometry.header = path_msg->header;
    odometry.header.frame_id = "world";
    odometry.child_frame_id = "world";
    odometry.pose.pose.position.x = global_t.x();
    odometry.pose.pose.position.y = global_t.y();
    odometry.pose.pose.position.z = global_t.z();
    odometry.pose.pose.orientation.x = global_q.x();
    odometry.pose.pose.orientation.y = global_q.y();
    odometry.pose.pose.orientation.z = global_q.z();
    odometry.pose.pose.orientation.w = global_q.w();
    pub_global_odometry.publish(odometry);
    pub_global_path.publish(*global_path);
    pub_gps_path.publish(*gps_path);
    pub_uwb_path.publish(*uwb_path);
    publish_car_model(t, global_t, global_q);


    // write result to file
    std::ofstream foutC("/home/tony-ws1/output/vio_global.csv", ios::app);
    foutC.setf(ios::fixed, ios::floatfield);
    foutC.precision(0);
    foutC << path_msg->header.stamp.toSec() * 1e9 << ",";
    foutC.precision(5);
    foutC << global_t.x() << ","
            << global_t.y() << ","
            << global_t.z() << ","
            << global_q.w() << ","
            << global_q.x() << ","
            << global_q.y() << ","
            << global_q.z() << endl;
    foutC.close();
}

void command()
{
    while(1)
    {
        char c = getchar();
        if (c == 's')
        {
            u_buf.lock();
            m_buf.lock();

            std::ofstream foutC_0("/home/pci/vins_output/global_path.txt", ios::app);
            foutC_0.setf(ios::fixed, ios::floatfield);
            for (unsigned int i = 0; i < global_path->poses.size(); i++)
            {
                foutC_0.precision(0);
                foutC_0 << global_path->poses[i].header.stamp.toSec() * 1e9 << " ";
                foutC_0.precision(5);
                foutC_0 << global_path->poses[i].pose.position.x << " "
                        << global_path->poses[i].pose.position.y << " "
                        << global_path->poses[i].pose.position.z << " "
                        << global_path->poses[i].pose.orientation.x << " "
                        << global_path->poses[i].pose.orientation.y << " "
                        << global_path->poses[i].pose.orientation.z << " "
                        << global_path->poses[i].pose.orientation.w << endl;
            }
            foutC_0.close();

            std::ofstream foutC_1("/home/pci/vins_output/gps_path.txt", ios::app);
            foutC_1.setf(ios::fixed, ios::floatfield);
            for (unsigned int i = 0; i < gps_path->poses.size(); i++)
            {
                foutC_1.precision(0);
                foutC_1 << gps_path->poses[i].header.stamp.toSec() * 1e9 << " ";
                foutC_1.precision(5);
                foutC_1 << gps_path->poses[i].pose.position.x << " "
                        << gps_path->poses[i].pose.position.y << " "
                        << gps_path->poses[i].pose.position.z << " "
                        << gps_path->poses[i].pose.orientation.x << " "
                        << gps_path->poses[i].pose.orientation.y << " "
                        << gps_path->poses[i].pose.orientation.z << " "
                        << gps_path->poses[i].pose.orientation.w << endl;
            }
            foutC_1.close();

            std::ofstream foutC_2("/home/pci/vins_output/uwb_path.txt", ios::app);
            foutC_2.setf(ios::fixed, ios::floatfield);
            for (unsigned int i = 0; i < uwb_path->poses.size(); i++)
            {
                foutC_2.precision(0);
                foutC_2 << uwb_path->poses[i].header.stamp.toSec() * 1e9 << " ";
                foutC_2.precision(5);
                foutC_2 << uwb_path->poses[i].pose.position.x << " "
                        << uwb_path->poses[i].pose.position.y << " "
                        << uwb_path->poses[i].pose.position.z << " "
                        << uwb_path->poses[i].pose.orientation.x << " "
                        << uwb_path->poses[i].pose.orientation.y << " "
                        << uwb_path->poses[i].pose.orientation.z << " "
                        << uwb_path->poses[i].pose.orientation.w << endl;
            }
            foutC_2.close();

            m_buf.unlock();
            u_buf.unlock();
            printf("save path finish\n");
            printf("program shutting down...\n");
            ros::shutdown();
        }
        else if (c == 'e')
        {
            globalEstimator.optimize_para_ex();
        }

        std::chrono::milliseconds dura(10);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "globalEstimator");
    ros::NodeHandle n("~");

    global_path = &globalEstimator.global_path;
    gps_path = &globalEstimator.gps_path;
    uwb_path = &globalEstimator.uwb_path;

    ros::Subscriber sub_GPS = n.subscribe("/gps", 100, GPS_callback);
    //ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 100, vio_callback);
    ros::Subscriber sub_pose_graph_path = n.subscribe("/loop_fusion/pose_graph_path", 100, pose_graph_path_callback);
    ros::Subscriber sub_UWB = n.subscribe("/uwb/gps_info", 100, UWB_callback);
    pub_global_path = n.advertise<nav_msgs::Path>("global_path", 100);
    pub_gps_path = n.advertise<nav_msgs::Path>("gps_path", 100);
    pub_uwb_path = n.advertise<nav_msgs::Path>("uwb_path", 100);
    pub_global_odometry = n.advertise<nav_msgs::Odometry>("global_odometry", 100);
    pub_car = n.advertise<visualization_msgs::MarkerArray>("car_model", 1000);

    std::thread keyboard_command_process;
    keyboard_command_process = std::thread(command);

    ros::spin();
    return 0;
}
