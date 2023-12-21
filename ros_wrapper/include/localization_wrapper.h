#pragma once

#include <fstream>
#include <memory>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include "imu_gps_localizer/imu_gps_localizer.h"

class LocalizationWrapper {
public:
    LocalizationWrapper(ros::NodeHandle& nh);
    ~LocalizationWrapper();

    void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

    void MagCallBack(const sensor_msgs::MagneticFieldConstPtr& mag_msg_ptr);

    void GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr);

private:
    void LogState(const ImuGpsLocalization::State& state);
    void LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data, Eigen::Vector3d gps_enu);

    void ConvertStateToRosTopic(const ImuGpsLocalization::State& state);

    void ConvertGps_enuToRosTopic(const Eigen::Vector3d& gps_enu);
    
    ros::Subscriber imu_sub_;
    ros::Subscriber mag_sub_;  // added Subscriber to accept the topic /imu/mag
    ros::Subscriber gps_position_sub_;
    ros::Publisher state_pub_;
    ros::Publisher gps_pub;

    std::ofstream file_state_;
    std::ofstream file_gps_;

    nav_msgs::Path ros_path_;
    nav_msgs::Path gps_path_;

    std::unique_ptr<ImuGpsLocalization::ImuGpsLocalizer> imu_gps_localizer_ptr_;
};