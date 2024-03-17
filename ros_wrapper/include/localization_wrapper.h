#pragma once

#include <fstream>
#include <memory>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <imu_gps_localization/uwb.h>

#include "imu_gps_localizer/imu_gps_localizer.h"

class LocalizationWrapper {
public:
    LocalizationWrapper(ros::NodeHandle& nh);
    ~LocalizationWrapper();

    void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

    void GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr);

    void UwbCallback(const imu_gps_localization::uwbConstPtr& uwb_msg_ptr);

private:
    void LogState(const ImuGpsLocalization::State& state);
    void LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data, Eigen::Vector3d gps_enu);

    void ConvertStateToRosTopic(const ImuGpsLocalization::State& state);

    void ConvertGps_enuToRosTopic(const Eigen::Vector3d& gps_enu);

    void ConvertUwbToRosTopic(const ImuGpsLocalization::UwbDataPtr& uwb_data);
    
    ros::Subscriber imu_sub_;
    ros::Subscriber gps_position_sub_;
    ros::Subscriber uwb_sub_;
    ros::Publisher state_pub_;
    ros::Publisher gps_pub_;
    ros::Publisher uwb_pub_;

    std::ofstream file_state_;
    std::ofstream file_gps_;
    std::ofstream file_uwb_;

    nav_msgs::Path ros_path_;
    nav_msgs::Path gps_path_;
    nav_msgs::Path uwb_path_;

    std::unique_ptr<ImuGpsLocalization::ImuGpsLocalizer> imu_gps_localizer_ptr_;
};