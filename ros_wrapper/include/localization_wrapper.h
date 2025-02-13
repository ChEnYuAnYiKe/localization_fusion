#pragma once

#include "imu_gps_localizer/imu_gps_localizer.h"
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
// #include <imu_gps_localization/uwb.h>
#include <memory>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/LaserScan.h>

class LocalizationWrapper
{
public:
	LocalizationWrapper(ros::NodeHandle &nh);
	~LocalizationWrapper();

	void ImuCallback(const sensor_msgs::ImuConstPtr &imu_msg_ptr);

	// void GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr&
	// gps_msg_ptr);

	void UwbCallback(const geometry_msgs::PoseStamped::ConstPtr &uwb_msg_ptr);

	// void LidarCallback(const geometry_msgs::PoseStamped::ConstPtr &lidar_msg_ptr);

    void LidarCallback(const sensor_msgs::LaserScan::ConstPtr &lidar_msg_ptr);

    void attitudeCallback(const sensor_msgs::Imu::ConstPtr &msg);

private:
	void LogState(const ImuGpsLocalization::State &state);
	// void LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data,
	// Eigen::Vector3d gps_enu);

	void ConvertStateToRosTopic(const ImuGpsLocalization::State &state);

	// void ConvertGps_enuToRosTopic(const Eigen::Vector3d& gps_enu);

	void ConvertUwbToRosTopic(const ImuGpsLocalization::UwbDataPtr &uwb_data);

	ros::Subscriber imu_sub_;
	// ros::Subscriber gps_position_sub_;
	ros::Subscriber uwb_sub_;
	ros::Subscriber lidar_sub_;
    ros::Subscriber attitude_sub_;

    ros::Publisher state_pub_;
	// ros::Publisher gps_pub_;
	ros::Publisher uwb_pub_;
	ros::Publisher velocity_filter_pub_;
	ros::Publisher position_filter_pub_;
    ros::Publisher odom_pub_;

    std::ofstream file_state_;
	// std::ofstream file_gps_;
	std::ofstream file_uwb_;

	nav_msgs::Path ros_path_;
	// nav_msgs::Path gps_path_;
	nav_msgs::Path uwb_path_;

	// uwb position info using lidar_height
	ImuGpsLocalization::UwbDataPtr uwb_data_ptr_ =
		std::make_shared<ImuGpsLocalization::UwbData>();

	geometry_msgs::TwistStamped velocity_filter_;
	geometry_msgs::PoseStamped position_filter_;
    nav_msgs::Odometry fused_odom_;

    // for the attitude from the ahrs_imu
    Eigen::Vector3d attitude_ahrs_;
    tf::Quaternion modified_iq;

    std::unique_ptr<ImuGpsLocalization::ImuGpsLocalizer> imu_gps_localizer_ptr_;
};