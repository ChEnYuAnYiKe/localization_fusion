#pragma once

#include "imu_gps_localizer/imu_gps_localizer.h"
#include <fstream>
#include <deque>
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

    // RANSAC核心逻辑的私有成员函数
    void runRansac();

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
    
    // 新增：RANSAC滤波器相关的成员变量
private:
    // 用于存储带有位置信息的LiDAR测量点的结构体
    struct RansacPoint {
        double x, y, z;
    };
    std::deque<RansacPoint> ransac_buffer_;     // 作为滑动窗口的数据缓冲区
    int ransac_buffer_size_;                    // 缓冲区大小
    int ransac_iterations_;                     // RANSAC迭代次数
    double ransac_distance_threshold_;          // 判断内点（inlier）的距离阈值
    double filtered_lidar_z_;                   // 存储滤波后干净的高度值
    bool lidar_initialized_ = false;            // 标记LiDAR滤波器是否已初始化
};