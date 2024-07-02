#include "localization_wrapper.h"
#include "imu_gps_localizer/base_type.h"
#include <glog/logging.h>
#include <iomanip>

LocalizationWrapper::LocalizationWrapper(ros::NodeHandle& nh) {
	// Load configs.
	double acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise;
	nh.param("acc_noise", acc_noise, 1e-1);
	nh.param("gyro_noise", gyro_noise, 1e-1);
	nh.param("acc_bias_noise", acc_bias_noise, 1e-4);
	nh.param("gyro_bias_noise", gyro_bias_noise, 1e-4);

	double x, y, z;
	nh.param("I_p_Gps_x", x, 0.);
	nh.param("I_p_Gps_y", y, 0.);
	nh.param("I_p_Gps_z", z, 0.);
	const Eigen::Vector3d I_p_Gps(x, y, z);

	double x_u, y_u, z_u;
	nh.param("I_p_Uwb_x", x_u, 0.);
	nh.param("I_p_Uwb_y", y_u, 0.);
	nh.param("I_p_Uwb_z", z_u, 0.);
	const Eigen::Vector3d I_p_Uwb(x_u, y_u, z_u);

	std::string log_folder = "/home";
	ros::param::get("log_folder", log_folder);

	// Log.
	file_state_.open(log_folder + "/state.csv");
	// file_gps_.open(log_folder +"/gps.csv");
	file_uwb_.open(log_folder + "/uwb.csv");

	// Initialization imu gps localizer.
	imu_gps_localizer_ptr_ =
	    std::make_unique<ImuGpsLocalization::ImuGpsLocalizer>(
	        acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise, I_p_Gps,
	        I_p_Uwb);

	// Subscribe topics.  mavros中的imu话题为/mavros/imu/data
	imu_sub_ = nh.subscribe("/mavros/imu/data_raw", 100,
	                        &LocalizationWrapper::ImuCallback, this);
	// gps_position_sub_ = nh.subscribe("/fix", 50,
	// &LocalizationWrapper::GpsPositionCallback, this);
	uwb_sub_ =
	    nh.subscribe("/uwb/data", 100, &LocalizationWrapper::UwbCallback, this);
	lidar_sub_ = nh.subscribe("/lidar_position", 100,
	                          &LocalizationWrapper::LidarCallback, this);

	// state_pub_ = nh.advertise<nav_msgs::Path>("/fused_path", 100);
	// gps_pub_ = nh.advertise<nav_msgs::Path>("/gps_path", 50);
	// uwb_pub_ = nh.advertise<nav_msgs::Path>("/uwb_path", 100);

	velocity_filter_pub_ =
	    nh.advertise<geometry_msgs::TwistStamped>("/velocity_filter", 100);
	position_filter_pub_ =
	    nh.advertise<geometry_msgs::PoseStamped>("/position_filter", 100);
}

LocalizationWrapper::~LocalizationWrapper() {
	file_state_.close();
	// file_gps_.close();
}

void LocalizationWrapper::ImuCallback(
    const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
	ImuGpsLocalization::ImuDataPtr imu_data_ptr =
	    std::make_shared<ImuGpsLocalization::ImuData>();
	imu_data_ptr->timestamp = imu_msg_ptr->header.stamp.toSec();
	imu_data_ptr->acc << imu_msg_ptr->linear_acceleration.x / 1000.,
	    imu_msg_ptr->linear_acceleration.y / 1000.,
	    imu_msg_ptr->linear_acceleration.z / 1000.;
	imu_data_ptr->gyro << imu_msg_ptr->angular_velocity.x,
	    imu_msg_ptr->angular_velocity.y, imu_msg_ptr->angular_velocity.z;

	ImuGpsLocalization::State fused_state;
	const bool ok =
	    imu_gps_localizer_ptr_->ProcessImuData(imu_data_ptr, &fused_state);
	if (!ok) {
		return;
	}

	// Publish fused state.
	ConvertStateToRosTopic(fused_state);
	state_pub_.publish(ros_path_);
	velocity_filter_pub_.publish(velocity_filter_);
	position_filter_pub_.publish(position_filter_);

	// Log fused state.
	LogState(fused_state);
}

// void LocalizationWrapper::GpsPositionCallback(const
// sensor_msgs::NavSatFixConstPtr& gps_msg_ptr) {
//     // Check the gps_status.
//     if (gps_msg_ptr->status.status != 2) {
//         LOG(WARNING) << "[GpsCallBack]: Bad gps message!";
//         return;
//     }

//     ImuGpsLocalization::GpsPositionDataPtr gps_data_ptr =
//     std::make_shared<ImuGpsLocalization::GpsPositionData>();
//     gps_data_ptr->timestamp = gps_msg_ptr->header.stamp.toSec();
//     gps_data_ptr->lla << gps_msg_ptr->latitude,
//                          gps_msg_ptr->longitude,
//                          gps_msg_ptr->altitude;
//     gps_data_ptr->cov = Eigen::Map<const
//     Eigen::Matrix3d>(gps_msg_ptr->position_covariance.data());

//     Eigen::Vector3d gps_enu;
//     imu_gps_localizer_ptr_->ProcessGpsPositionData(gps_data_ptr, &gps_enu);

//     // Publish Gps state.
//     ConvertGps_enuToRosTopic(gps_enu);
//     gps_pub_.publish(gps_path_);

//     //LogGps(gps_data_ptr, gps_enu);
// }

void LocalizationWrapper::UwbCallback(
    const imu_gps_localization::uwbConstPtr& uwb_msg_ptr) {
	// ImuGpsLocalization::UwbDataPtr uwb_data_ptr =
	// std::make_shared<ImuGpsLocalization::UwbData>();
	uwb_data_ptr_->timestamp = uwb_msg_ptr->time.toSec();
	// uwb_data_ptr->location << uwb_msg_ptr->x, uwb_msg_ptr->y, uwb_msg_ptr->z;
	uwb_data_ptr_->location[0] = uwb_msg_ptr->x;
	uwb_data_ptr_->location[1] = uwb_msg_ptr->y;

	imu_gps_localizer_ptr_->ProcessUwbData(uwb_data_ptr_);

	ConvertUwbToRosTopic(uwb_data_ptr_);
	uwb_pub_.publish(uwb_path_);
}

void LocalizationWrapper::LidarCallback(
    const geometry_msgs::PoseStamped& lidar_msg_ptr) {
	uwb_data_ptr_->location[2] = lidar_msg_ptr.pose.position.z;
}

void LocalizationWrapper::LogState(const ImuGpsLocalization::State& state) {
	// const Eigen::Quaterniond G_q_I(state.G_R_I);
	Eigen::Vector3d eulerAngle = state.G_R_I.eulerAngles(2, 1, 0);

	file_state_
	    << std::fixed << std::setprecision(15) << state.timestamp
	    << ","
	    // << state.lla[0] << "," << state.lla[1] << "," << state.lla[2] << ","
	    << state.G_p_I[0] << "," << state.G_p_I[1] << "," << state.G_p_I[2]
	    << "," << state.G_v_I[0] << "," << state.G_v_I[1] << ","
	    << state.G_v_I[2] << "," << eulerAngle[0] << "," << eulerAngle[1] << ","
	    << eulerAngle[2] << "," << state.acc_bias[0] << "," << state.acc_bias[1]
	    << "," << state.acc_bias[2] << "," << state.gyro_bias[0] << ","
	    << state.gyro_bias[1] << "," << state.gyro_bias[2] << "\n";
}

// void LocalizationWrapper::LogGps(const ImuGpsLocalization::GpsPositionDataPtr
// gps_data, Eigen::Vector3d gps_enu) {
//     file_gps_ << std::fixed << std::setprecision(15)
//               << gps_data->timestamp << ","
//               << gps_data->lla[0] << "," << gps_data->lla[1] << "," <<
//               gps_data->lla[2] << ","
//               << gps_enu[0] << "," << gps_enu[1] << "," << gps_enu[2] <<
//               "\n";
// }

void LocalizationWrapper::ConvertStateToRosTopic(
    const ImuGpsLocalization::State& state) {
	ros_path_.header.frame_id = "global";
	ros_path_.header.stamp = ros::Time::now();

	geometry_msgs::PoseStamped pose;
	pose.header = ros_path_.header;

	pose.pose.position.x = state.G_p_I[0];
	pose.pose.position.y = state.G_p_I[1];
	pose.pose.position.z = state.G_p_I[2];

	const Eigen::Quaterniond G_q_I(state.G_R_I);
	pose.pose.orientation.x = G_q_I.x();
	pose.pose.orientation.y = G_q_I.y();
	pose.pose.orientation.z = G_q_I.z();
	pose.pose.orientation.w = G_q_I.w();
	ros_path_.poses.push_back(pose);

	velocity_filter_.header.frame_id = "global";
	velocity_filter_.header.stamp = ros::Time::now();
	velocity_filter_.twist.linear.x = state.G_v_I[0];
	velocity_filter_.twist.linear.y = state.G_v_I[1];
	velocity_filter_.twist.linear.z = state.G_v_I[2];

	position_filter_ = pose;
}

// void LocalizationWrapper::ConvertGps_enuToRosTopic(const Eigen::Vector3d&
// gps_enu) {
//     gps_path_.header.frame_id = "global";
//     gps_path_.header.stamp = ros::Time::now();

//     geometry_msgs::PoseStamped pose;
//     pose.header = gps_path_.header;

//     pose.pose.position.x = gps_enu[0];
//     pose.pose.position.y = gps_enu[1];
//     pose.pose.position.z = gps_enu[2];

//     gps_path_.poses.push_back(pose);
// }

void LocalizationWrapper::ConvertUwbToRosTopic(
    const ImuGpsLocalization::UwbDataPtr& uwb_data) {
	uwb_path_.header.frame_id = "global";
	uwb_path_.header.stamp = ros::Time::now();

	geometry_msgs::PoseStamped pose;
	pose.header = uwb_path_.header;

	pose.pose.position.x = uwb_data->location[0];
	pose.pose.position.y = uwb_data->location[1];
	pose.pose.position.z = uwb_data->location[2];

	uwb_path_.poses.push_back(pose);
}
