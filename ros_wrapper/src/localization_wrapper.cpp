#include "localization_wrapper.h"

#include <iomanip>

#include <glog/logging.h>

#include "imu_gps_localizer/base_type.h"

LocalizationWrapper::LocalizationWrapper(ros::NodeHandle& nh) {
    // Load configs.
    double acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise;
    nh.param("acc_noise",       acc_noise, 1e-2);
    nh.param("gyro_noise",      gyro_noise, 1e-4);
    nh.param("acc_bias_noise",  acc_bias_noise, 1e-6);
    nh.param("gyro_bias_noise", gyro_bias_noise, 1e-8);

    double x, y, z;
    nh.param("I_p_Gps_x", x, 0.);
    nh.param("I_p_Gps_y", y, 0.);
    nh.param("I_p_Gps_z", z, 0.);
    const Eigen::Vector3d I_p_Gps(x, y, z);

    std::string log_folder = "/home";
    ros::param::get("log_folder", log_folder);

    // Log.
    file_state_.open(log_folder + "/state.csv");
    file_gps_.open(log_folder +"/gps.csv");

    // Initialization imu gps localizer.
    imu_gps_localizer_ptr_ = 
        std::make_unique<ImuGpsLocalization::ImuGpsLocalizer>(acc_noise, gyro_noise,
                                                              acc_bias_noise, gyro_bias_noise,
                                                              I_p_Gps);

    // Subscribe topics.
    imu_sub_ = nh.subscribe("/imu/data", 10,  &LocalizationWrapper::ImuCallback, this);
    mag_sub_ = nh.subscribe("/imu/mag", 10, &LocalizationWrapper::MagCallBack, this);
    gps_position_sub_ = nh.subscribe("/fix", 10,  &LocalizationWrapper::GpsPositionCallback, this);

    state_pub_ = nh.advertise<nav_msgs::Path>("fused_path", 10);
    imu_pub_ = nh.advertise<nav_msgs::Path>("imu_path", 10);  // added publisher
    gps_pub_ = nh.advertise<nav_msgs::Path>("gps_path", 10);  // added publisher
}

LocalizationWrapper::~LocalizationWrapper() {
    file_state_.close();
    file_gps_.close();
}

void LocalizationWrapper::ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
    ImuGpsLocalization::ImuDataPtr imu_data_ptr = std::make_shared<ImuGpsLocalization::ImuData>();
    imu_data_ptr->timestamp = imu_msg_ptr->header.stamp.toSec();
    imu_data_ptr->acc << imu_msg_ptr->linear_acceleration.x, 
                         imu_msg_ptr->linear_acceleration.y,
                         imu_msg_ptr->linear_acceleration.z;
    imu_data_ptr->gyro << imu_msg_ptr->angular_velocity.x,
                          imu_msg_ptr->angular_velocity.y,
                          imu_msg_ptr->angular_velocity.z;
    
    ImuGpsLocalization::State prior_state;
    
    const bool okk = imu_gps_localizer_ptr_->ProcessImuData(imu_data_ptr, &prior_state);
    if (!okk) {
        return;
    }
    ConvertP_StateToRosTopic(prior_state);
    imu_pub_.publish(imu_path_);

}

void LocalizationWrapper::MagCallBack(const sensor_msgs::MagneticFieldConstPtr& mag_msg_ptr) {
    ImuGpsLocalization::MagDataPtr mag_data_ptr = std::make_shared<ImuGpsLocalization::MagData>();
    mag_data_ptr->timestamp = mag_msg_ptr->header.stamp.toSec();
    mag_data_ptr->mag_xyz << mag_msg_ptr->magnetic_field.x,
                             mag_msg_ptr->magnetic_field.y,
                             mag_msg_ptr->magnetic_field.z;
    mag_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(mag_msg_ptr->magnetic_field_covariance.data());

    imu_gps_localizer_ptr_->ProcessMagData(mag_data_ptr);

};  // added function: to accept the mag_msg and process it

void LocalizationWrapper::GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr) {
    // Check the gps_status.
    if (gps_msg_ptr->status.status != 2) {
        LOG(WARNING) << "[GpsCallBack]: Bad gps message!";
        return;
    }

    ImuGpsLocalization::GpsPositionDataPtr gps_data_ptr = std::make_shared<ImuGpsLocalization::GpsPositionData>();
    gps_data_ptr->timestamp = gps_msg_ptr->header.stamp.toSec();
    gps_data_ptr->lla << gps_msg_ptr->latitude,
                         gps_msg_ptr->longitude,
                         gps_msg_ptr->altitude;
    gps_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gps_msg_ptr->position_covariance.data());

    ImuGpsLocalization::State fused_state;
    Eigen::Vector3d gps_enu;

    const bool ok = imu_gps_localizer_ptr_->ProcessGpsPositionData(gps_data_ptr, &gps_enu, &fused_state);
    if (!ok) {
        return;
    }

    // Publish the accepted Gps data
    ConvertGps_enuToRosTopic(gps_enu);
    gps_pub_.publish(gps_path_);

    // Publish fused state.
    ConvertStateToRosTopic(fused_state);
    state_pub_.publish(ros_path_);

    // Log fused state.
    LogState(fused_state);
    LogGps(gps_data_ptr, gps_enu);
}

void LocalizationWrapper::LogState(const ImuGpsLocalization::State& state) {
    // const Eigen::Quaterniond G_q_I(state.G_R_I);
    file_state_ << std::fixed << std::setprecision(15)
                << state.timestamp << ","
                << state.lla[0] << "," << state.lla[1] << "," << state.lla[2] << ","
                << state.G_p_I[0] << "," << state.G_p_I[1] << "," << state.G_p_I[2] << ","
                << state.G_v_I[0] << "," << state.G_v_I[1] << "," << state.G_v_I[2] << ","
                << state.G_q.x() << "," << state.G_q.y() << "," << state.G_q.z() << "," << state.G_q.w() << ","
                << state.acc_bias[0] << "," << state.acc_bias[1] << "," << state.acc_bias[2] << ","
                << state.gyro_bias[0] << "," << state.gyro_bias[1] << "," << state.gyro_bias[2] << "\n";
}

void LocalizationWrapper::LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data, Eigen::Vector3d gps_enu) {
    file_gps_ << std::fixed << std::setprecision(15)
              << gps_data->timestamp << ","
              << gps_data->lla[0] << "," << gps_data->lla[1] << "," << gps_data->lla[2] << ","
              << gps_enu[0] << "," << gps_enu[1] << "," << gps_enu[2] << "\n";
} // changed! add the gps_enu data

void LocalizationWrapper::ConvertGps_enuToRosTopic(const Eigen::Vector3d& gps_enu) {
    gps_path_.header.frame_id = "world";
    gps_path_.header.stamp = ros::Time::now();  

    geometry_msgs::PoseStamped pose;
    pose.header = gps_path_.header;

    pose.pose.position.x = gps_enu[0];
    pose.pose.position.y = gps_enu[1];
    pose.pose.position.z = gps_enu[2];

    gps_path_.poses.push_back(pose);
}  // added function

void LocalizationWrapper::ConvertP_StateToRosTopic(const ImuGpsLocalization::State& state) {
    imu_path_.header.frame_id = "world";
    imu_path_.header.stamp = ros::Time::now();  

    geometry_msgs::PoseStamped pose;
    pose.header = imu_path_.header;

    pose.pose.position.x = state.G_p_I[0];
    pose.pose.position.y = state.G_p_I[1];
    pose.pose.position.z = state.G_p_I[2];

    // const Eigen::Quaterniond G_q_I(state.G_R_I);
    pose.pose.orientation.x = state.G_q.x();
    pose.pose.orientation.y = state.G_q.y();
    pose.pose.orientation.z = state.G_q.z();
    pose.pose.orientation.w = state.G_q.w();

    imu_path_.poses.push_back(pose);
}  // added function

void LocalizationWrapper::ConvertStateToRosTopic(const ImuGpsLocalization::State& state) {
    ros_path_.header.frame_id = "world";
    ros_path_.header.stamp = ros::Time::now();  

    geometry_msgs::PoseStamped pose;
    pose.header = ros_path_.header;

    pose.pose.position.x = state.G_p_I[0];
    pose.pose.position.y = state.G_p_I[1];
    pose.pose.position.z = state.G_p_I[2];

    // const Eigen::Quaterniond G_q_I(state.G_R_I);
    pose.pose.orientation.x = state.G_q.x();
    pose.pose.orientation.y = state.G_q.y();
    pose.pose.orientation.z = state.G_q.z();
    pose.pose.orientation.w = state.G_q.w();

    ros_path_.poses.push_back(pose);
}