#include "imu_gps_localizer/initializer.h"

#include <Eigen/Dense>
#include <cmath>
#include <glog/logging.h>

#include "imu_gps_localizer/utils.h"

namespace ImuGpsLocalization {

Initializer::Initializer(const Eigen::Vector3d& init_I_p_Gps) 
    : init_I_p_Gps_(init_I_p_Gps) { }

void Initializer::AddImuData(const ImuDataPtr imu_data_ptr) {
    imu_buffer_.push_back(imu_data_ptr);

    if (imu_buffer_.size() > kImuDataBufferLength) {
        imu_buffer_.pop_front();
    }
}

void Initializer::AddMagData(const MagDataPtr mag_data_ptr) {
    mag_buffer_.push_back(mag_data_ptr);

    if (mag_buffer_.size() > kMagDataBufferLength) {
        mag_buffer_.pop_front();
    }
};  // added function: storage the output of the topic /imu/mag[sensor_msgs/MagneticField Message]

bool Initializer::AddGpsPositionData(const GpsPositionDataPtr gps_data_ptr, State* state) {
    if (imu_buffer_.size() < kImuDataBufferLength 
        && mag_buffer_.size() < kMagDataBufferLength) {
        LOG(WARNING) << "[AddGpsPositionData]: No enought imu & mag data!";
        return false;
    }

    const ImuDataPtr last_imu_ptr = imu_buffer_.back();
    // TODO: need to figure out how to synchronize all sensors.
    if (std::abs(gps_data_ptr->timestamp - last_imu_ptr->timestamp) > 0.5) {
        LOG(ERROR) << "[AddGpsPositionData]: Gps and imu timestamps are not synchronized!";
        return false;
    }

    // Set timestamp and imu date.
    state->timestamp = last_imu_ptr->timestamp;
    state->imu_data_ptr = last_imu_ptr;

    // Set initial mean.
    state->G_p_I.setZero();

    // We have no information to set initial velocity. 
    // So, just set it to zero and given big covariance.
    state->G_v_I.setZero();

    // We can use the direction of gravity to set roll and pitch. 
    // But, we cannot set the yaw. 
    // So, we set yaw to zero and give it a big covariance.
    /*
    if (!ComputeG_R_IFromImuData(&state->G_R_I)) {
        LOG(WARNING) << "[AddGpsPositionData]: Failed to compute G_R_I!";
        return false;
    }
    */

    if (!ComputeQuaternionFromImu(&state->G_q)) {
        LOG(WARNING) << "[AddGpsPositionData]: Failed to compute G_q!";
        return false;
    }

    // generate rotation matrix from quaternion
    state->G_R_I = state->G_q.toRotationMatrix();

    // Set bias to zero.
    state->acc_bias.setZero();
    state->gyro_bias.setZero();

    // Set covariance.
    state->cov.setZero();
    // position std: 10 m
    state->cov.block<3, 3>(0, 0) = 100. * Eigen::Matrix3d::Identity();
    // velocity std: 10 m/s
    state->cov.block<3, 3>(3, 3) = 100. * Eigen::Matrix3d::Identity(); 

    /*
    // roll pitch std 10 degree.
    state->cov.block<2, 2>(6, 6) = 10. * kDegreeToRadian * 10. * kDegreeToRadian * Eigen::Matrix2d::Identity();
    // yaw std: 100 degree.
    state->cov(8, 8)             = 100. * kDegreeToRadian * 100. * kDegreeToRadian; 
    // Acc bias.
    state->cov.block<3, 3>(9, 9) = 0.0004 * Eigen::Matrix3d::Identity();
    // Gyro bias.
    state->cov.block<3, 3>(12, 12) = 0.0004 * Eigen::Matrix3d::Identity();
    */

    // Set quaternion 
    state->cov.block<4, 4>(6, 6) = 0.0025 * Eigen::Matrix4d::Identity();
    // Acc bias.
    state->cov.block<3, 3>(10, 10) = 0.0004 * Eigen::Matrix3d::Identity();
    // Gyro bias.
    state->cov.block<3, 3>(13, 13) = 0.0004 * Eigen::Matrix3d::Identity();
   
    return true;
}

bool Initializer::ComputeQuaternionFromImu(Eigen::Quaterniond* G_q) {
    // Compute mean of the imu & mag buffer.
    Eigen::Vector3d sum_acc(0., 0., 0.);
    for (const auto imu_data : imu_buffer_) {
        sum_acc += imu_data->acc;
    }
    const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buffer_.size();

    Eigen::Vector3d sum_err2(0., 0., 0.);
    for (const auto imu_data : imu_buffer_) {
        sum_err2 += (imu_data->acc - mean_acc).cwiseAbs2();
    }
    const Eigen::Vector3d std_acc = (sum_err2 / (double)imu_buffer_.size()).cwiseSqrt();

    if (std_acc.maxCoeff() > kAccStdLimit) {
        LOG(WARNING) << "[ComputeG_R_IFromImuData]: Too big acc std: " << std_acc.transpose();
        return false;
    }

    Eigen::Vector3d sum_mag(0., 0., 0.);
    for (const auto mag_data : mag_buffer_) {
        sum_mag += mag_data->mag_xyz;
    }
    const Eigen::Vector3d mean_mag = sum_mag / (double)mag_buffer_.size();

    // cal the initial theta(roll) and phi(pitch) of the static UAV
    const double theta_roll = asin( - mean_acc.x()/ mean_acc.norm() );
    const double phi_pitch  = asin( - mean_acc.y()/ (cos(theta_roll) * mean_acc.norm()));

    // cal the initial psi(yaw) of the static UAV
    const double psi_yaw = -atan( (mean_mag.y() * cos(phi_pitch) - mean_mag.z() * sin(phi_pitch)) 
                                 / (mean_mag.x() * cos(theta_roll) + mean_mag.y() * sin(theta_roll) * sin(phi_pitch) 
                                  + mean_mag.z() * sin(theta_roll) * cos(phi_pitch)) );

    // transfer r/p/y to quaternion
    Eigen::AngleAxisd rollAngle(theta_roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(phi_pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(psi_yaw, Eigen::Vector3d::UnitZ());
    *G_q = yawAngle * pitchAngle * rollAngle;

};  // added function: use the imu_data & mag_data to cal the quaternion of UAV


bool Initializer::ComputeG_R_IFromImuData(Eigen::Matrix3d* G_R_I) {
    // Compute mean and std of the imu buffer.
    Eigen::Vector3d sum_acc(0., 0., 0.);
    for (const auto imu_data : imu_buffer_) {
        sum_acc += imu_data->acc;
    }
    const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buffer_.size();

    Eigen::Vector3d sum_err2(0., 0., 0.);
    for (const auto imu_data : imu_buffer_) {
        sum_err2 += (imu_data->acc - mean_acc).cwiseAbs2();
    }
    const Eigen::Vector3d std_acc = (sum_err2 / (double)imu_buffer_.size()).cwiseSqrt();

    if (std_acc.maxCoeff() > kAccStdLimit) {
        LOG(WARNING) << "[ComputeG_R_IFromImuData]: Too big acc std: " << std_acc.transpose();
        return false;
    }

    // Compute rotation.
    // Please refer to 
    // https://github.com/rpng/open_vins/blob/master/ov_core/src/init/InertialInitializer.cpp
    
    // Three axises of the ENU frame in the IMU frame.
    // z-axis.
    const Eigen::Vector3d& z_axis = mean_acc.normalized(); 

    // x-axis.
    Eigen::Vector3d x_axis = 
        Eigen::Vector3d::UnitX() - z_axis * z_axis.transpose() * Eigen::Vector3d::UnitX();
    x_axis.normalize();

    // y-axis.
    Eigen::Vector3d y_axis = z_axis.cross(x_axis);
    y_axis.normalize();

    Eigen::Matrix3d I_R_G;
    I_R_G.block<3, 1>(0, 0) = x_axis;
    I_R_G.block<3, 1>(0, 1) = y_axis;
    I_R_G.block<3, 1>(0, 2) = z_axis;

    *G_R_I = I_R_G.transpose();

    return true;
}

}  // namespace ImuGpsLocalization