#include "imu_gps_localizer/imu_gps_localizer.h"

#include <glog/logging.h>

#include "imu_gps_localizer/utils.h"

#include <iostream>

namespace ImuGpsLocalization {

ImuGpsLocalizer::ImuGpsLocalizer(const double acc_noise, const double gyro_noise,
                                 const double acc_bias_noise, const double gyro_bias_noise,
                                 const Eigen::Vector3d& I_p_Gps) 
    : initialized_(false){
    initializer_ = std::make_unique<Initializer>(I_p_Gps);
    imu_processor_ = std::make_unique<ImuProcessor>(acc_noise, gyro_noise, 
                                                    acc_bias_noise, gyro_bias_noise,
                                                    Eigen::Vector3d(0., 0., -9.80665));
    gps_processor_ = std::make_unique<GpsProcessor>(I_p_Gps);
}

bool ImuGpsLocalizer::ProcessImuData(const ImuDataPtr imu_data_ptr) {
    if (!initialized_) {
        initializer_->AddImuData(imu_data_ptr);
        return false;
    }
    
    imu_buffer_ekf.push_back(imu_data_ptr);

    // Predict.
    // imu_processor_->Predict(state_.imu_data_ptr, imu_data_ptr, &state_);

    // Convert ENU state to lla.
    // ConvertENUToLLA(init_lla_, state_.G_p_I, &(state_.lla));
    
    return true;
}

bool ImuGpsLocalizer::ProcessMagData(const MagDataPtr mag_data_ptr) {
    if (!initialized_) {
        initializer_->AddMagData(mag_data_ptr);
        return false;
    }
    return true;
}

bool ImuGpsLocalizer::ProcessGpsPositionData(const GpsPositionDataPtr gps_data_ptr, Eigen::Vector3d* gps_enu) {
    if (!initialized_) {
        if (!initializer_->AddGpsPositionData(gps_data_ptr, &state_)) {
            return false;
        }

        // Initialize the initial gps point used to convert lla to ENU.
        init_lla_ = gps_data_ptr->lla;
        
        initialized_ = true;

        LOG(INFO) << "[ProcessGpsPositionData]: System initialized!";
        return true;
    }

    gps_buffer_ekf.push_back(gps_data_ptr);

    ConvertLLAToENU(init_lla_, gps_data_ptr->lla, gps_enu);

    return true;
}

bool ImuGpsLocalizer::ProcessFlow(std::queue<State>* fused_state) {

    while (!imu_buffer_ekf.empty() && !gps_buffer_ekf.empty()) {
        ImuDataPtr curr_imu_data_;
        GpsPositionDataPtr curr_gps_data_;

        curr_imu_data_ = imu_buffer_ekf.front();
        curr_gps_data_ = gps_buffer_ekf.front();
    
        if (curr_imu_data_->timestamp < curr_gps_data_->timestamp){ // IMU数据比当前GPS数据时间早，则使用IMU预测
            imu_processor_->Predict(state_.imu_data_ptr, curr_imu_data_, &state_); // IMU预测
            ConvertENUToLLA(init_lla_, state_.G_p_I, &(state_.lla));
            imu_buffer_ekf.pop_front();
            // LOG(INFO) << "[ProcessFlow]: processing imu_data!";
     
        } else{ // IMU数据时间比GPS晚一点
            gps_processor_->CorrectStateByGpsPosition(init_lla_, curr_gps_data_, &state_); // GPS数据观测
            gps_buffer_ekf.pop_front();
            fused_state->push(state_);
            // LOG(INFO) << "[ProcessFlow]: processing gps_data!";
        }
    } 
    // std::cout << fused_state->size() << std::endl;
    return true;
}

bool ImuGpsLocalizer::ProcessFlowWithoutGps(std::queue<ImuGpsLocalization::State>* fused_state) {
    while (!imu_buffer_ekf.empty()) {
        ImuDataPtr curr_imu_data_;
        curr_imu_data_ = imu_buffer_ekf.front();

        imu_processor_->PredictWithoutGps(state_.imu_data_ptr, curr_imu_data_, &state_); // IMU预测
        ConvertENUToLLA(init_lla_, state_.G_p_I, &(state_.lla));
        imu_buffer_ekf.pop_front();
        fused_state->push(state_);
    }
    while (!gps_buffer_ekf.empty()) gps_buffer_ekf.pop_front();
    return true;
}

}  // namespace ImuGpsLocalization