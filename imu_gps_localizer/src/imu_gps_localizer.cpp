#include "imu_gps_localizer/imu_gps_localizer.h"
#include "imu_gps_localizer/utils.h"

namespace ImuGpsLocalization {

ImuGpsLocalizer::ImuGpsLocalizer(const double acc_noise,
                                 const double gyro_noise,
                                 const double acc_bias_noise,
                                 const double gyro_bias_noise,
                                 const Eigen::Vector3d& I_p_Gps,
                                 const Eigen::Vector3d& I_p_Uwb)
    : initialized_(false) {
	initializer_ = std::make_unique<Initializer>(I_p_Gps);
	imu_processor_ = std::make_unique<ImuProcessor>(
	    acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise,
	    Eigen::Vector3d(0., 0., -9.81));
	gps_processor_ = std::make_unique<GpsProcessor>(I_p_Gps);
	uwb_processor_ = std::make_unique<UwbProcessor>(I_p_Uwb);
}

bool ImuGpsLocalizer::ProcessImuData(const ImuDataPtr imu_data_ptr,
                                     State* fused_state) {
	if (!initialized_) {
		initializer_->AddImuData(imu_data_ptr);
		return false;
	}

	// Predict.
	imu_processor_->Predict(state_.imu_data_ptr, imu_data_ptr, &state_);

	// Convert ENU state to lla.
	ConvertENUToLLA(init_lla_, state_.G_p_I, &(state_.lla));
	*fused_state = state_;
	return true;
}

bool ImuGpsLocalizer::ProcessGpsPositionData(
    const GpsPositionDataPtr gps_data_ptr, Eigen::Vector3d* gps_enu) {
	if (!initialized_) {
		if (!initializer_->AddGpsPositionData(gps_data_ptr, &state_)) {
			return false;
		}

		// Initialize the initial gps point used to convert lla to ENU.
		init_lla_ = gps_data_ptr->lla;

		initialized_ = true;

		ROS_INFO_STREAM("[ProcessGpsPositionData]: System initialized!");
		return true;
	}

	ConvertLLAToENU(init_lla_, gps_data_ptr->lla, gps_enu);

	// Update.
	gps_processor_->UpdateStateByGpsPosition(init_lla_, gps_data_ptr, &state_);

	return true;
}

bool ImuGpsLocalizer::ProcessUwbData(const UwbDataPtr uwb_data_ptr) {
	if (!initialized_) {
		if (!initializer_->AddUwbData(uwb_data_ptr, &state_)) {
			return false;
		}

		init_uwb_ = uwb_data_ptr->location;

		initialized_ = true;

		ROS_INFO_STREAM("[ProcessUwbData]: System initialized!");
		return true;
	}
	// 排除异常点

	// Update.
	uwb_processor_->UpdateStateByUwbPosition(init_uwb_, uwb_data_ptr, &state_);

	return true;
}

} // namespace ImuGpsLocalization