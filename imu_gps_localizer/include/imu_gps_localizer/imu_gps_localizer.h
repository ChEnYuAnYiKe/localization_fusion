#pragma once

#include <Eigen/Core>

#include "imu_gps_localizer/base_type.h"
#include "imu_gps_localizer/gps_processor.h"
#include "imu_gps_localizer/imu_processor.h"
#include "imu_gps_localizer/initializer.h"

namespace ImuGpsLocalization {

class ImuGpsLocalizer {
public:
    ImuGpsLocalizer(const double acc_noise, const double gyro_noise,
                    const double acc_bias_noise, const double gyro_bias_noise,
                    const Eigen::Vector3d& I_p_Gps);

    bool ProcessImuData(const ImuDataPtr imu_data_ptr, State* prior_state);

    bool ProcessMagData(const MagDataPtr mag_data_ptr); // added function: to storage mag_data and 
                                                        // cal the initial yaw of the UAV

    bool ProcessGpsPositionData(const GpsPositionDataPtr gps_data_ptr, Eigen::Vector3d* gps_enu, State* fused_state);

    void ProcessFlow();
private:
    std::unique_ptr<Initializer>  initializer_;
    std::unique_ptr<ImuProcessor> imu_processor_;
    std::unique_ptr<GpsProcessor> gps_processor_;

    bool initialized_;
    Eigen::Vector3d init_lla_; // The initial reference gps point.
    State state_;  // The state staged on

    std::deque<ImuDataPtr> imu_buffer_ekf;
    std::deque<GpsPositionDataPtr> gps_buffer_ekf;
};

}  // namespace ImuGpsLocalization