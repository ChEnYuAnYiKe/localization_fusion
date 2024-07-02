#pragma once

#include "imu_gps_localizer/base_type.h"
#include <Eigen/Dense>

namespace ImuGpsLocalization {

class GpsProcessor {
public:
	GpsProcessor(const Eigen::Vector3d& I_p_Gps);

	void UpdateStateByGpsPosition(const Eigen::Vector3d& init_lla,
	                              const GpsPositionDataPtr gps_data_ptr,
	                              State* state);

private:
	void ComputeJacobianAndResidual(const Eigen::Vector3d& init_lla,
	                                const GpsPositionDataPtr gps_data,
	                                const State& state,
	                                Eigen::Matrix<double, 3, 15>* jacobian,
	                                Eigen::Vector3d* residual);

	void AddDeltaToState(const Eigen::Matrix<double, 15, 1>& delta_x,
	                     State* state);

	const Eigen::Vector3d I_p_Gps_;
};

} // namespace ImuGpsLocalization