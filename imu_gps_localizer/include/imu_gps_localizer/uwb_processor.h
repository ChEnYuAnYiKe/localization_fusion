#pragma once 

#include <Eigen/Dense>

#include "imu_gps_localizer/base_type.h"

namespace ImuGpsLocalization {

class UwbProcessor {
public:
    UwbProcessor(const Eigen::Vector3d& I_p_Uwb);

    void UpdateStateByUwbPosition(const Eigen::Vector3d& init_uwb, const UwbDataPtr uwb_data_ptr, State* state);

private:    
    void ComputeJacobianAndResidual(const Eigen::Vector3d& init_uwb,  
                                    const UwbDataPtr uwb_data, 
                                    const State& state,
                                    Eigen::Matrix<double, 3, 15>* jacobian,
                                    Eigen::Vector3d* residual);

    void AddDeltaToState(const Eigen::Matrix<double, 15, 1>& delta_x, State* state);

    const Eigen::Vector3d I_p_Uwb_;  
};



}  // namespace ImuGpsLocalization