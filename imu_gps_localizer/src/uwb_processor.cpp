#include "imu_gps_localizer/uwb_processor.h"

#include "imu_gps_localizer/utils.h"

namespace ImuGpsLocalization {

UwbProcessor::UwbProcessor(const Eigen::Vector3d& I_p_Uwb) : I_p_Uwb_(I_p_Uwb) { }

void UwbProcessor::UpdateStateByUwbPosition(const Eigen::Vector3d& init_uwb, const UwbDataPtr uwb_data_ptr, State* state) {
    Eigen::Matrix<double, 3, 15> H;
    Eigen::Vector3d residual;
    ComputeJacobianAndResidual(init_uwb, uwb_data_ptr, *state, &H, &residual);
    // EKF.    
    Eigen::Vector3d v(0.0001, 0.0004, 0.16);
    const Eigen::Matrix3d& V = v.asDiagonal();
    const Eigen::MatrixXd& P = state->cov;
    const Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + V).inverse();
    const Eigen::VectorXd delta_x = K * residual;

    // Update state.
    AddDeltaToState(delta_x, state);

    // Covarance.
    const Eigen::MatrixXd I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
    state->cov = I_KH * P * I_KH.transpose() + K * V * K.transpose();
}

void UwbProcessor::ComputeJacobianAndResidual(const Eigen::Vector3d& init_uwb,  
                                              const UwbDataPtr uwb_data, 
                                              const State& state,
                                              Eigen::Matrix<double, 3, 15>* jacobian,
                                              Eigen::Vector3d* residual) {
    const Eigen::Vector3d& G_p_I  = state.G_p_I;
    const Eigen::Matrix3d& G_R_I  = state.G_R_I;

    const Eigen::Vector3d I_p_Uwb = uwb_data->location;
    // Compute residual.
    *residual = I_p_Uwb - (G_p_I + G_R_I * I_p_Uwb_);

    // Compute jacobian.
    jacobian->setZero();
    jacobian->block<3, 3>(0, 0)  = Eigen::Matrix3d::Identity();
    jacobian->block<3, 3>(0, 6)  = - G_R_I * GetSkewMatrix(I_p_Uwb_);
}

void UwbProcessor::AddDeltaToState(const Eigen::Matrix<double, 15, 1>& delta_x, State* state) {
    state->G_p_I     += delta_x.block<3, 1>(0, 0);
    state->G_v_I     += delta_x.block<3, 1>(3, 0);
    state->acc_bias  += delta_x.block<3, 1>(9, 0);
    state->gyro_bias += delta_x.block<3, 1>(12, 0);

    if (delta_x.block<3, 1>(6, 0).norm() > 1e-12) {
        state->G_R_I *= Eigen::AngleAxisd(delta_x.block<3, 1>(6, 0).norm(), delta_x.block<3, 1>(6, 0).normalized()).toRotationMatrix();
    }
}


}  // namespace ImuGpsLocalization