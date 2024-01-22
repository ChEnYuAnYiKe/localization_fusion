#include "imu_gps_localizer/imu_processor.h"

#include <glog/logging.h>
#include <Eigen/Dense>

#include "imu_gps_localizer/utils.h"

namespace ImuGpsLocalization {

ImuProcessor::ImuProcessor(const double acc_noise, const double gyro_noise,
                           const double acc_bias_noise, const double gyro_bias_noise,
                           const Eigen::Vector3d& gravity)
    : acc_noise_(acc_noise), gyro_noise_(gyro_noise), 
      acc_bias_noise_(acc_bias_noise), gyro_bias_noise_(gyro_bias_noise),
      gravity_(gravity) { }

void ImuProcessor::Predict(const ImuDataPtr last_imu, const ImuDataPtr cur_imu, State* state) {
    // Time.
    const double delta_t = cur_imu->timestamp - last_imu->timestamp;
    const double delta_t2 = delta_t * delta_t;

    // Set last state.
    State last_state = *state;
    // Acc and gyro.
    const Eigen::Vector3d acc_unbias = 0.5 * (last_imu->acc + cur_imu->acc) - last_state.acc_bias;
    const Eigen::Vector3d gyro_unbias = 0.5 * (last_imu->gyro + cur_imu->gyro) - last_state.gyro_bias;

    state->G_p_I = last_state.G_p_I + last_state.G_v_I * delta_t + 
                   0.5 * (last_state.G_R_I * acc_unbias + gravity_) * delta_t2;
    state->G_v_I = last_state.G_v_I + (last_state.G_R_I * acc_unbias + gravity_) * delta_t;

    const Eigen::Vector3d delta_angle_axis = gyro_unbias * delta_t;
    state->G_R_I = last_state.G_R_I * Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix();

    // Covariance Fx cal.
    Eigen::Matrix<double, 16, 16> Fx_ = Eigen::Matrix<double, 16, 16>::Zero();
    Fx_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    Eigen::Quaterniond curr_ori(last_state.G_R_I);
    double q0 = curr_ori.w();
    double q1 = curr_ori.x();
    double q2 = curr_ori.y();
    double q3 = curr_ori.z();

    double FVq0 = 2 * Eigen::Vector3d(q0,-q3,q2).transpose()*acc_unbias;
    double FVq1 = 2 * Eigen::Vector3d(q1,q2,q3).transpose()*acc_unbias;
    double FVq2 = 2 * Eigen::Vector3d(-q2,q1,q0).transpose()*acc_unbias;
    double FVq3 = 2 * Eigen::Vector3d(-q3,-q0,q1).transpose()*acc_unbias;
    Eigen::Matrix<double,3,4> FVq = (Eigen::Matrix<double,3,4>()<<  FVq0,FVq1,FVq2,FVq3,
                                                                    -FVq3,-FVq2,FVq1,FVq0,
                                                                    FVq2,-FVq3,-FVq0,FVq1).finished();
    Fx_.block<3, 4>(3, 6) =  FVq;                                                               

    Fx_.block<3, 3>(3, 10) = last_state.G_R_I;

    Eigen::Vector3d w = gyro_unbias;
    Eigen::Matrix<double,4,4> Fqq = 0.5* (Eigen::Matrix<double,4,4>()<<0,-w.x(),-w.y(),-w.z(),
                                                                        w.x(),0,w.z(),-w.y(),
                                                                        w.y(),-w.z(),0,w.x(),
                                                                        w.z(),w.y(),-w.x(),0).finished();
    Fx_.block<4, 4>(6, 6) = Fqq;

    Eigen::Matrix<double,4,3> Fqw  = 0.5 * (Eigen::Matrix<double,4,3>()<<-q1,-q2,-q3,
                                                                        q0,-q3,q2,
                                                                        q3,q0,-q1,
                                                                        -q2,q1,q0).finished();
    Fx_.block<4, 3>(6, 13)  = Fqw;

    Eigen::Matrix<double, 16, 16> Fx = Eigen::Matrix<double, 16, 16>::Identity() + Fx_ * delta_t;

    // Fi cal.
    Eigen::Matrix<double, 16, 6> Fi_ = Eigen::Matrix<double, 16, 6>::Zero();
    Fi_.block<3,3>(0, 3) = last_state.G_R_I;
    Fi_.block<4,3>(6, 3) = Fqw;
    Eigen::Matrix<double, 16, 6> Fi = Fi_ * delta_t;

    // Covariance Qi cal.
    Eigen::Matrix<double, 6, 6> Qi = Eigen::Matrix<double, 6, 6>::Zero();
    Qi.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * acc_noise_ * acc_noise_;
    Qi.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * gyro_noise_ * gyro_noise_;

    // Update cov
    state->cov = Fx * last_state.cov * Fx.transpose() + Fi * Qi * Fi.transpose();

    // Time and imu.
    state->timestamp = cur_imu->timestamp;
    state->imu_data_ptr = cur_imu;
}

}  // namespace ImuGpsLocalization