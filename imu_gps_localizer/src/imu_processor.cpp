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
    Eigen::Matrix<double, 16, 16> Fx = Eigen::Matrix<double, 16, 16>::Identity();
    Fx.block<3, 3>(0, 3)   = Eigen::Matrix3d::Identity() * delta_t;

    Eigen::Quaterniond curr_ori(state->G_R_I);
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
    Fx.block<3, 4>(3, 6) =   FVq * delta_t;                                                               

    Fx.block<3, 3>(3, 10)   =  state->G_R_I * delta_t;

    Eigen::Vector3d w = gyro_unbias;
    Eigen::Matrix<double,4,4> Fqq = 0.5* (Eigen::Matrix<double,4,4>()<<0,-w.x(),-w.y(),-w.z(),
                                                                        w.x(),0,w.z(),-w.y(),
                                                                        w.y(),-w.z(),0,w.x(),
                                                                        w.z(),w.y(),-w.x(),0).finished();
    Fx.block<4, 4>(6, 6) = Eigen::Matrix4d::Identity() + Fqq * delta_t;

    Eigen::Matrix<double,4,3> Fqw  = 0.5 * (Eigen::Matrix<double,4,3>()<<-q1,-q2,-q3,
                                                                        q0,-q3,q2,
                                                                        q3,q0,-q1,
                                                                        -q2,q1,q0).finished();
    Fx.block<4, 3>(6, 13)  = Fqw * delta_t;

    // Covariance Fi cal.
    Eigen::Matrix<double, 16, 13> Fi = Eigen::Matrix<double, 16, 13>::Zero();
    Fi.block<13, 13>(3, 0) = Eigen::Matrix<double, 13, 13>::Identity();

    Eigen::Matrix<double, 13, 13> Qi = Eigen::Matrix<double, 13, 13>::Zero();
    Qi.block<3, 3>(0, 0) = delta_t2 * acc_noise_ * Eigen::Matrix3d::Identity();
    Qi.block<4, 4>(3, 3) = delta_t2 * gyro_noise_ * Eigen::Matrix4d::Identity();
    Qi.block<3, 3>(7, 7) = delta_t * acc_bias_noise_ * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(10, 10) = delta_t * gyro_bias_noise_ * Eigen::Matrix3d::Identity();

    // state->cov = Fx * last_state.cov * Fx.transpose() + Fi * Qi * Fi.transpose();
    state->cov = Fx * last_state.cov * Fx.transpose();

    // Time and imu.
    state->timestamp = cur_imu->timestamp;
    state->imu_data_ptr = cur_imu;
}

}  // namespace ImuGpsLocalization