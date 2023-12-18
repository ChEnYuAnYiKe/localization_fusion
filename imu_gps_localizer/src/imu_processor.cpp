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

    // Obtain acc and gyro without bias.
    const Eigen::Vector3d acc_unbias = 0.5 * (last_imu->acc + cur_imu->acc) - last_state.acc_bias;
    const Eigen::Vector3d gyro_unbias = 0.5 * (last_imu->gyro + cur_imu->gyro) - last_state.gyro_bias;

    // ****************************************************************************************************
    // STEP 1: Calculate the estimation of the prior state
    state->G_p_I = last_state.G_p_I + last_state.G_v_I * delta_t + 
                   0.5 * (last_state.G_R_I * acc_unbias + gravity_) * delta_t2;
    state->G_v_I = last_state.G_v_I + (last_state.G_R_I * acc_unbias + gravity_) * delta_t;

    /*
    const Eigen::Vector3d delta_angle_axis = gyro_unbias * delta_t;
    if (delta_angle_axis.norm() > 1e-12) {
        state->G_R_I = last_state.G_R_I * Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix();
    } 
    */

    // added: to update the quaternion
    Eigen::Quaterniond delta_q;
    const Eigen::Vector3d delta_angle = gyro_unbias * delta_t * 0.5;
    if (delta_angle.norm() > 1e-12) {
        delta_q.w() = cos(delta_angle.norm());
        delta_q.vec() = sin(delta_angle.norm()) * delta_angle.normalized();
        state->G_q = (last_state.G_q * delta_q).normalized();
    }

    //  Combine all state vectors into a single vector
    state->state_vector << state->G_p_I, state->G_v_I, 
                            state->G_q.w(), state->G_q.x(), state->G_q.y(), state->G_q.z(),
                            state->acc_bias, state->gyro_bias;
    
    // Update the rotation matrix
    state->G_R_I = state->G_q.toRotationMatrix();

    // ****************************************************************************************************

    /*
    // Covariance of the error-state.   
    Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity();
    Fx.block<3, 3>(0, 3)   = Eigen::Matrix3d::Identity() * delta_t;
    Fx.block<3, 3>(3, 6)   = - state->G_R_I * GetSkewMatrix(acc_unbias) * delta_t;
    Fx.block<3, 3>(3, 9)   = - state->G_R_I * delta_t;
    if (delta_angle_axis.norm() > 1e-12) {
        Fx.block<3, 3>(6, 6) = Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix().transpose();
    } else {
        Fx.block<3, 3>(6, 6).setIdentity();
    }
    Fx.block<3, 3>(6, 12)  = - Eigen::Matrix3d::Identity() * delta_t;

    Eigen::Matrix<double, 15, 12> Fi = Eigen::Matrix<double, 15, 12>::Zero();
    Fi.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();

    Eigen::Matrix<double, 12, 12> Qi = Eigen::Matrix<double, 12, 12>::Zero();
    Qi.block<3, 3>(0, 0) = delta_t2 * acc_noise_ * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(3, 3) = delta_t2 * gyro_noise_ * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(6, 6) = delta_t * acc_bias_noise_ * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(9, 9) = delta_t * gyro_bias_noise_ * Eigen::Matrix3d::Identity();
    */

    // ****************************************************************************************************
    // STEP 2: Calculate the Jacobian matrix
    // changed! beacuse of the 16-dim state-vector
    F_.setZero();  // initializing Jacobian matrix as a zero matrix
    F_.block<3,3>(INDEX_STATE_POSI, INDEX_STATE_VEL) = Eigen::Matrix3d::Identity(); 

    double q0 = last_state.G_q.w();
    double q1 = last_state.G_q.x();
    double q2 = last_state.G_q.y();
    double q3 = last_state.G_q.z();

    double FVq0 = 2 * Eigen::Vector3d(q0,-q3,q2).transpose() * acc_unbias;
    double FVq1 = 2 * Eigen::Vector3d(q1,q2,q3).transpose() * acc_unbias;
    double FVq2 = 2 * Eigen::Vector3d(-q2,q1,q0).transpose() * acc_unbias;
    double FVq3 = 2 * Eigen::Vector3d(-q3,-q0,q1).transpose() * acc_unbias;
    Eigen::Matrix<double,3,4> FVq = (Eigen::Matrix<double,3,4>()<<  FVq0,FVq1,FVq2,FVq3,
                                                                    -FVq3,-FVq2,FVq1,FVq0,
                                                                    FVq2,-FVq3,-FVq0,FVq1).finished();
    F_.block<3,4>(INDEX_STATE_VEL, INDEX_STATE_ORI) = FVq;

    F_.block<3,3>(INDEX_STATE_VEL, INDEX_STATE_ACC_BIAS) = last_state.G_R_I;

    Eigen::Vector3d w = gyro_unbias;
    Eigen::Matrix<double,4,4> Fqq = 0.5* (Eigen::Matrix<double,4,4>()<<0,-w.x(),-w.y(),-w.z(),
                                                                        w.x(),0,w.z(),-w.y(),
                                                                        w.y(),-w.z(),0,w.x(),
                                                                        w.z(),w.y(),-w.x(),0).finished();
    F_.block<4,4>(INDEX_STATE_ORI,INDEX_STATE_ORI) = Fqq;

    Eigen::Matrix<double,4,3> Fqkesi  = 0.5 * (Eigen::Matrix<double,4,3>()<<-q1,-q2,-q3,
                                                                        q0,-q3,q2,
                                                                        q3,q0,-q1,
                                                                        -q2,q1,q0).finished();
    F_.block<4,3>(INDEX_STATE_ORI,INDEX_STATE_GYRO_BIAS) = Fqkesi;         

    B_.setZero();
    B_.block<3,3>(INDEX_STATE_VEL, 3) = last_state.G_R_I;
    B_.block<4,3>(INDEX_STATE_ORI, 0) = Fqkesi;                           

    TypeMatrixF Fk = TypeMatrixF::Identity() + F_ * delta_t;
    TypeMatrixB Bk = B_ * delta_t;

    Q_.setZero();
    Q_.block<3,3>(0,0) = Eigen::Matrix3d::Identity() * gyro_noise_;
    Q_.block<3,3>(3,3) = Eigen::Matrix3d::Identity() * acc_noise_;

    // ****************************************************************************************************


    // ****************************************************************************************************
    // STEP 3: Calculate the prior state estimation error covariance matrix
    state->cov = Fk * last_state.cov * Fk.transpose() + Bk * Q_ * Bk.transpose();;

    // ****************************************************************************************************

    // Time and imu.
    state->timestamp = cur_imu->timestamp;
    state->imu_data_ptr = cur_imu;
}

}  // namespace ImuGpsLocalization