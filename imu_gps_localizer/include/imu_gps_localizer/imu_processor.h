#pragma once

#include "imu_gps_localizer/base_type.h"

namespace ImuGpsLocalization {

class ImuProcessor{
public:
    ImuProcessor(const double acc_noise, const double gyro_noise,
                 const double acc_bias_noise, const double gyro_bias_noise,
                 const Eigen::Vector3d& gravity);

    void Predict(const ImuDataPtr last_imu, const ImuDataPtr cur_imu, State* state);

private:
    const double acc_noise_;
    const double gyro_noise_;
    const double acc_bias_noise_;
    const double gyro_bias_noise_;

    const Eigen::Vector3d gravity_;

    // added: Set the state basic types for EKF
    static const unsigned int DIM_STATE = 16;
    static const unsigned int DIM_STATE_NOISE = 6; // 噪声只有6维，陀螺仪和加速度计的bias
    static const unsigned int DIM_MEASUREMENT = 3; // 观测向量只有3维
    static const unsigned int DIM_MEASUREMENT_NOISE = 3; // 观测噪声

    static const unsigned int INDEX_STATE_POSI = 0;  // 位置
    static const unsigned int INDEX_STATE_VEL = 3;  // 速度
    static const unsigned int INDEX_STATE_ORI = 6;  // 角度
    static const unsigned int INDEX_STATE_ACC_BIAS = 10;  // 加速度计bias
    static const unsigned int INDEX_STATE_GYRO_BIAS = 13;  // 陀螺仪bias

    typedef typename Eigen::Matrix<double, DIM_STATE, 1> TypeVectorX;  // 状态向量 16*1
    typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, 1> TypeVectorY;  // 观测向量 GPS的位置 3*1
    typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE> TypeMatrixF;  // 16*16
    typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE_NOISE> TypeMatrixB;  // 16*6
    typedef typename Eigen::Matrix<double, DIM_STATE_NOISE, 1> TypeMatrixW;  // 陀螺仪和加速度计噪声 6*1 
    typedef typename Eigen::Matrix<double, DIM_STATE_NOISE, DIM_STATE_NOISE> TypeMatrixQ;  // 6*6
    typedef typename Eigen::Matrix<double, DIM_STATE, DIM_STATE> TypeMatrixP;  // 16*16
    typedef typename Eigen::Matrix<double, DIM_STATE, DIM_MEASUREMENT> TypeMatrixK;  // 16*3
    typedef typename Eigen::Matrix<double, DIM_MEASUREMENT_NOISE, DIM_MEASUREMENT_NOISE> TypeMatrixC;  // 3*3
    typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, DIM_STATE> TypeMatrixG;  // 3*16
    typedef typename Eigen::Matrix<double, DIM_MEASUREMENT, DIM_MEASUREMENT> TypeMatrixR;  // 3*3\

    TypeVectorX X_;
    TypeVectorY Y_;
    TypeMatrixF F_;
    TypeMatrixB B_;
    TypeMatrixW W_;
    TypeMatrixQ Q_;
    TypeMatrixP P_;
    TypeMatrixK K_;
    TypeMatrixC C_;
    TypeMatrixG G_;
    TypeMatrixC R_;
};

}  // namespace ImuGpsLocalization