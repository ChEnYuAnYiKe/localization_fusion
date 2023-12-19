#pragma once

#include <memory>
#include <Eigen/Dense>

namespace ImuGpsLocalization {

struct ImuData {
    double timestamp;      // In second.

    Eigen::Vector3d acc;   // Acceleration in m/s^2
    Eigen::Vector3d gyro;  // Angular velocity in radian/s.
};
using ImuDataPtr = std::shared_ptr<ImuData>;

struct GpsPositionData {
    double timestamp;     // In second.
 
    Eigen::Vector3d lla;  // Latitude in degree, longitude in degree, and altitude in meter.
    Eigen::Matrix3d cov;  // Covariance in m^2.
};
using GpsPositionDataPtr = std::shared_ptr<GpsPositionData>;

struct MagData {
    double timestamp;

    Eigen::Vector3d mag_xyz;  // x, y, and z components of the field vector            
    Eigen::Matrix3d cov;
};  // added data-type 
using MagDataPtr = std::shared_ptr<MagData>;

// added param: Set the state basic types for EKF
static const unsigned int DIM_STATE = 16;
static const unsigned int DIM_STATE_NOISE = 6; // 噪声只有6维，陀螺仪和加速度计的bias
static const unsigned int DIM_MEASUREMENT = 3; // 观测向量只有3维
static const unsigned int DIM_MEASUREMENT_NOISE = 3; // 观测噪声

static const unsigned int INDEX_STATE_POSI = 0;  // 位置
static const unsigned int INDEX_STATE_VEL = 3;  // 速度
static const unsigned int INDEX_STATE_ORI = 6;  // 角度
static const unsigned int INDEX_STATE_ACC_BIAS = 10;  // 加速度计bias
static const unsigned int INDEX_STATE_GYRO_BIAS = 13;  // 陀螺仪bias
static const unsigned int INDEX_MEASUREMENT_POSI = 0;

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

struct State {
    double timestamp;
    
    Eigen::Vector3d lla;       // WGS84 position.

    Eigen::Vector3d G_p_I;     // The original point of the IMU frame in the Global frame.
    Eigen::Vector3d G_v_I;     // The velocity original point of the IMU frame in the Global frame.
    Eigen::Quaterniond G_q;    // The quaternions of the UAV in the Global coord. (added)
    Eigen::Vector3d acc_bias;  // The bias of the acceleration sensor.
    Eigen::Vector3d gyro_bias; // The bias of the gyroscope sensor.

    TypeVectorX state_vector;  // The combination of the data above

    Eigen::Matrix3d G_R_I;     // The rotation from the IMU frame to the Global frame.

    // Covariance Matirx.
    Eigen::Matrix<double, DIM_STATE, DIM_STATE> cov;  // Changed! from 15-dim to 16-dim
                                        // using quaternions(4-dim) instead of picth/roll/yaw
    // The imu data.
    ImuDataPtr imu_data_ptr; 
};

}  // ImuGpsLocalization