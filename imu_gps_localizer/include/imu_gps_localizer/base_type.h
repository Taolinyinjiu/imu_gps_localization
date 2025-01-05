#pragma once
// memory头文件，用于智能指针和动态内存管理
#include <memory>
// Eigen头文件，用于线性代数运算
#include <Eigen/Core>

// ImuGpsLocalization命名空间
namespace ImuGpsLocalization {
// IMU数据结构体
struct ImuData {
    double timestamp;      // 时间戳，以秒为单位

    Eigen::Vector3d acc;   // 加速度，单位为 m/s^2
    Eigen::Vector3d gyro;  // 角速度，单位为弧度/秒
};
// 使用std::shared_ptr智能指针管理ImuData结构体
using ImuDataPtr = std::shared_ptr<ImuData>;

// GPS位置数据结构体
// RMUA传入的GPS数据是使用的geometry_msgs::PoseStamped类型，有x,y,z数据和角度的四元数
struct GpsPositionData {
    double timestamp;     // 时间戳，以秒为单位。
 
    Eigen::Vector3d lla;  // 纬度（度）、经度（度）和高度（米）。
    Eigen::Matrix3d cov;  // 协方差矩阵，单位为平方米。
};


// 使用std::shared_ptr智能指针管理GpsPositionData结构体
using GpsPositionDataPtr = std::shared_ptr<GpsPositionData>;

// 估计状态结构体
struct State {
    double timestamp;         // 时间戳。 
    
    Eigen::Vector3d lla;       // WGS84坐标系，包含纬度、经度和高度。
    Eigen::Vector3d G_p_I;     // IMU框架在全局框架中的原点。
    Eigen::Vector3d G_v_I;     // IMU框架在全局框架中的速度原点。
    Eigen::Matrix3d G_R_I;     // 从IMU框架到全局框架的旋转矩阵。
    Eigen::Vector3d acc_bias;  // 加速度传感器的偏置。
    Eigen::Vector3d gyro_bias; // 陀螺仪传感器的偏置。

    // 协方差矩阵
    Eigen::Matrix<double, 15, 15> cov;

    // IMU数据
    ImuDataPtr imu_data_ptr; 
};

}  // ImuGpsLocalization