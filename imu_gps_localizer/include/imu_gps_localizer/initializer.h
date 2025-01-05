#pragma once

#include <deque>
// 导入基本数据类型头文件
#include "imu_gps_localizer/base_type.h"
// 命名空间ImuGpsLocalization
namespace ImuGpsLocalization {
// 设置IMU数据缓冲区长度和加速度标准差限制
constexpr int kImuDataBufferLength = 100;
constexpr int kAccStdLimit         = 3.;
// 声明初始化器类
class Initializer {
public:
    // 构造函数，接受一个Eigen::Vector3d类型的参数init_I_p_Gps。
    Initializer(const Eigen::Vector3d& init_I_p_Gps);
    // 添加IMU数据。
    void AddImuData(const ImuDataPtr imu_data_ptr);
    // 添加GPS位置数据。
    bool AddGpsPositionData(const GpsPositionDataPtr gps_data_ptr, State* state);

private:
    // 计算从IMU数据中获取的旋转矩阵。
    bool ComputeG_R_IFromImuData(Eigen::Matrix3d* G_R_I);
    // 计算从GPS数据中获取的位置。
    Eigen::Vector3d init_I_p_Gps_;
    // IMU数据缓冲区
    std::deque<ImuDataPtr> imu_buffer_;
};

}  // namespace ImuGpsLocalization