#pragma once

#include <Eigen/Core>
// 导入基本数据类型头文件
#include "imu_gps_localizer/base_type.h"
// 导入gps处理头文件
#include "imu_gps_localizer/gps_processor.h"
// 导入imu处理头文件
#include "imu_gps_localizer/imu_processor.h"
// 导入初始化器头文件
#include "imu_gps_localizer/initializer.h"
// 命名空间ImuGpsLocalization
namespace ImuGpsLocalization {
// 声明ImuGpsLocalizer类
class ImuGpsLocalizer {
public:
    // 构造函数，接受四个double类型的参数acc_noise、gyro_noise、acc_bias_noise、gyro_bias_noise和一个Eigen::Vector3d类型的参数I_p_Gps。
    ImuGpsLocalizer(const double acc_noise, const double gyro_noise,
                    const double acc_bias_noise, const double gyro_bias_noise,
                    const Eigen::Vector3d& I_p_Gps);
    // 处理IMU数据。
    bool ProcessImuData(const ImuDataPtr imu_data_ptr, State* fused_state);
    // 处理GPS位置数据。
    bool ProcessGpsPositionData(const GpsPositionDataPtr gps_data_ptr);

private:
    // 初始化器指针
    std::unique_ptr<Initializer>  initializer_;
    // IMU处理器指针
    std::unique_ptr<ImuProcessor> imu_processor_;
    // GPS处理器指针
    std::unique_ptr<GpsProcessor> gps_processor_;
    // 是否初始化
    bool initialized_;
    // 初始的经纬度和高度
    Eigen::Vector3d init_lla_; // The initial reference gps point.
    // 状态 
    State state_;
};

}  // namespace ImuGpsLocalization