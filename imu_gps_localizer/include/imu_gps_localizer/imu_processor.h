#pragma once
// 导入基本数据类型头文件
#include "imu_gps_localizer/base_type.h"
// 命名空间ImuGpsLocalization
namespace ImuGpsLocalization {
// 声明ImuProcessor类
class ImuProcessor{
public:
    // 构造函数，接受四个double类型的参数acc_noise、gyro_noise、acc_bias_noise、gyro_bias_noise和一个Eigen::Vector3d类型的参数gravity。
    ImuProcessor(const double acc_noise, const double gyro_noise,
                 const double acc_bias_noise, const double gyro_bias_noise,
                 const Eigen::Vector3d& gravity);
    // 预测状态。
    void Predict(const ImuDataPtr last_imu, const ImuDataPtr cur_imu, State* state);

private:
    // 加速度噪声、角速度噪声、加速度偏差噪声和角速度偏差噪声。
    const double acc_noise_;
    const double gyro_noise_;
    const double acc_bias_noise_;
    const double gyro_bias_noise_;
    // 重力加速度。
    const Eigen::Vector3d gravity_;
};

}  // namespace ImuGpsLocalization