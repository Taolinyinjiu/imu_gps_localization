// 导入initializer.h和utils.h头文件，实现Initializer类的AddImuData和AddGpsPositionData方法。
#include "imu_gps_localizer/imu_processor.h"
#include "imu_gps_localizer/utils.h"
// 导入用于线性计算的Eigen库和用于日志记录的glog库。
#include <Eigen/Dense>
#include <glog/logging.h>

// 定义ImuGpsLocalization命名空间。
namespace ImuGpsLocalization {

// TODO：修改noise和bias
// 实现ImuProcessor类的构造函数。接受四个double类型的参数acc_noise、gyro_noise、acc_bias_noise、gyro_bias_noise和一个Eigen::Vector3d类型的参数gravity。
ImuProcessor::ImuProcessor(const double acc_noise, const double gyro_noise,
                           const double acc_bias_noise, const double gyro_bias_noise,
                           const Eigen::Vector3d& gravity)
    : acc_noise_(acc_noise), gyro_noise_(gyro_noise), 
      acc_bias_noise_(acc_bias_noise), gyro_bias_noise_(gyro_bias_noise),
      gravity_(gravity) { }
// 上面使用列表进行变量的初始化，将传入的参数acc_noise、gyro_noise、acc_bias_noise、gyro_bias_noise和gravity分别赋值给成员变量acc_noise_、gyro_noise_、acc_bias_noise_、gyro_bias_noise_和gravity_。

// 实现ImuProcessor类的Predict方法。接受两个ImuDataPtr类型的参数last_imu和cur_imu，以及一个State类型的指针state。
void ImuProcessor::Predict(const ImuDataPtr last_imu, const ImuDataPtr cur_imu, State* state) {
    // 计算时间差。
    const double delta_t = cur_imu->timestamp - last_imu->timestamp;
    const double delta_t2 = delta_t * delta_t;

    // 将传入的state赋值给last_state。
    State last_state = *state;

    // 计算加速度和角速度的偏差。
    const Eigen::Vector3d acc_unbias = 0.5 * (last_imu->acc + cur_imu->acc) - last_state.acc_bias;
    const Eigen::Vector3d gyro_unbias = 0.5 * (last_imu->gyro + cur_imu->gyro) - last_state.gyro_bias;

    // Normal state. 
    // Using P58. of "Quaternion kinematics for the error-state Kalman Filter".
    // 计算位置、速度和姿态。
    state->G_p_I = last_state.G_p_I + last_state.G_v_I * delta_t + 
                   0.5 * (last_state.G_R_I * acc_unbias + gravity_) * delta_t2;
    state->G_v_I = last_state.G_v_I + (last_state.G_R_I * acc_unbias + gravity_) * delta_t;
    const Eigen::Vector3d delta_angle_axis = gyro_unbias * delta_t;
    // 如果角度变化大于1e-12，则更新姿态。
    if (delta_angle_axis.norm() > 1e-12) {
        // 使用四元数更新姿态。
        state->G_R_I = last_state.G_R_I * Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix();
    }
    // Error-state. Not needed.

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

    state->cov = Fx * last_state.cov * Fx.transpose() + Fi * Qi * Fi.transpose();

    // Time and imu.
    state->timestamp = cur_imu->timestamp;
    state->imu_data_ptr = cur_imu;
}

}  // namespace ImuGpsLocalization