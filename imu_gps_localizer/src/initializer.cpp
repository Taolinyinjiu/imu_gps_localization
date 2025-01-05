// 导入initializer.h和utils.h头文件，实现Initializer类的AddImuData和AddGpsPositionData方法。
#include "imu_gps_localizer/initializer.h"
#include "imu_gps_localizer/utils.h"
// 导入用于线性计算的Eigen库和用于日志记录的glog库。
#include <Eigen/Dense>
#include <glog/logging.h>

// 定义ImuGpsLocalization命名空间。
namespace ImuGpsLocalization {
// 实现Initializer类的构造函数。接受一个 Eigen::Vector3d 类型的参数 init_I_p_Gps，并将其赋值给成员变量 init_I_p_Gps_。
Initializer::Initializer(const Eigen::Vector3d& init_I_p_Gps) 
    : init_I_p_Gps_(init_I_p_Gps) { } //初始化列表，将传入的参数init_I_p_Gps赋值给成员变量init_I_p_Gps_。

// 实现Initializer类的AddImuData方法。接受一个 ImuDataPtr 类型的参数 imu_data_ptr，并将其添加到 imu_buffer_ 中。
void Initializer::AddImuData(const ImuDataPtr imu_data_ptr) {
    // 将传入的参数imu_data_ptr添加到imu_buffer_中。
    imu_buffer_.push_back(imu_data_ptr);
    // 如果imu_buffer_的大小超过了kImuDataBufferLength，则删除最前面的元素。
    if (imu_buffer_.size() > kImuDataBufferLength) {
        // 删除imu_buffer_的第一个元素。
        imu_buffer_.pop_front();
    }
}

// 实现Initializer类的AddGpsPositionData方法。接受一个 GpsPositionDataPtr 类型的参数 gps_data_ptr 和一个 State 类型的指针 state。
bool Initializer::AddGpsPositionData(const GpsPositionDataPtr gps_data_ptr, State* state) {
    // 如果imu_buffer_的大小小于kImuDataBufferLength，则记录警告日志并返回false。
    if (imu_buffer_.size() < kImuDataBufferLength) {
        LOG(WARNING) << "[AddGpsPositionData]: No enought imu data!";
        return false;
    }
    // 获取imu_buffer_的最后一个元素。
    const ImuDataPtr last_imu_ptr = imu_buffer_.back();
    // TODO: 需要同步所有传感器的时间戳，这里只是简单的比较了gps和imu的时间戳。
    // 如果gps_data_ptr的时间戳和last_imu_ptr的时间戳之差大于0.5，则记录错误日志并返回false。
    if (std::abs(gps_data_ptr->timestamp - last_imu_ptr->timestamp) > 0.5) {
        LOG(ERROR) << "[AddGpsPositionData]: Gps and imu timestamps are not synchronized!";
        return false;
    }

    // 更新state的时间戳为last_imu_ptr的时间戳。
    state->timestamp = last_imu_ptr->timestamp;
    // 更新state的imu_data_ptr为last_imu_ptr。
    state->imu_data_ptr = last_imu_ptr;

    // Set initial mean.
    state->G_p_I.setZero();

    // We have no information to set initial velocity. 
    // So, just set it to zero and given big covariance.
    state->G_v_I.setZero();

    // 我们可以使用重力方向来设置滚转和俯仰。
    // 但是，我们无法从中得到偏航角。
    // 因此，我们设置偏航为零，并给它一个大的协方差。
    // 
    if (!ComputeG_R_IFromImuData(&state->G_R_I)) {
        LOG(WARNING) << "[AddGpsPositionData]: Failed to compute G_R_I!";
        return false;
    }

    // Set bias to zero.
    state->acc_bias.setZero();
    state->gyro_bias.setZero();

    // Set covariance.
    state->cov.setZero();
    state->cov.block<3, 3>(0, 0) = 100. * Eigen::Matrix3d::Identity(); // position std: 10 m
    state->cov.block<3, 3>(3, 3) = 100. * Eigen::Matrix3d::Identity(); // velocity std: 10 m/s
    // roll pitch std 10 degree.
    state->cov.block<2, 2>(6, 6) = 10. * kDegreeToRadian * 10. * kDegreeToRadian * Eigen::Matrix2d::Identity();
    state->cov(8, 8)             = 100. * kDegreeToRadian * 100. * kDegreeToRadian; // yaw std: 100 degree.
    // Acc bias.
    state->cov.block<3, 3>(9, 9) = 0.0004 * Eigen::Matrix3d::Identity();
    // Gyro bias.
    state->cov.block<3, 3>(12, 12) = 0.0004 * Eigen::Matrix3d::Identity();

    return true;
}

// 实现Initializer类的ComputeG_R_IFromImuData方法。接受一个 Eigen::Matrix3d 类型的指针 G_R_I。
bool Initializer::ComputeG_R_IFromImuData(Eigen::Matrix3d* G_R_I) {
    // 计算imu缓冲区的均值和标准差。
    // 声明并初始化一个Eigen::Vector3d类型的变量sum_acc，其值为(0., 0., 0.)。
    Eigen::Vector3d sum_acc(0., 0., 0.);
    // 遍历imu_buffer_中的每一个元素imu_data。
    for (const auto imu_data : imu_buffer_) {
        // 对加速度求和。
        sum_acc += imu_data->acc;
    }
    // 计算加速度的均值。
    const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buffer_.size();
    // 声明并初始化一个Eigen::Vector3d类型的变量sum_err2，其值为(0., 0., 0.)。
    Eigen::Vector3d sum_err2(0., 0., 0.);
    // 遍历imu_buffer_中的每一个元素imu_data。
    for (const auto imu_data : imu_buffer_) {
        // 对加速度误差的平方求和。
        sum_err2 += (imu_data->acc - mean_acc).cwiseAbs2();
    }
    // 计算加速度的标准差。
    const Eigen::Vector3d std_acc = (sum_err2 / (double)imu_buffer_.size()).cwiseSqrt();
    // 如果加速度的标准差的最大值大于kAccStdLimit，则记录警告日志并返回false。
    if (std_acc.maxCoeff() > kAccStdLimit) {
        LOG(WARNING) << "[ComputeG_R_IFromImuData]: Too big acc std: " << std_acc.transpose();
        return false;
    }
    
    // 计算旋转，NED坐标系下的三个轴分别为x轴（北），y轴（东）和z轴（向下）。
    // z-axis (down).
    const Eigen::Vector3d& z_axis = -mean_acc.normalized(); 

    // x-axis (north).
    Eigen::Vector3d x_axis = 
        Eigen::Vector3d::UnitX() - z_axis * z_axis.transpose() * Eigen::Vector3d::UnitX();
    x_axis.normalize();

    // y-axis (east).
    Eigen::Vector3d y_axis = z_axis.cross(x_axis);
    y_axis.normalize();

    Eigen::Matrix3d I_R_G;
    I_R_G.block<3, 1>(0, 0) = x_axis;
    I_R_G.block<3, 1>(0, 1) = y_axis;
    I_R_G.block<3, 1>(0, 2) = z_axis;

    *G_R_I = I_R_G.transpose();

    return true;
}

}  // namespace ImuGpsLocalization