// 导入GPS处理头文件
#include "imu_gps_localizer/gps_processor.h"
// 导入坐标转换头文件
#include "imu_gps_localizer/utils.h"
// ImuGpsLocalization命名空间
namespace ImuGpsLocalization {
// 构造函数，接受一个Eigen::Vector3d类型的参数I_p_Gps，使用成员初始化列表初始化I_p_Gps_。
GpsProcessor::GpsProcessor(const Eigen::Vector3d& I_p_Gps) : I_p_Gps_(I_p_Gps) { }

// 通过GPS位置数据更新状态，接受一个Eigen::Vector3d类型的参数init_lla、一个GpsPositionDataPtr类型的参数gps_data_ptr和一个State类型的指针state。
bool GpsProcessor::UpdateStateByGpsPosition(const Eigen::Vector3d& init_lla, const GpsPositionDataPtr gps_data_ptr, State* state) {
    // 雅可比矩阵和残差。
    Eigen::Matrix<double, 3, 15> H;
    Eigen::Vector3d residual;
    // 计算雅可比矩阵和残差。
    ComputeJacobianAndResidual(init_lla, gps_data_ptr, *state, &H, &residual);
    // GPS位置数据的协方差矩阵。
    const Eigen::Matrix3d& V = gps_data_ptr->cov;

    // EKF.
    // 协方差矩阵。
    const Eigen::MatrixXd& P = state->cov;
    // 卡尔曼增益。
    const Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + V).inverse();
    // 误差增量。
    const Eigen::VectorXd delta_x = K * residual;

    // Add delta_x to state.
    // 将增量添加到状态。
    AddDeltaToState(delta_x, state);

    // Covarance.
    // 卡尔曼滤波更新协方差矩阵。
    const Eigen::MatrixXd I_KH = Eigen::Matrix<double, 15, 15>::Identity() - K * H;
    // 更新协方差矩阵。
    state->cov = I_KH * P * I_KH.transpose() + K * V * K.transpose();
}

// 定义函数，用于计算雅可比矩阵和残差。传入参数为初始经纬度、GPS位置数据、状态、雅可比矩阵和残差。
void GpsProcessor::ComputeJacobianAndResidual(const Eigen::Vector3d& init_lla,  
                                              const GpsPositionDataPtr gps_data, 
                                              const State& state,
                                              Eigen::Matrix<double, 3, 15>* jacobian,
                                              Eigen::Vector3d* residual) {
    // IMU框架在全局框架中的原点。
    const Eigen::Vector3d& G_p_I   = state.G_p_I;
    // 从IMU框架到全局框架的旋转矩阵。
    const Eigen::Matrix3d& G_R_I   = state.G_R_I;

    // Convert wgs84 to ENU frame.
    // 将经纬度和高度转换为本地ENU坐标系。
    Eigen::Vector3d G_p_Gps;
    ConvertLLAToENU(init_lla, gps_data->lla, &G_p_Gps);

    // Compute residual.
    // 计算残差。
    *residual = G_p_Gps - (G_p_I + G_R_I * I_p_Gps_);

    // Compute jacobian.
    // 计算雅可比矩阵。
    jacobian->setZero();
    jacobian->block<3, 3>(0, 0)  = Eigen::Matrix3d::Identity();
    jacobian->block<3, 3>(0, 6)  = - G_R_I * GetSkewMatrix(I_p_Gps_);
}

// 定义函数，用于将增量添加到状态。传入参数为增量和状态。
void AddDeltaToState(const Eigen::Matrix<double, 15, 1>& delta_x, State* state) {
    state->G_p_I     += delta_x.block<3, 1>(0, 0);
    state->G_v_I     += delta_x.block<3, 1>(3, 0);
    state->acc_bias  += delta_x.block<3, 1>(9, 0);
    state->gyro_bias += delta_x.block<3, 1>(12, 0);

    if (delta_x.block<3, 1>(6, 0).norm() > 1e-12) {
        state->G_R_I *= Eigen::AngleAxisd(delta_x.block<3, 1>(6, 0).norm(), delta_x.block<3, 1>(6, 0).normalized()).toRotationMatrix();
    }
}

}  // namespace ImuGpsLocalization