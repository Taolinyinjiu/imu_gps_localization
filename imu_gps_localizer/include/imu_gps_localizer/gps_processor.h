#pragma once 
// #pragma once 是一种编译器指令，用于防止头文件被多次包含。它的作用类似于传统的头文件保护宏，但更简洁和易于使用。
// Eigen头文件，用于线性代数运算
#include <Eigen/Dense>
// 导入基本数据类型头文件
#include "imu_gps_localizer/base_type.h"
// ImuGpsLocalization命名空间
namespace ImuGpsLocalization {
// 声明GpsProcessor类
class GpsProcessor {
public:
    // 构造函数，接受一个Eigen::Vector3d类型的参数I_p_Gps。
    GpsProcessor(const Eigen::Vector3d& I_p_Gps);
    // 通过GPS位置数据更新状态。
    // 接受一个Eigen::Vector3d类型的参数init_lla、一个GpsPositionDataPtr类型的参数gps_data_ptr和一个State类型的指针state。
    bool UpdateStateByGpsPosition(const Eigen::Vector3d& init_lla, const GpsPositionDataPtr gps_data_ptr, State* state);

private:    
    // 计算雅可比矩阵和残差。
    void ComputeJacobianAndResidual(const Eigen::Vector3d& init_lla,  
                                    const GpsPositionDataPtr gps_data, 
                                    const State& state,
                                    Eigen::Matrix<double, 3, 15>* jacobian,
                                    Eigen::Vector3d* residual);
    // GPS位置数据的协方差矩阵。
    const Eigen::Vector3d I_p_Gps_;  
};
// 将增量添加到状态。
void AddDeltaToState(const Eigen::Matrix<double, 15, 1>& delta_x, State* state);

}  // namespace ImuGpsLocalization