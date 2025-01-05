#pragma once
// 导入Eigen头文件，用于线性代数运算
#include <Eigen/Core>
// 导入GeographicLib库中的LocalCartesian头文件，可以将地理坐标系转换为本地笛卡尔坐标系
#include <LocalCartesian.hpp>
// ImuGpsLocalization命名空间
namespace ImuGpsLocalization {
// 定义常量，将角度转换为弧度和将弧度转换为角度
constexpr double kDegreeToRadian = M_PI / 180.;
constexpr double kRadianToDegree = 180. / M_PI;
// 将经纬度和高度转换为本地ENU坐标系，init_lla为初始经纬度和高度，point_lla为待转换的经纬度和高度，point_enu为转换后的ENU坐标
inline void ConvertLLAToENU(const Eigen::Vector3d& init_lla, 
                            const Eigen::Vector3d& point_lla, 
                            Eigen::Vector3d* point_enu) {
    // 创建一个LocalCartesian对象local_cartesian，使用init_lla初始化 
    static GeographicLib::LocalCartesian local_cartesian;
    // Reset方法，用于设置本地笛卡尔座标系的原点，参数为经度、纬度和高度
    local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
    // Forward方法，用于将地理坐标系转换为本地笛卡尔坐标系，参数为经度、纬度和高度，返回值为本地笛卡尔坐标系的坐标
    // 注意，Forward方法的参数和返回值都是引用类型，并且输出的坐标系是ENU坐标系
    local_cartesian.Forward(point_lla(0), point_lla(1), point_lla(2), 
                            point_enu->data()[0], point_enu->data()[1], point_enu->data()[2]);
}
// 将经纬度和高度转换为本地NED坐标系
inline void ConvertLLAToNED(const Eigen::Vector3d& init_lla, 
                            const Eigen::Vector3d& point_lla, 
                            Eigen::Vector3d* point_ned) {
    static GeographicLib::LocalCartesian local_cartesian;
    local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
    
    Eigen::Vector3d point_enu;
    // 得到ENU坐标系
    local_cartesian.Forward(point_lla(0), point_lla(1), point_lla(2), 
                            point_enu.data()[0], point_enu.data()[1], point_enu.data()[2]);
    
    // 将ENU坐标转换为NED坐标
    (*point_ned)(0) = point_enu(1);  // North
    (*point_ned)(1) = point_enu(0);  // East
    (*point_ned)(2) = -point_enu(2); // Down
}

// 将ENU坐标系转换为经纬度和高度
inline void ConvertENUToLLA(const Eigen::Vector3d& init_lla, 
                            const Eigen::Vector3d& point_enu,
                            Eigen::Vector3d* point_lla) {
    static GeographicLib::LocalCartesian local_cartesian;
    local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
    local_cartesian.Reverse(point_enu(0), point_enu(1), point_enu(2), 
                            point_lla->data()[0], point_lla->data()[1], point_lla->data()[2]);                            
}

// 将NED坐标系转换为经纬度和高度
inline void ConvertNEDToLLA(const Eigen::Vector3d& init_lla,
                            // 引用NED坐标系的点
                            const Eigen::Vector3d& point_ned,
                            Eigen::Vector3d* point_lla) {
    static GeographicLib::LocalCartesian local_cartesian;
    local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));

    // 将NED坐标系转换为ENU坐标系
    Eigen::Vector3d point_enu;
    point_enu(0) = point_ned(1);  // East
    point_enu(1) = point_ned(0);  // North
    point_enu(2) = -point_ned(2); // Up

    local_cartesian.Reverse(point_enu(0), point_enu(1), point_enu(2), 
                            point_lla->data()[0], point_lla->data()[1], point_lla->data()[2]);                            
}

// 生成一个给定三维向量的反对称矩阵
inline Eigen::Matrix3d GetSkewMatrix(const Eigen::Vector3d& v) {
    Eigen::Matrix3d w;
    w <<  0.,   -v(2),  v(1),
          v(2),  0.,   -v(0),
         -v(1),  v(0),  0.;

    return w;
}

}  // namespace ImuGpsLocalization