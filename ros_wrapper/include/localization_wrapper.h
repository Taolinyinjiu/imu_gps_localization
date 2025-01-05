#pragma once

#include <fstream>
#include <memory>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include "imu_gps_localizer/imu_gps_localizer.h"

// 声明类LocalizationWrapper
class LocalizationWrapper {
public:
    // 构造函数，接受一个ros::NodeHandle类型的引用nh。
    LocalizationWrapper(ros::NodeHandle& nh);
    // 析构函数。
    ~LocalizationWrapper();
    // IMU回调函数，接受一个sensor_msgs::ImuConstPtr类型的指针imu_msg_ptr。
    void ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);
    // GPS位置回调函数，接受一个sensor_msgs::NavSatFixConstPtr类型的指针gps_msg_ptr。
    void GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr);

private:
    // 记录状态。
    void LogState(const ImuGpsLocalization::State& state);
    // 记录GPS位置。
    void LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data);
    // 将状态转换为ROS话题。
    void ConvertStateToRosTopic(const ImuGpsLocalization::State& state);
    // 订阅IMU数据。
    ros::Subscriber imu_sub_;
    // 订阅GPS位置数据。
    ros::Subscriber gps_position_sub_;
    // 发布状态话题。
    ros::Publisher state_pub_;

    // 文件输出流，用于记录状态和GPS数据
    std::ofstream file_state_;
    std::ofstream file_gps_;

    // ROS路径消息，用于发布路径
    nav_msgs::Path ros_path_;

    // 智能指针，管理ImuGpsLocalizer对象的生命周期
    std::unique_ptr<ImuGpsLocalization::ImuGpsLocalizer> imu_gps_localizer_ptr_;
};