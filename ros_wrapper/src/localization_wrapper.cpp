// 导入localization_wrapper.h头文件，包含了LocalizationWrapper类的声明。
#include "localization_wrapper.h"
// 导入iomanip库，用于格式化输入和输出
#include <iomanip>
// 导入glog库，用于日志记录
#include <glog/logging.h>
// 导入基本数据类型头文件
#include "imu_gps_localizer/base_type.h"
// 构造函数，接受一个ros::NodeHandle类型的引用nh。
LocalizationWrapper::LocalizationWrapper(ros::NodeHandle& nh) {
    // 读取参数
    double acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise;
    nh.param("acc_noise",       acc_noise, 1e-2);
    nh.param("gyro_noise",      gyro_noise, 1e-4);
    nh.param("acc_bias_noise",  acc_bias_noise, 1e-6);
    nh.param("gyro_bias_noise", gyro_bias_noise, 1e-8);
    // 定义x,y,z三个变量，分别读取参数I_p_Gps_x、I_p_Gps_y、I_p_Gps_z的值，如果没有设置则默认为0。
    double x, y, z;
    nh.param("I_p_Gps_x", x, 0.);
    nh.param("I_p_Gps_y", y, 0.);
    nh.param("I_p_Gps_z", z, 0.);
    // 定义Eigen::Vector3d类型的变量I_p_Gps，用于存储x、y、z三个值。
    const Eigen::Vector3d I_p_Gps(x, y, z);
    // 定义std::string类型的变量log_folder，用于存储日志文件夹路径。
    std::string log_folder = "/home";
    // 读取参数log_folder，如果没有设置则默认为/home。
    ros::param::get("log_folder", log_folder);

    // 日志记录
    file_state_.open(log_folder + "/state.csv");
    file_gps_.open(log_folder +"/gps.csv");

    // Initialization imu gps localizer.
    // 初始化ImuGpsLocalizer对象。使用智能指针管理对象的生命周期。
    imu_gps_localizer_ptr_ = 
        std::make_unique<ImuGpsLocalization::ImuGpsLocalizer>(acc_noise, gyro_noise,
                                                              acc_bias_noise, gyro_bias_noise,
                                                              I_p_Gps);

    // Subscribe topics.
    // 订阅话题
    imu_sub_ = nh.subscribe("/imu/data", 10,  &LocalizationWrapper::ImuCallback, this);
    gps_position_sub_ = nh.subscribe("/fix", 10,  &LocalizationWrapper::GpsPositionCallback, this);
    // 发布话题
    state_pub_ = nh.advertise<nav_msgs::Path>("fused_path", 10);
}

// 析构函数。关闭日志文件流。
LocalizationWrapper::~LocalizationWrapper() {
    file_state_.close();
    file_gps_.close();
}

// IMU回调函数，接受一个sensor_msgs::ImuConstPtr类型的指针imu_msg_ptr。
void LocalizationWrapper::ImuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
    // 构造ImuData对象指针imu_data_ptr，使用make_shared函数创建对象。
    ImuGpsLocalization::ImuDataPtr imu_data_ptr = std::make_shared<ImuGpsLocalization::ImuData>();
    // 将IMU消息的时间戳转换为秒。
    imu_data_ptr->timestamp = imu_msg_ptr->header.stamp.toSec();
    // 将IMU消息的线性加速度和角速度转换为Eigen::Vector3d类型的变量acc和gyro。
    imu_data_ptr->acc << imu_msg_ptr->linear_acceleration.x, 
                         imu_msg_ptr->linear_acceleration.y,
                         imu_msg_ptr->linear_acceleration.z;
    // 将IMU消息的角速度转换为Eigen::Vector3d类型的变量gyro。  
    imu_data_ptr->gyro << imu_msg_ptr->angular_velocity.x,
                          imu_msg_ptr->angular_velocity.y,
                          imu_msg_ptr->angular_velocity.z;
    // 定义ImuGpsLocalization::State类型的变量fused_state，用于存储融合后的状态。
    ImuGpsLocalization::State fused_state;
    // 调用ImuGpsLocalizer对象的ProcessImuData函数，传入imu_data_ptr和fused_state。
    const bool ok = imu_gps_localizer_ptr_->ProcessImuData(imu_data_ptr, &fused_state);
    if (!ok) {
        return;
    }

    // Publish fused state.
    // 将融合后的状态转换为ROS话题。
    ConvertStateToRosTopic(fused_state);
    // 发布ROS话题。
    state_pub_.publish(ros_path_);

    // Log fused state.
    // 记录融合后的状态。
    LogState(fused_state);
}

void LocalizationWrapper::GpsPositionCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr) {
    // Check the gps_status.
    if (gps_msg_ptr->status.status != 2) {
        LOG(WARNING) << "[GpsCallBack]: Bad gps message!";
        return;
    }

    ImuGpsLocalization::GpsPositionDataPtr gps_data_ptr = std::make_shared<ImuGpsLocalization::GpsPositionData>();
    gps_data_ptr->timestamp = gps_msg_ptr->header.stamp.toSec();
    gps_data_ptr->lla << gps_msg_ptr->latitude,
                         gps_msg_ptr->longitude,
                         gps_msg_ptr->altitude;
    gps_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gps_msg_ptr->position_covariance.data());

    imu_gps_localizer_ptr_->ProcessGpsPositionData(gps_data_ptr);

    LogGps(gps_data_ptr);
}

void LocalizationWrapper::LogState(const ImuGpsLocalization::State& state) {
    const Eigen::Quaterniond G_q_I(state.G_R_I);
    file_state_ << std::fixed << std::setprecision(15)
                << state.timestamp << ","
                << state.lla[0] << "," << state.lla[1] << "," << state.lla[2] << ","
                << state.G_p_I[0] << "," << state.G_p_I[1] << "," << state.G_p_I[2] << ","
                << state.G_v_I[0] << "," << state.G_v_I[1] << "," << state.G_v_I[2] << ","
                << G_q_I.x() << "," << G_q_I.y() << "," << G_q_I.z() << "," << G_q_I.w() << ","
                << state.acc_bias[0] << "," << state.acc_bias[1] << "," << state.acc_bias[2] << ","
                << state.gyro_bias[0] << "," << state.gyro_bias[1] << "," << state.gyro_bias[2] << "\n";
}

void LocalizationWrapper::LogGps(const ImuGpsLocalization::GpsPositionDataPtr gps_data) {
    file_gps_ << std::fixed << std::setprecision(15)
              << gps_data->timestamp << ","
              << gps_data->lla[0] << "," << gps_data->lla[1] << "," << gps_data->lla[2] << "\n";
}

void LocalizationWrapper::ConvertStateToRosTopic(const ImuGpsLocalization::State& state) {
    ros_path_.header.frame_id = "world";
    ros_path_.header.stamp = ros::Time::now();  

    geometry_msgs::PoseStamped pose;
    pose.header = ros_path_.header;

    pose.pose.position.x = state.G_p_I[0];
    pose.pose.position.y = state.G_p_I[1];
    pose.pose.position.z = state.G_p_I[2];

    const Eigen::Quaterniond G_q_I(state.G_R_I);
    pose.pose.orientation.x = G_q_I.x();
    pose.pose.orientation.y = G_q_I.y();
    pose.pose.orientation.z = G_q_I.z();
    pose.pose.orientation.w = G_q_I.w();

    ros_path_.poses.push_back(pose);
}