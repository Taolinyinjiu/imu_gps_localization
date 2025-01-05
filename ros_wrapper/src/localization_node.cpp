#include <memory>

// #include <glog/logging.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include "localization_wrapper.h"

int main (int argc, char** argv) {
    // Set glog.
    // FLAGS_colorlogtostderr = true;
    // google::InitGoogleLogging(argv[0]);
    // Initialize ros.
    ros::init(argc, argv, "imu_gps_localization");
    ros::NodeHandle nh;
    
    // Initialize localizer.
    LocalizationWrapper localizer(nh);
    ROS_INFO("Start imu_gps_localization node.");
    ros::spin();
    return 1;
}