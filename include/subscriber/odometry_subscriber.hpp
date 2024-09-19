#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_ODOMETRY_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_ODOMETRY_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "sensor_data/pose_data.hpp"

namespace localization {
class OdometrySubscriber {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    OdometrySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    OdometrySubscriber() = default;
    void ParseData(std::deque<PoseData,Eigen::aligned_allocator<PoseData>>& deque_pose_data);

  private:
    void msg_callback(const nav_msgs::OdometryConstPtr& odom_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<PoseData,Eigen::aligned_allocator<PoseData>> new_pose_data_; 
};
}
#endif