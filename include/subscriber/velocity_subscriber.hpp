#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_VELOCITY_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include "geometry_msgs/TwistStamped.h"
#include "sensor_data/velocity_data.hpp"

namespace localization {
class VelocitySubscriber {
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VelocitySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    VelocitySubscriber() = default;
    void ParseData(std::deque<VelocityData,Eigen::aligned_allocator<VelocityData>>& deque_velocity_data);

  private:
    void msg_callback(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<VelocityData,Eigen::aligned_allocator<VelocityData>> new_velocity_data_; 
};
}
#endif