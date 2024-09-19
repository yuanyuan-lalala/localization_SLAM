#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_KEY_FRAME_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_KEY_FRAME_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "sensor_data/key_frame.hpp"

namespace localization {
class KeyFrameSubscriber {
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KeyFrameSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    KeyFrameSubscriber() = default;
    void ParseData(std::deque<KeyFrame,Eigen::aligned_allocator<KeyFrame>>& key_frame_buff);

  private:
    void msg_callback(const geometry_msgs::PoseStampedConstPtr& key_frame_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<KeyFrame,Eigen::aligned_allocator<KeyFrame>> new_key_frame_; 
};
}
#endif