#ifndef LIDAR_LOCALIZATION_PUBLISHER_KEY_FRAMES_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_KEY_FRAMES_PUBLISHER_HPP_

#include <string>
#include <deque>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "sensor_data/key_frame.hpp"

namespace localization {
class KeyFramesPublisher {
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KeyFramesPublisher(ros::NodeHandle& nh, 
                      std::string topic_name, 
                      std::string frame_id,
                      int buff_size);
    KeyFramesPublisher() = default;

    void Publish(const std::deque<KeyFrame,Eigen::aligned_allocator<KeyFrame>>& key_frames);

    bool HasSubscribers();

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_ = "";
};
}
#endif