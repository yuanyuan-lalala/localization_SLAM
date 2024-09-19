#ifndef LIDAR_LOCALIZATION_PUBLISHER_KEY_FRAME_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_KEY_FRAME_PUBLISHER_HPP_

#include <string>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "sensor_data/key_frame.hpp"

namespace localization {
class KeyFramePublisher {
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    KeyFramePublisher(ros::NodeHandle& nh, 
                      std::string topic_name, 
                      std::string frame_id,
                      int buff_size);
    KeyFramePublisher() = default;

    void Publish(KeyFrame& key_frame);

    bool HasSubscribers();

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_ = "";
};
}
#endif