#ifndef LIDAR_LOCALIZATION_PUBLISHER_IMU_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_IMU_PUBLISHER_HPP_

#include "sensor_msgs/Imu.h"
#include "sensor_data/IMU_data.hpp"
#include"ros/ros.h"
namespace localization {
class IMUPublisher {
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    IMUPublisher(ros::NodeHandle& nh,
                 std::string topic_name,
                 size_t buff_size,
                 std::string frame_id);
    IMUPublisher() = default;

    void Publish(IMUData imu_data);

    bool HasSubscribers();

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
};
} 
#endif