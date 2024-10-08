/*
 * @Description: 订阅imu数据
 * @Author: Ren Qian
 * @Date: 2019-08-19 19:22:17
 */
#ifndef LIDAR_LOCALIZATION_SUBSCRIBER_IMU_SUBSCRIBER_HPP_
#define LIDAR_LOCALIZATION_SUBSCRIBER_IMU_SUBSCRIBER_HPP_

#include <deque>
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "sensor_data/IMU_data.hpp"

namespace localization {
class IMUSubscriber {
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    IMUSubscriber() = default;
    void ParseData(std::deque<IMUData,Eigen::aligned_allocator<IMUData>>& deque_imu_data);

  private:
    void msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;

    std::deque<IMUData,Eigen::aligned_allocator<IMUData>> new_imu_data_; 
};
}
#endif