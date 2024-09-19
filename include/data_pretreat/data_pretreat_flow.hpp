#ifndef LIDAR_LOCALIZATION_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_
#define LIDAR_LOCALIZATION_DATA_PRETREAT_DATA_PRETREAT_FLOW_HPP_

#include <ros/ros.h>
// subscriber
#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/imu_subscriber.hpp"
#include "subscriber/velocity_subscriber.hpp"
#include "subscriber/gnss_subscriber.hpp"
#include "tf_listener/tf_listener.hpp"
// publisher
#include "publisher/cloud_publisher.hpp"
#include "publisher/odometry_publisher.hpp"
// models
#include "models/scan_adjust/distortion_adjust.hpp"

namespace localization {
class DataPretreatFlow {
  
  public:
    DataPretreatFlow(ros::NodeHandle& nh);

    bool Run();

  private:
    bool ReadData();
    bool InitCalibration();
    bool InitGNSS();
    bool HasData();
    bool ValidData();
    bool TransformData();
    bool PublishData();

  private:
    // subscriber
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
    std::shared_ptr<TFListener> lidar_to_imu_ptr_;
    // publisher
    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
    std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
    // models
    std::shared_ptr<DistortionAdjust> distortion_adjust_ptr_;

    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();
    std::deque<CloudData<PointXYZIRT>,Eigen::aligned_allocator<CloudData<PointXYZIRT>>> cloud_data_buff_;
    std::deque<IMUData,Eigen::aligned_allocator<IMUData>> imu_data_buff_;
    std::deque<VelocityData,Eigen::aligned_allocator<VelocityData>> velocity_data_buff_;
    std::deque<GNSSData,Eigen::aligned_allocator<GNSSData>> gnss_data_buff_;

    CloudData<PointXYZIRT> current_cloud_data_;
    IMUData current_imu_data_;
    VelocityData current_velocity_data_;
    GNSSData current_gnss_data_;

    Eigen::Matrix4f gnss_pose_ = Eigen::Matrix4f::Identity();
};
}

#endif