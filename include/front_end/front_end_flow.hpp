#ifndef LIDAR_LOCALIZATION_FRONT_END_FRONT_END_FLOW_HPP_
#define LIDAR_LOCALIZATION_FRONT_END_FRONT_END_FLOW_HPP_

#include <ros/ros.h>

#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/imu_subscriber.hpp"
#include "subscriber/gnss_subscriber.hpp"
#include "tf_listener/tf_listener.hpp"
#include "publisher/cloud_publisher.hpp"
#include "publisher/odometry_publisher.hpp"
#include "front_end/front_end.hpp"
#include"subscriber/velocity_subscriber.hpp"
namespace localization {
class FrontEndFlow {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
    FrontEndFlow(ros::NodeHandle& nh);

    bool Run();
    bool SaveMap();
    bool PublishGlobalMap();

  private:
    bool ReadData();
    bool InitCalibration();
    bool InitGNSS();
    bool HasData();
    bool ValidData();
    bool UpdateGNSSOdometry();
    bool UpdateLaserOdometry();
    bool PublishData();

  private:
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
    std::shared_ptr<TFListener> lidar_to_imu_ptr_;
    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
    std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
    std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
    std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
    std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
    std::shared_ptr<FrontEnd> front_end_ptr_;
    std::shared_ptr<VelocitySubscriber> velocity_sub_ptr_;


    std::deque<CloudData<PointXYZIRT>,Eigen::aligned_allocator<CloudData<PointXYZIRT>>> cloud_data_buff_;
    std::deque<IMUData, Eigen::aligned_allocator<IMUData>> imu_data_buff_;
    std::deque<GNSSData, Eigen::aligned_allocator<GNSSData>> gnss_data_buff_;
    std::deque<VelocityData,Eigen::aligned_allocator<VelocityData>> velocity_data_buff_;
    
    Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();
    CloudData<PointXYZIRT> current_cloud_data_;
    IMUData current_imu_data_;
    GNSSData current_gnss_data_;

    CloudData<PointXYZIRT>::CLOUD_PTR local_map_ptr_;
    CloudData<PointXYZIRT>::CLOUD_PTR global_map_ptr_;
    CloudData<PointXYZIRT>::CLOUD_PTR current_scan_ptr_;
    Eigen::Matrix4f gnss_odometry_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
};
}

#endif