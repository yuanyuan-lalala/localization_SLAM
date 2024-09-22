#ifndef LIDAR_LOCALIZATION_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_
#define LIDAR_LOCALIZATION_MAPPING_FRONT_END_FRONT_END_FLOW_HPP_

#include <ros/ros.h>

#include "subscriber/cloud_subscriber.hpp"
#include "publisher/odometry_publisher.hpp"

#include "mapping/front_end/front_end.hpp"

namespace localization {
class FrontEndFlow {
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FrontEndFlow(ros::NodeHandle& nh);

    bool Run();

  private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool UpdateLaserOdometry();
    bool PublishData();

  private:
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
    std::shared_ptr<FrontEnd> front_end_ptr_;
    

    std::deque<CloudData<PointXYZIRT>,Eigen::aligned_allocator<CloudData<PointXYZIRT>>> cloud_data_buff_;

    CloudData<PointXYZIRT> current_cloud_data_;

    Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();
};
}

#endif