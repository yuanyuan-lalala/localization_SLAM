#ifndef LIDAR_LOCALIZATION_MAPPING_BACK_END_FRONT_END_FLOW_HPP_
#define LIDAR_LOCALIZATION_MAPPING_BACK_END_FRONT_END_FLOW_HPP_

#include <ros/ros.h>

#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/odometry_subscriber.hpp"

#include "publisher/odometry_publisher.hpp"
#include "publisher/key_frame_publisher.hpp"
#include "publisher/key_frames_publisher.hpp"

#include "mapping/back_end/back_end.hpp"

namespace localization {
class BackEndFlow {
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BackEndFlow(ros::NodeHandle& nh);

    bool Run();

  private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool UpdateBackEnd();
    bool SaveTrajectory();
    bool PublishData();

  private:
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> gnss_pose_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> laser_odom_sub_ptr_;

    std::shared_ptr<OdometryPublisher> transformed_odom_pub_ptr_;
    std::shared_ptr<KeyFramePublisher> key_frame_pub_ptr_;
    std::shared_ptr<KeyFramesPublisher> key_frames_pub_ptr_;
    std::shared_ptr<BackEnd> back_end_ptr_;

    std::deque<CloudData<PointXYZIRT>,Eigen::aligned_allocator<CloudData<PointXYZIRT>>> cloud_data_buff_;
    std::deque<PoseData,Eigen::aligned_allocator<PoseData>> gnss_pose_data_buff_;
    std::deque<PoseData,Eigen::aligned_allocator<PoseData>> laser_odom_data_buff_;

    PoseData current_gnss_pose_data_;
    PoseData current_laser_odom_data_;
    CloudData<PointXYZIRT> current_cloud_data_;
};
}

#endif