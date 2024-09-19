#ifndef LIDAR_LOCALIZATION_MAPPING_VIEWER_VIEWER_FLOW_HPP_
#define LIDAR_LOCALIZATION_MAPPING_VIEWER_VIEWER_FLOW_HPP_

#include <deque>
#include <ros/ros.h>
// subscriber
#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/odometry_subscriber.hpp"
#include "subscriber/key_frame_subscriber.hpp"
#include "subscriber/key_frames_subscriber.hpp"
// publisher
#include "publisher/odometry_publisher.hpp"
#include "publisher/cloud_publisher.hpp"
// viewer
#include "mapping/viewer/viewer.hpp"
#include"global_settings/global_settings.hpp"

namespace localization {
class ViewerFlow {
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ViewerFlow(ros::NodeHandle& nh);

    bool Run();
    bool SaveMap();

  private:
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool UpdateViewer();
    bool PublishData();

  private:
    // subscriber
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> transformed_odom_sub_ptr_;
    std::shared_ptr<KeyFrameSubscriber> key_frame_sub_ptr_;
    std::shared_ptr<KeyFramesSubscriber> optimized_key_frames_sub_ptr_;
    // publisher
    std::shared_ptr<OdometryPublisher> optimized_odom_pub_ptr_;
    std::shared_ptr<CloudPublisher> current_scan_pub_ptr_;
    std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
    std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
    // viewer
    std::shared_ptr<Viewer> viewer_ptr_;

    std::deque<CloudData<PointXYZIRT>,Eigen::aligned_allocator<CloudData<PointXYZIRT>>> cloud_data_buff_;
    std::deque<PoseData,Eigen::aligned_allocator<PoseData>> transformed_odom_buff_;
    std::deque<KeyFrame,Eigen::aligned_allocator<KeyFrame>> key_frame_buff_;
    std::deque<KeyFrame,Eigen::aligned_allocator<KeyFrame>> optimized_key_frames_;
    std::deque<KeyFrame,Eigen::aligned_allocator<KeyFrame>> all_key_frames_;

    CloudData<PointXYZIRT> current_cloud_data_;
    PoseData current_transformed_odom_;
};
}

#endif