#ifndef VIEWER_HPP_
#define VIEWER_HPP_

#include <string>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>

#include "sensor_data/LiDAR_data.hpp"
#include "sensor_data/key_frame.hpp"
#include "sensor_data/pose_data.hpp"
#include "models/cloud_filter/voxel_filter.hpp"

namespace localization {
class Viewer {
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Viewer();

    bool Update(std::deque<KeyFrame,Eigen::aligned_allocator<KeyFrame>>& new_key_frames,
                std::deque<KeyFrame,Eigen::aligned_allocator<KeyFrame>>& optimized_key_frames,
                PoseData transformed_data,
                CloudData<PointXYZIRT> cloud_data);

    bool SaveMap();
    Eigen::Matrix4f& GetCurrentPose();
    CloudData<PointXYZIRT>::CLOUD_PTR& GetCurrentScan();
    bool GetLocalMap(CloudData<PointXYZIRT>::CLOUD_PTR& local_map_ptr);
    bool GetGlobalMap(CloudData<PointXYZIRT>::CLOUD_PTR& local_map_ptr);
    bool HasNewLocalMap();
    bool HasNewGlobalMap();

  private:
    bool InitWithConfig();
    bool InitParam(const YAML::Node& config_node);
    bool InitDataPath(const YAML::Node& config_node);
    bool InitFilter(std::string filter_user, 
                    std::shared_ptr<CloudFilterInterface>& filter_ptr, 
                    const YAML::Node& config_node);

    void ResetParam();
    bool OptimizeKeyFrames();
    bool JointGlobalMap(CloudData<PointXYZIRT>::CLOUD_PTR& global_map_ptr);
    bool JointLocalMap(CloudData<PointXYZIRT>::CLOUD_PTR& local_map_ptr);
    bool JointCloudMap(const std::deque<KeyFrame,Eigen::aligned_allocator<KeyFrame>>& key_frames, 
                             CloudData<PointXYZIRT>::CLOUD_PTR& map_cloud_ptr);

  private:
    std::string data_path_ = "";
    int local_frame_num_ = 20;

    std::string key_frames_path_ = "";
    std::string map_path_ = "";

    std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;

    Eigen::Matrix4f pose_to_optimize_ = Eigen::Matrix4f::Identity();
    PoseData optimized_odom_;
    CloudData<PointXYZIRT> optimized_cloud_;
    std::deque<KeyFrame,Eigen::aligned_allocator<KeyFrame>> optimized_key_frames_;
    std::deque<KeyFrame,Eigen::aligned_allocator<KeyFrame>> all_key_frames_;

    bool has_new_global_map_ = false;
    bool has_new_local_map_ = false;
};
}

#endif