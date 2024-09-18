#ifndef LIDAR_LOCALIZATION_FRONT_END_FRONT_END_HPP_
#define LIDAR_LOCALIZATION_FRONT_END_FRONT_END_HPP_

#include <deque>

#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>

#include "sensor_data/LiDAR_data.hpp"
#include"global_settings/global_settings.hpp"
#include"memory"
#include <Eigen/Core>

namespace localization {


class FrontEnd {
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 保证内存对齐
    class Frame {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        CloudData<PointXYZIRT> cloud_data;
    };

  public:
    FrontEnd();

    Eigen::Matrix4f Update(const CloudData<PointXYZIRT>& cloud_data);
    bool SetInitPose(const Eigen::Matrix4f& init_pose);
    bool SetPredictPose(const Eigen::Matrix4f& predict_pose);

    bool GetNewLocalMap(CloudData<PointXYZIRT>::CLOUD_PTR& local_map_ptr);
    bool GetNewGlobalMap(CloudData<PointXYZIRT>::CLOUD_PTR& global_map_ptr);
    bool GetCurrentScan(CloudData<PointXYZIRT>::CLOUD_PTR& current_scan_ptr);
  
  private:
    void UpdateNewFrame(const Frame& new_key_frame);

  private:
    pcl::VoxelGrid<PointXYZIRT> cloud_filter_;
    pcl::VoxelGrid<PointXYZIRT> local_map_filter_;
    pcl::VoxelGrid<PointXYZIRT> display_filter_;
    
    
    
    // std::shared_ptr<pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>, 
                        // Eigen::aligned_allocator<pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>>> ndt_ptr_;
    pcl::NormalDistributionsTransform<PointXYZIRT, PointXYZIRT>::Ptr ndt_ptr_;

    std::deque<Frame,Eigen::aligned_allocator<Frame>> local_map_frames_;
    std::deque<Frame,Eigen::aligned_allocator<Frame>> global_map_frames_;

    bool has_new_local_map_ = false;
    bool has_new_global_map_ = false;
    CloudData<PointXYZIRT>::CLOUD_PTR local_map_ptr_;
    CloudData<PointXYZIRT>::CLOUD_PTR global_map_ptr_;
    CloudData<PointXYZIRT>::CLOUD_PTR result_cloud_ptr_;
    Frame current_frame_;

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f predict_pose_ = Eigen::Matrix4f::Identity();
};
}

#endif