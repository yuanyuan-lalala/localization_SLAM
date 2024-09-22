#ifndef LIDAR_LOCALIZATION_FRONT_END_FRONT_END_HPP_
#define LIDAR_LOCALIZATION_FRONT_END_FRONT_END_HPP_

#include <deque>

#include <Eigen/Dense>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/ndt.h>
#include <yaml-cpp/yaml.h>

#include "sensor_data/LiDAR_data.hpp"
#include"sensor_data/IMU_data.hpp"
#include "models/registration/ndt_registration.hpp"
#include "models/cloud_filter/voxel_filter.hpp"
#include <boost/make_shared.hpp>
namespace localization {
class FrontEnd {
  public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 保证内存对齐
    struct Frame { 
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        CloudData<PointXYZIRT> cloud_data;
        // EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 保证内存对齐
    };

  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
    FrontEnd();

    bool InitWithConfig();
    
    bool Update(const CloudData<PointXYZIRT>& cloud_data, Eigen::Matrix4f& cloud_pose);
    bool SetInitPose(const Eigen::Matrix4f& init_pose);

    // bool SaveMap();
    // bool GetNewLocalMap(CloudData<PointXYZIRT>::CLOUD_PTR& local_map_ptr);
    // bool GetNewGlobalMap(CloudData<PointXYZIRT>::CLOUD_PTR& global_map_ptr);
    // bool GetCurrentScan(CloudData<PointXYZIRT>::CLOUD_PTR& current_scan_ptr);
    bool IsInitialized() const ;
  
  private:
    
    bool InitDataPath(const YAML::Node& config_node);
    
    bool InitParam(const YAML::Node& config_node);
    bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);
    bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node);
    bool UpdateWithNewFrame(const Frame& new_key_frame);
   
  private:
    std::string data_path_ = "";

    std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
    // std::shared_ptr<CloudFilterInterface> display_filter_ptr_;
    std::shared_ptr<RegistrationInterface> registration_ptr_; 

    std::deque<Frame,Eigen::aligned_allocator<Frame>> local_map_frames_;
    // std::deque<Frame,Eigen::aligned_allocator<Frame>> global_map_frames_;

    // bool has_new_local_map_ = false;
    // bool has_new_global_map_ = false;
    bool initialized_; //add

    CloudData<PointXYZIRT>::CLOUD_PTR local_map_ptr_;
    // CloudData<PointXYZIRT>::CLOUD_PTR global_map_ptr_;
    // CloudData<PointXYZIRT>::CLOUD_PTR result_cloud_ptr_;
    Frame current_frame_;

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();

    float key_frame_distance_ = 2.0;
    int local_frame_num_ = 20;
};
}

#endif