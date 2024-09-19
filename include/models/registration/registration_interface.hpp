#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "sensor_data/LiDAR_data.hpp"

namespace localization {
class RegistrationInterface {
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~RegistrationInterface() = default;

    virtual bool SetInputTarget(const CloudData<PointXYZIRT>::CLOUD_PTR& input_target) = 0;
    virtual bool ScanMatch(const CloudData<PointXYZIRT>::CLOUD_PTR& input_source, 
                          const Eigen::Matrix4f& predict_pose, 
                          CloudData<PointXYZIRT>::CLOUD_PTR& result_cloud_ptr,
                          Eigen::Matrix4f& result_pose) = 0;
};
} 

#endif