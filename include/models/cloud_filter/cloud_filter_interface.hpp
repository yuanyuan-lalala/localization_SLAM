#ifndef LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_
#define LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_CLOUD_FILTER_INTERFACE_HPP_

#include <yaml-cpp/yaml.h>
#include "sensor_data/LiDAR_data.hpp"

namespace localization {
class CloudFilterInterface {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  public:
    virtual ~CloudFilterInterface() = default;

    virtual bool Filter(const CloudData<PointXYZIRT>::CLOUD_PTR& input_cloud_ptr, CloudData<PointXYZIRT>::CLOUD_PTR& filtered_cloud_ptr) = 0;
};
}

#endif