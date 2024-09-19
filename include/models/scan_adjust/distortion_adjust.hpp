#ifndef LIDAR_LOCALIZATION_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_HPP_
#define LIDAR_LOCALIZATION_MODELS_SCAN_ADJUST_DISTORTION_ADJUST_HPP_

#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include "glog/logging.h"


#include "sensor_data/velocity_data.hpp"
#include "sensor_data/LiDAR_data.hpp"

namespace localization {
class DistortionAdjust {
  public:
    void SetMotionInfo(float scan_period, VelocityData velocity_data);
    bool AdjustCloud(const CloudData<PointXYZIRT>::CLOUD_PTR& input_cloud_ptr, const CloudData<PointXYZIRT>::CLOUD_PTR& output_cloud_ptr);

  private:
    inline Eigen::Matrix3f UpdateMatrix(float real_time);

  private:
    float scan_period_;
    Eigen::Vector3f velocity_;
    Eigen::Vector3f angular_rate_;
};
} 
#endif