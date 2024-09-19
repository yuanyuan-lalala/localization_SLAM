#ifndef LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_VOXEL_FILTER_HPP_
#define LIDAR_LOCALIZATION_MODELS_CLOUD_FILTER_VOXEL_FILTER_HPP_

#include <pcl/filters/voxel_grid.h>
#include "models/cloud_filter/cloud_filter_interface.hpp"

namespace localization {
class VoxelFilter: public CloudFilterInterface {
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    VoxelFilter(const YAML::Node& node);
    VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);

    bool Filter(const CloudData<PointXYZIRT>::CLOUD_PTR& input_cloud_ptr, CloudData<PointXYZIRT>::CLOUD_PTR& filtered_cloud_ptr) override;

  private:
    bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);

  private:
    pcl::VoxelGrid<PointXYZIRT> voxel_filter_;
};
}
#endif