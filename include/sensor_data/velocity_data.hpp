#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_VELOCITY_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_VELOCITY_DATA_HPP_

#include <deque>
#include <Eigen/Dense>

namespace localization {
class VelocityData {
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    struct LinearVelocity {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    struct AngularVelocity {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    double time = 0.0;
    LinearVelocity linear_velocity;
    AngularVelocity angular_velocity;
  
  public:
    static bool SyncData(std::deque<VelocityData,Eigen::aligned_allocator<VelocityData>>& UnsyncedData, std::deque<VelocityData,Eigen::aligned_allocator<VelocityData>>& SyncedData, double sync_time);
    void TransformCoordinate(Eigen::Matrix4f transform_matrix);

};
}
#endif