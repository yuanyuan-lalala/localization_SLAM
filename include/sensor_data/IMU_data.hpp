#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_IMU_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_IMU_DATA_HPP_

#include <Eigen/Dense>
#include"global_settings/global_settings.hpp"
#include"deque"
namespace localization {
class IMUData {
  public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    struct LinearAcceleration {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };

    struct AngularVelocity {
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
    };
     
    class Orientation {
      public:
      double x = 0.0;
      double y = 0.0;
      double z = 0.0;
      double w = 0.0;
      public:
      void Normlize() {
          double norm = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0));
          x /= norm;
          y /= norm;
          z /= norm;
          w /= norm;
        }
    };

    double time = 0.0;
    LinearAcceleration linear_acceleration;
    AngularVelocity angular_velocity;
    Orientation orientation;

    static bool SyncData(std::deque<IMUData,Eigen::aligned_allocator<IMUData>> & UnsyncedData, std::deque<IMUData,Eigen::aligned_allocator<IMUData>>& SyncedData, double sync_time);
  public:
    // 把四元数转换成旋转矩阵送出去
      Eigen::Matrix3f GetOrientationMatrix() ;
};
}
#endif