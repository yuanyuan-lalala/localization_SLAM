
#ifndef LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_REGISTRATION_HPP_
#define LIDAR_LOCALIZATION_MODELS_REGISTRATION_NDT_REGISTRATION_HPP_

#include <pcl/registration/ndt.h>
#include "models/registration/registration_interface.hpp"

namespace localization {
class NDTRegistration: public RegistrationInterface {
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    NDTRegistration(const YAML::Node& node);
    NDTRegistration(float res, float step_size, float trans_eps, int max_iter);

    bool SetInputTarget(const CloudData<PointXYZIRT>::CLOUD_PTR& input_target) override;
    bool ScanMatch(const CloudData<PointXYZIRT>::CLOUD_PTR& input_source, 
                   const Eigen::Matrix4f& predict_pose, 
                   CloudData<PointXYZIRT>::CLOUD_PTR& result_cloud_ptr,
                   Eigen::Matrix4f& result_pose) override;
  
  private:
    bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

  private:
  pcl::NormalDistributionsTransform<PointXYZIRT, PointXYZIRT>::Ptr ndt_ptr_;

};
}

#endif