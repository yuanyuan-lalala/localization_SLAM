#include "models/registration/ndt_registration.hpp"

#include "glog/logging.h"

namespace localization {

NDTRegistration::NDTRegistration(const YAML::Node& node)
    :ndt_ptr_(new pcl::NormalDistributionsTransform<PointXYZIRT, PointXYZIRT>()) {
    
    float res = node["res"].as<float>();
    float step_size = node["step_size"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

NDTRegistration::NDTRegistration(float res, float step_size, float trans_eps, int max_iter)
    :ndt_ptr_(new pcl::NormalDistributionsTransform<PointXYZIRT, PointXYZIRT>()) {

    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

bool NDTRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter) {
    ndt_ptr_->setResolution(res);
    ndt_ptr_->setStepSize(step_size);
    ndt_ptr_->setTransformationEpsilon(trans_eps);
    ndt_ptr_->setMaximumIterations(max_iter);

    LOG(INFO) << "NDT 的匹配参数为：" << std::endl
              << "res: " << res << ", "
              << "step_size: " << step_size << ", "
              << "trans_eps: " << trans_eps << ", "
              << "max_iter: " << max_iter 
              << std::endl << std::endl;

    return true;
}

bool NDTRegistration::SetInputTarget(const CloudData<PointXYZIRT>::CLOUD_PTR& input_target) {
    ndt_ptr_->setInputTarget(input_target);

    return true;
}

bool NDTRegistration::ScanMatch(const CloudData<PointXYZIRT>::CLOUD_PTR& input_source, 
                                const Eigen::Matrix4f& predict_pose, 
                                CloudData<PointXYZIRT>::CLOUD_PTR& result_cloud_ptr,
                                Eigen::Matrix4f& result_pose) {
    ndt_ptr_->setInputSource(input_source);
    ndt_ptr_->align(*result_cloud_ptr, predict_pose);
    result_pose = ndt_ptr_->getFinalTransformation();

    return true;
}
}