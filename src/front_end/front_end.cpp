#include "front_end/front_end.hpp"

#include <cmath>
#include <pcl/common/transforms.h>
#include "glog/logging.h"
#include"memory"
namespace localization {
FrontEnd::FrontEnd()
    :
    ndt_ptr_(new pcl::NormalDistributionsTransform<PointXYZIRT, PointXYZIRT>()),
    // ndt_ptr_(std::make_shared<pcl::NormalDistributionsTransform<PointXYZIRT, PointXYZIRT>>()),
    local_map_ptr_(new CloudData<PointXYZIRT>::CLOUD()),
    global_map_ptr_(new CloudData<PointXYZIRT>::CLOUD()),
    result_cloud_ptr_(new CloudData<PointXYZIRT>::CLOUD()) {

    // 给个默认参数，以免类的使用者在匹配之前忘了设置参数 要设置降采样和ndt的参数
    cloud_filter_.setLeafSize(1.3, 1.3, 1.3);
    local_map_filter_.setLeafSize(0.6, 0.6, 0.6);
    display_filter_.setLeafSize(0.5, 0.5, 0.5);
    ndt_ptr_->setResolution(1.0);
    ndt_ptr_->setStepSize(0.1);
    ndt_ptr_->setTransformationEpsilon(0.01);
    ndt_ptr_->setMaximumIterations(30);
}
// FrontEnd::FrontEnd()
// :
// ndt_ptr_(std::make_shared<pcl::NormalDistributionsTransform<PointXYZIRT, PointXYZIRT>>()),
// local_map_ptr_(std::make_shared<CloudData<PointXYZIRT>::CLOUD>()),
// global_map_ptr_(std::make_shared<CloudData<PointXYZIRT>::CLOUD>()),
// result_cloud_ptr_(std::make_shared<CloudData<PointXYZIRT>::CLOUD>()) {
//     // 设置默认参数
//     cloud_filter_.setLeafSize(1.3, 1.3, 1.3);
//     local_map_filter_.setLeafSize(0.6, 0.6, 0.6);
//     display_filter_.setLeafSize(0.5, 0.5, 0.5);
//     ndt_ptr_->setResolution(1.0);
//     ndt_ptr_->setStepSize(0.1);
//     ndt_ptr_->setTransformationEpsilon(0.01);
//     ndt_ptr_->setMaximumIterations(30);
// }

Eigen::Matrix4f FrontEnd::Update(const CloudData<PointXYZIRT>& cloud_data) {
    // current_frame_.cloud_data.time = cloud_data.time;
    current_frame_.cloud_data.SetTime(cloud_data.GetTime()); 
    std::vector<int> indices;
    //删除无效点放入当前帧
    pcl::removeNaNFromPointCloud(*cloud_data.GetCloudData(), *current_frame_.cloud_data.GetCloudData(), indices);

    //对当前帧的点云进行降采样
    CloudData<PointXYZIRT>::CLOUD_PTR filtered_cloud_ptr(new CloudData<PointXYZIRT>::CLOUD());
    cloud_filter_.setInputCloud(current_frame_.cloud_data.GetCloudData());
    cloud_filter_.filter(*filtered_cloud_ptr);

    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = init_pose_;

    // 局部地图容器中没有关键帧，代表是第一帧数据
    // 此时把当前帧数据作为第一个关键帧，并更新局部地图容器和全局地图容器
    if (local_map_frames_.size() == 0) {
        current_frame_.pose = init_pose_;//当前帧为第一帧
        UpdateNewFrame(current_frame_);
        return current_frame_.pose;
    }

    // 不是第一帧，就正常匹配
    ndt_ptr_->setInputSource(filtered_cloud_ptr);
    ndt_ptr_->align(*result_cloud_ptr_, predict_pose);
    current_frame_.pose = ndt_ptr_->getFinalTransformation();

    // 更新相邻两帧的相对运动
    step_pose = last_pose.inverse() * current_frame_.pose;
    predict_pose = current_frame_.pose * step_pose;
    last_pose = current_frame_.pose;

    // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新
    if (fabs(last_key_frame_pose(0,3) - current_frame_.pose(0,3)) + 
        fabs(last_key_frame_pose(1,3) - current_frame_.pose(1,3)) +
        fabs(last_key_frame_pose(2,3) - current_frame_.pose(2,3)) > 2.0) {
        UpdateNewFrame(current_frame_);
        last_key_frame_pose = current_frame_.pose;
    }

    return current_frame_.pose;
}

bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose) {
    init_pose_ = init_pose;
    return true;
}

bool FrontEnd::SetPredictPose(const Eigen::Matrix4f& predict_pose) {
    predict_pose_ = predict_pose;
    return true;
}

void FrontEnd::UpdateNewFrame(const Frame& new_key_frame) {
    Frame key_frame = new_key_frame;
    // 这一步的目的是为了把关键帧的点云保存下来
    // 由于用的是共享指针，所以直接复制只是复制了一个指针而已
    // 此时无论你放多少个关键帧在容器里，这些关键帧点云指针都是指向的同一个点云
    key_frame.cloud_data.GetCloudData().reset(new CloudData<PointXYZIRT>::CLOUD(*new_key_frame.cloud_data.GetCloudData()));
    CloudData<PointXYZIRT>::CLOUD_PTR transformed_cloud_ptr(new CloudData<PointXYZIRT>::CLOUD());
    
    // 更新局部地图 20个关键帧构成一个局部地图
    local_map_frames_.push_back(key_frame);
    while (local_map_frames_.size() > 20) {
        local_map_frames_.pop_front();
    }
    local_map_ptr_.reset(new CloudData<PointXYZIRT>::CLOUD());
    //进行地图拼接
    for (size_t i = 0; i < local_map_frames_.size(); ++i) {
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.GetCloudData(), 
                                 *transformed_cloud_ptr, 
                                 local_map_frames_.at(i).pose);
        *local_map_ptr_ += *transformed_cloud_ptr;
    }
    has_new_local_map_ = true;

    // 更新ndt匹配的目标点云
    if (local_map_frames_.size() < 10) {
        
        ndt_ptr_->setInputTarget(local_map_ptr_);
    
    } else {

        CloudData<PointXYZIRT>::CLOUD_PTR filtered_local_map_ptr(new CloudData<PointXYZIRT>::CLOUD());
        local_map_filter_.setInputCloud(local_map_ptr_);
        local_map_filter_.filter(*filtered_local_map_ptr);
        ndt_ptr_->setInputTarget(filtered_local_map_ptr);
    
    }

    // 更新全局地图
    global_map_frames_.push_back(key_frame);
    if (global_map_frames_.size() % 100 != 0) {
        return;
    } else {
        global_map_ptr_.reset(new CloudData<PointXYZIRT>::CLOUD());
        for (size_t i = 0; i < global_map_frames_.size(); ++i) {
            pcl::transformPointCloud(*global_map_frames_.at(i).cloud_data.GetCloudData(), 
                                    *transformed_cloud_ptr, 
                                    global_map_frames_.at(i).pose);
            *global_map_ptr_ += *transformed_cloud_ptr;
        }
        has_new_global_map_ = true;
    }
}

bool FrontEnd::GetNewLocalMap(CloudData<PointXYZIRT>::CLOUD_PTR& local_map_ptr) {
    if (has_new_local_map_) {
        display_filter_.setInputCloud(local_map_ptr_);
        display_filter_.filter(*local_map_ptr);
        return true;
    }
    return false;

}

bool FrontEnd::GetNewGlobalMap(CloudData<PointXYZIRT>::CLOUD_PTR& global_map_ptr) {
    if (has_new_global_map_) {

        display_filter_.setInputCloud(global_map_ptr_);
        display_filter_.filter(*global_map_ptr);
        return true;
    
    }
    return false;
}

bool FrontEnd::GetCurrentScan(CloudData<PointXYZIRT>::CLOUD_PTR& current_scan_ptr) {
    display_filter_.setInputCloud(result_cloud_ptr_);
    display_filter_.filter(*current_scan_ptr);
    return true;
}
}