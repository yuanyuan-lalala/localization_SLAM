#include "mapping/back_end/back_end_flow.hpp"

#include "glog/logging.h"

#include "tools/file_manager.hpp"
#include "global_settings/global_settings.hpp"

namespace localization {
BackEndFlow::BackEndFlow(ros::NodeHandle& nh) {
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "synced_cloud", 100000);
    gnss_pose_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "synced_gnss", 100000);
    laser_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "laser_odom", 100000);

    transformed_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "transformed_odom", "map", "lidar", 100);
    key_frame_pub_ptr_ = std::make_shared<KeyFramePublisher>(nh, "key_frame", "map", 100);
    key_frames_pub_ptr_ = std::make_shared<KeyFramesPublisher>(nh, "optimized_key_frames", "map", 100);
    back_end_ptr_ = std::allocate_shared<BackEnd>(Eigen::aligned_allocator<BackEnd>());
    // back_end_ptr_ = std::make_shared<BackEnd>();
}

bool BackEndFlow::Run() {
    if (!ReadData())
        return false;

    while(HasData()) {
        if (!ValidData())
            continue;

        UpdateBackEnd();

        PublishData();
    }

    return true;
}

bool BackEndFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    gnss_pose_sub_ptr_->ParseData(gnss_pose_data_buff_);
    laser_odom_sub_ptr_->ParseData(laser_odom_data_buff_);

    return true;
}

bool BackEndFlow::HasData() {
    if (cloud_data_buff_.size() == 0)
        return false;
    if (gnss_pose_data_buff_.size() == 0)
        return false;
    if (laser_odom_data_buff_.size() == 0)
        return false;

    return true;
}

bool BackEndFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    current_gnss_pose_data_ = gnss_pose_data_buff_.front();
    current_laser_odom_data_ = laser_odom_data_buff_.front();
    //雷达和gnss时间差、雷达和雷达里程计的时间差
    double diff_gnss_time = current_cloud_data_.GetTime() - current_gnss_pose_data_.time;
    double diff_laser_time = current_cloud_data_.GetTime() - current_laser_odom_data_.time;
    //雷达点云时间不能在gnss和里程计前
    if (diff_gnss_time < -0.05 || diff_laser_time < -0.05) {
        cloud_data_buff_.pop_front();
        return false;
    }

    if (diff_gnss_time > 0.05) {
        gnss_pose_data_buff_.pop_front();
        return false;
    }

    if (diff_laser_time > 0.05) {
        laser_odom_data_buff_.pop_front();
        return false;
    }

    cloud_data_buff_.pop_front();
    gnss_pose_data_buff_.pop_front();
    laser_odom_data_buff_.pop_front();

    return true;
}

bool BackEndFlow::UpdateBackEnd() {
    static bool odometry_inited = false;
    static Eigen::Matrix4f odom_init_pose = Eigen::Matrix4f::Identity();

    if (!odometry_inited) {
        odometry_inited = true;
        //计算雷达里程计坐标系到 GNSS 坐标系的初始变换矩阵，作为初始的对齐。
        odom_init_pose = current_gnss_pose_data_.pose * current_laser_odom_data_.pose.inverse();
    }
    //统一变换到GNSS系下
    current_laser_odom_data_.pose = odom_init_pose * current_laser_odom_data_.pose;

    return back_end_ptr_->Update(current_cloud_data_, current_laser_odom_data_, current_gnss_pose_data_);
}

bool BackEndFlow::PublishData() {
    //里程计发布
    transformed_odom_pub_ptr_->Publish(current_laser_odom_data_.pose, current_laser_odom_data_.time);

    if (back_end_ptr_->HasNewKeyFrame()) {
        KeyFrame key_frame;
        back_end_ptr_->GetLatestKeyFrame(key_frame);
        key_frame_pub_ptr_->Publish(key_frame);
    }

    if (back_end_ptr_->HasNewOptimized()) {
        std::deque<KeyFrame,Eigen::aligned_allocator<KeyFrame>> optimized_key_frames;
        back_end_ptr_->GetOptimizedKeyFrames(optimized_key_frames);
        key_frames_pub_ptr_->Publish(optimized_key_frames);
    }

    return true;
}
}