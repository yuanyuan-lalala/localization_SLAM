#include "mapping/front_end/front_end_flow.hpp"
#include "glog/logging.h"
#include "global_settings/global_settings.hpp"

namespace localization {
FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh) {
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, "synced_cloud", 100000);
    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, "laser_odom", "map", "lidar", 100);
    front_end_ptr_ = std::allocate_shared<FrontEnd>(Eigen::aligned_allocator<FrontEnd>());

    // front_end_ptr_ = std::make_shared<FrontEnd>();
}

bool FrontEndFlow::Run() {
    if (!ReadData())
        return false;

    while(HasData()) {
        if (!ValidData())
            continue;

        if (UpdateLaserOdometry()) {
            PublishData();
        }
    }

    return true;
}

bool FrontEndFlow::ReadData() {
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    return true;
}

bool FrontEndFlow::HasData() {
    return cloud_data_buff_.size() > 0;
}

bool FrontEndFlow::ValidData() {
    current_cloud_data_ = cloud_data_buff_.front();
    cloud_data_buff_.pop_front();

    return true;
}

bool FrontEndFlow::UpdateLaserOdometry() {
    static bool odometry_inited = false;
    if (!odometry_inited) {
        odometry_inited = true;
        front_end_ptr_->SetInitPose(Eigen::Matrix4f::Identity());
        return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
    }

    return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
}

bool FrontEndFlow::PublishData() {
    
    laser_odom_pub_ptr_->Publish(laser_odometry_, current_cloud_data_.GetTime());

    return true;
}
}