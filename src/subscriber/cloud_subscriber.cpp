/*
 * @Description: 订阅激光点云信息，并解析数据
 * @Author: Ren Qian
 * @Date: 2020-02-05 02:27:30
 */

#include "subscriber/cloud_subscriber.hpp"

#include "glog/logging.h"

namespace localization {
CloudSubscriber::CloudSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this);
}

void CloudSubscriber::msg_callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg_ptr) {
    CloudData<PointXYZIRT> cloud_data;
    cloud_data.SetTime(cloud_msg_ptr->header.stamp.toSec());
    pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.GetCloudData()));
    new_cloud_data_.push_back(cloud_data);
}

void CloudSubscriber::ParseData(std::deque<CloudData<PointXYZIRT>>& cloud_data_buff) {
    if (new_cloud_data_.size() > 0) {
        cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end());
        new_cloud_data_.clear();
    }
}
} // namespace data_input