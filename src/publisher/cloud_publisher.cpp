#include "publisher/cloud_publisher.hpp"
#include "glog/logging.h"

namespace localization {
CloudPublisher::CloudPublisher(ros::NodeHandle& nh,
                               std::string topic_name,
                               std::string frame_id,
                               size_t buff_size)
    :nh_(nh), frame_id_(frame_id) {
    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
}

void CloudPublisher::Publish(const CloudData<PointXYZIRT>::CLOUD_PTR& cloud_ptr_input, double time) {
    
    ros::Time ros_time = ros::Time(time);
    PublishData(cloud_ptr_input, ros_time);

}

void CloudPublisher::Publish(const CloudData<PointXYZIRT>::CLOUD_PTR& cloud_ptr_input) {
    ros::Time time = ros::Time::now();
    PublishData(cloud_ptr_input, time);
}


void CloudPublisher::PublishData(const CloudData<PointXYZIRT>::CLOUD_PTR&  cloud_ptr_input, ros::Time time) {
    // 检查点云的宽度和高度是否正确匹配
    if (cloud_ptr_input->width * cloud_ptr_input->height != cloud_ptr_input->points.size()) {
        // 如果不匹配，调整宽度和高度
        cloud_ptr_input->width = cloud_ptr_input->points.size();
        cloud_ptr_input->height = 1;  // 处理为无组织点云
    }

    // 创建ROS格式的PointCloud2消息
    sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
    
    
    
    pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);

    cloud_ptr_output->header.stamp = time;
    cloud_ptr_output->header.frame_id = frame_id_;
    publisher_.publish(*cloud_ptr_output);
}

bool CloudPublisher::HasSubscribers() {
    return publisher_.getNumSubscribers() != 0;
}
} // namespace lidar_localization