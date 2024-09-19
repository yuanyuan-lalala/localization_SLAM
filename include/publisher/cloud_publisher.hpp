#ifndef LIDAR_LOCALIZATION_PUBLISHER_CLOUD_PUBLISHER_HPP_
#define LIDAR_LOCALIZATION_PUBLISHER_CLOUD_PUBLISHER_HPP_

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "sensor_data/LiDAR_data.hpp"
#include"global_settings/global_settings.hpp"
#include <ros/time.h>


namespace localization {
class CloudPublisher {
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    CloudPublisher(ros::NodeHandle& nh,
                               std::string topic_name,
                               std::string frame_id,
                               size_t buff_size);
    
    CloudPublisher() = default;
    
    void Publish(const CloudData<PointXYZIRT>::CLOUD_PTR& cloud_ptr_input);
    void Publish(const CloudData<PointXYZIRT>::CLOUD_PTR&  cloud_ptr_input, double time);
    bool HasSubscribers();
  
  private:

    void PublishData(const CloudData<PointXYZIRT>::CLOUD_PTR& cloud_ptr_input, ros::Time time);
    
    
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;
};
} 
#endif