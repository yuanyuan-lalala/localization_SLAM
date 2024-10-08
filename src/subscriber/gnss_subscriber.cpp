#include "subscriber/gnss_subscriber.hpp"

#include "glog/logging.h"

namespace localization {

GNSSSubscriber::GNSSSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size) 
    :nh_(nh) {

    subscriber_ = nh_.subscribe(topic_name, buff_size, &GNSSSubscriber::msg_callback, this);
}

void GNSSSubscriber::msg_callback(const sensor_msgs::NavSatFixConstPtr& nav_sat_fix_ptr) {
    GNSSData gnss_data;
    gnss_data.time = nav_sat_fix_ptr->header.stamp.toSec();
    gnss_data.latitude = nav_sat_fix_ptr->latitude;
    gnss_data.longitude = nav_sat_fix_ptr->longitude;
    gnss_data.altitude = nav_sat_fix_ptr->longitude;
    gnss_data.status = nav_sat_fix_ptr->status.status;
    gnss_data.service = nav_sat_fix_ptr->status.service;

    new_gnss_data_.push_back(gnss_data);
}
// void GNSSSubscriber::msg_callback(const nmea_msgs::SentenceConstPtr& nmea_msg) {
//     GNSSData gnss_data;
//     // 在这里解析 NMEA 句子
//     std::string sentence = nmea_msg->sentence;

//     // 解析NMEA句子
//     if (gnss_data.ParseFromNMEA(sentence)) {
//         new_gnss_data_.push_back(gnss_data);
//     }
// }


void GNSSSubscriber::ParseData(std::deque<GNSSData,Eigen::aligned_allocator<GNSSData>>& gnss_data_buff) {
    if (new_gnss_data_.size() > 0) {
        gnss_data_buff.insert(gnss_data_buff.end(), new_gnss_data_.begin(), new_gnss_data_.end());
        new_gnss_data_.clear();
    }
}




}