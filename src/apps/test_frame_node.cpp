#include "ros/ros.h"
#include <pcl/common/transforms.h>
#include "glog/logging.h"

// #include "lidar_localization/global_defination/global_defination.h"
#include "publisher/cloud_publisher.hpp"
#include "publisher/odometry_publisher.hpp"
#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/imu_subscriber.hpp"
#include "subscriber/gnss_subscriber.hpp"
#include "tf_listener/tf_listener.hpp"
// #include "lio_sam/cloud_info.h"
#include"global_settings/global_settings.hpp"
using namespace localization;

int main(int argc, char *argv[]) {
    
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "test_frame_node");
    ros::NodeHandle nh;

    // std::shared_ptr<CloudSubscriber> cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh, "/velodyne_points", 100000);
    // std::shared_ptr<IMUSubscriber> imu_sub_ptr = std::make_shared<IMUSubscriber>(nh, "/imu/data", 1000000);
    // std::shared_ptr<GNSSSubscriber> gnss_sub_ptr = std::make_shared<GNSSSubscriber>(nh, "/nmea_sentence", 1000000);
    // std::shared_ptr<TFListener> lidar_to_imu_ptr = std::make_shared<TFListener>(nh, "velo_link", "imu_link");
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr = std::make_shared<CloudSubscriber>(nh, "/kitti/velo/pointcloud", 100000);
    std::shared_ptr<IMUSubscriber> imu_sub_ptr = std::make_shared<IMUSubscriber>(nh, "/kitti/oxts/imu", 1000000);
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr = std::make_shared<GNSSSubscriber>(nh, "/kitti/ oxts/gps/fix", 1000000);
    //base + child 雷达和IMU
    std::shared_ptr<TFListener> lidar_to_imu_ptr = std::make_shared<TFListener>(nh, "velo_link", "imu_link");



    std::shared_ptr<CloudPublisher> cloud_pub_ptr = std::make_shared<CloudPublisher>(nh, "current_scan", "map", 100);
    std::shared_ptr<OdometryPublisher> odom_pub_ptr = std::make_shared<OdometryPublisher>(nh, "lidar_odom", "map", "lidar", 100);

    std::deque<CloudData<PointXYZIRT>,Eigen::aligned_allocator<CloudData<PointXYZIRT>>> cloud_data_buff;
    std::deque<IMUData,Eigen::aligned_allocator<IMUData>> imu_data_buff;
    std::deque<GNSSData,Eigen::aligned_allocator<GNSSData>> gnss_data_buff;
    Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();//此处外参可以更改
    bool transform_received = false;
    bool gnss_origin_position_inited = false;

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        cloud_sub_ptr->ParseData(cloud_data_buff);
        imu_sub_ptr->ParseData(imu_data_buff);
        gnss_sub_ptr->ParseData(gnss_data_buff);

        if (!transform_received) {
            if (lidar_to_imu_ptr->LookupData(lidar_to_imu)) {
                transform_received = true;
                // LOG(INFO) << "lidar to imu transform matrix is:" << std::endl << lidar_to_imu;
            }
        } else {
            while (cloud_data_buff.size() > 0 && imu_data_buff.size() > 0 && gnss_data_buff.size() > 0) {
                CloudData<PointXYZIRT> cloud_data = cloud_data_buff.front();
                IMUData imu_data = imu_data_buff.front();
                GNSSData gnss_data = gnss_data_buff.front();

                double d_time = cloud_data.GetTime() - imu_data.time;
                if (d_time < -0.05) {
                    cloud_data_buff.pop_front();
                } else if (d_time > 0.05) {
                    imu_data_buff.pop_front();
                    gnss_data_buff.pop_front();
                } else {
                    cloud_data_buff.pop_front();
                    imu_data_buff.pop_front();
                    gnss_data_buff.pop_front();

                    Eigen::Matrix4f odometry_matrix;

                    if (!gnss_origin_position_inited) {
                        gnss_data.InitOriginPosition();
                        gnss_origin_position_inited = true;
                    }
                    gnss_data.UpdateXYZ();
                    //仅仅使用GNSS和IMU进行SE3位姿发布
                    odometry_matrix(0,3) = gnss_data.local_E;
                    odometry_matrix(1,3) = gnss_data.local_N;
                    odometry_matrix(2,3) = gnss_data.local_U;
                    odometry_matrix.block<3,3>(0,0) = imu_data.GetOrientationMatrix();
                    odometry_matrix *= lidar_to_imu;

                    pcl::transformPointCloud(*cloud_data.GetCloudData(), *cloud_data.GetCloudData(), odometry_matrix);

                    cloud_pub_ptr->Publish(cloud_data.GetCloudData());
                    cloud_pub_ptr->Publish(cloud_data.GetCloudData());
                    odom_pub_ptr->Publish(odometry_matrix);
                }
            }
        }

        rate.sleep();
    }

    return 0;
}