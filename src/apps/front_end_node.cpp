#include <ros/ros.h>
#include "glog/logging.h"

#include "global_settings/global_settings.hpp"
#include "mapping/front_end/front_end_flow.hpp"

using namespace localization;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh;

    // std::shared_ptr<FrontEndFlow> front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh);
    auto front_end_flow_ptr = std::allocate_shared<localization::FrontEndFlow>(
    Eigen::aligned_allocator<localization::FrontEndFlow>(), nh);
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        front_end_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}