#include <ros/ros.h>
#include "glog/logging.h"

#include "global_settings/global_settings.hpp"
#include "mapping/back_end/back_end_flow.hpp"

using namespace localization;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "back_end_node");
    ros::NodeHandle nh;

    // std::shared_ptr<BackEndFlow> back_end_flow_ptr = std::make_shared<BackEndFlow>(nh);
    auto back_end_flow_ptr = std::allocate_shared<localization::BackEndFlow>(
    Eigen::aligned_allocator<localization::BackEndFlow>(), nh);
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        back_end_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}