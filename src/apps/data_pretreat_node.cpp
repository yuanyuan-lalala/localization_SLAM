#include <ros/ros.h>
#include "glog/logging.h"

#include "global_settings/global_settings.hpp"
#include "data_pretreat/data_pretreat_flow.hpp"

using namespace localization;

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "data_pretreat_node");
    ros::NodeHandle nh;
    auto data_pretreat_flow_ptr = std::allocate_shared<localization::DataPretreatFlow>(
    Eigen::aligned_allocator<localization::DataPretreatFlow>(), nh);


    // std::shared_ptr<DataPretreatFlow> data_pretreat_flow_ptr = std::make_shared<DataPretreatFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        data_pretreat_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}