#include <ros/ros.h>
#include "glog/logging.h"
#include "global_settings/global_settings.hpp"
#include "front_end/front_end_flow.hpp"
#include"save_map/save_map.h"


using namespace localization;

std::shared_ptr<FrontEndFlow> _front_end_flow_ptr;

bool save_map_callback(localization_slam::save_map::Request &request, localization_slam::save_map::Response &response) {
    
    response.success = _front_end_flow_ptr->SaveMap();
    _front_end_flow_ptr->PublishGlobalMap();
    return response.success;
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "front_end_node");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("save_map", save_map_callback);
    _front_end_flow_ptr = std::shared_ptr<FrontEndFlow>(new FrontEndFlow(nh));
    // _front_end_flow_ptr = std::make_shared<FrontEndFlow>(nh);

    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        _front_end_flow_ptr->Run();

        rate.sleep();
    }

    return 0;
}