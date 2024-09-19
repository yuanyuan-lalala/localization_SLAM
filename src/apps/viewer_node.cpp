
#include <ros/ros.h>
#include "glog/logging.h"
#include"saveMap.h"
#include "global_settings/global_settings.hpp"
#include "mapping/viewer/viewer_flow.hpp"

using namespace localization;

std::shared_ptr<ViewerFlow> _viewer_flow_ptr;
bool _need_save_map = false;

bool save_map_callback(localization::saveMap::Request &request, localization::saveMap::Response &response) {
    _need_save_map = true;
    response.success = true;
    return response.success;
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "viewer_node");
    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("save_map", save_map_callback);
    // std::shared_ptr<ViewerFlow> _viewer_flow_ptr = std::make_shared<ViewerFlow>(nh);
    auto _viewer_flow_ptr= std::allocate_shared<localization::ViewerFlow>(
    Eigen::aligned_allocator<localization::ViewerFlow>(), nh);
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();

        _viewer_flow_ptr->Run();
        if (_need_save_map) {
            _need_save_map = false;
            _viewer_flow_ptr->SaveMap();
        }

        rate.sleep();
    }

    return 0;
}