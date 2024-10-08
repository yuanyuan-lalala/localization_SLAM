#include "mapping/viewer/viewer.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"
#include "tools/file_manager.hpp"


namespace localization {
Viewer::Viewer() {
    InitWithConfig();
}
bool Viewer::InitWithConfig() {
    std::string config_file_path = WORK_SPACE_PATH + "/config/viewer/config.yaml";
    std::cout << "Loading config file: " << config_file_path << std::endl;
    YAML::Node config_node;
    try {
        config_node = YAML::LoadFile(config_file_path);
    } catch (const YAML::Exception& e) {
        std::cerr << "Failed to load config file: " << e.what() << std::endl;
        return false;
    }

    if (!InitParam(config_node)) {
        std::cerr << "InitParam failed." << std::endl;
        return false;
    }
    if (!InitDataPath(config_node)) {
        std::cerr << "InitDataPath failed." << std::endl;
        return false;
    }
    if (!InitFilter("frame", frame_filter_ptr_, config_node)) {
        std::cerr << "InitFilter for frame failed." << std::endl;
        return false;
    }
    if (!InitFilter("local_map", local_map_filter_ptr_, config_node)) {
        std::cerr << "InitFilter for local_map failed." << std::endl;
        return false;
    }
    if (!InitFilter("global_map", global_map_filter_ptr_, config_node)) {
        std::cerr << "InitFilter for global_map failed." << std::endl;
        return false;
    }

    return true;
}

// bool Viewer::InitWithConfig() {
//     std::string config_file_path = WORK_SPACE_PATH + "/config/viewer/config.yaml";
//     YAML::Node config_node = YAML::LoadFile(config_file_path);

//     InitParam(config_node);
//     InitDataPath(config_node);
//     InitFilter("frame", frame_filter_ptr_, config_node);
//     InitFilter("local_map", local_map_filter_ptr_, config_node);
//     InitFilter("global_map", global_map_filter_ptr_, config_node);

//     return true;
// }

bool Viewer::InitParam(const YAML::Node& config_node) {
    local_frame_num_ = config_node["local_frame_num"].as<int>();
    return true;
}

bool Viewer::InitDataPath(const YAML::Node& config_node) {
    std::string data_path = config_node["data_path"].as<std::string>();
    if (data_path == "./") {
        data_path = WORK_SPACE_PATH;
    }

    key_frames_path_ = data_path + "/slam_data/key_frames";
    map_path_ = data_path + "/slam_data/map";

    if (!FileManager::InitDirectory(map_path_, "点云地图文件"))
        return false;

    return true;
}

bool Viewer::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();
    LOG(INFO) << "viewer_" + filter_user << "选择的滤波方法为：" << filter_mothod;

    if (filter_mothod == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
    } else {
        LOG(ERROR) << "没有为 " << filter_user << " 找到与 " << filter_mothod << " 相对应的滤波方法!";
        return false;
    }

    return true;
}

bool Viewer::Update(std::deque<KeyFrame,Eigen::aligned_allocator<KeyFrame>>& new_key_frames,
                    std::deque<KeyFrame,Eigen::aligned_allocator<KeyFrame>>& optimized_key_frames,
                    PoseData transformed_data,
                    CloudData<PointXYZIRT> cloud_data) {
    ResetParam();

    if (optimized_key_frames.size() > 0) {
        optimized_key_frames_ = optimized_key_frames;
        optimized_key_frames.clear();
        OptimizeKeyFrames();
        has_new_global_map_ = true;
    }

    if (new_key_frames.size()) {
        all_key_frames_.insert(all_key_frames_.end(), new_key_frames.begin(), new_key_frames.end());
        new_key_frames.clear();
        has_new_local_map_ = true;
    }

    optimized_odom_ = transformed_data;
    optimized_odom_.pose = pose_to_optimize_ * optimized_odom_.pose;

    optimized_cloud_ = cloud_data;
    pcl::transformPointCloud(*cloud_data.GetCloudData(), *optimized_cloud_.GetCloudData(), optimized_odom_.pose);

    return true;
}

void Viewer::ResetParam() {
    has_new_local_map_ = false;
    has_new_global_map_ = false;
}

bool Viewer::OptimizeKeyFrames() {
    size_t optimized_index = 0;
    size_t all_index = 0;
    while (optimized_index < optimized_key_frames_.size() && all_index < all_key_frames_.size()) {
        if (optimized_key_frames_.at(optimized_index).index < all_key_frames_.at(all_index).index) {
            optimized_index ++;
        } else if (optimized_key_frames_.at(optimized_index).index < all_key_frames_.at(all_index).index) {
            all_index ++;
        } else {
            pose_to_optimize_ = optimized_key_frames_.at(optimized_index).pose * all_key_frames_.at(all_index).pose.inverse();
            all_key_frames_.at(all_index) = optimized_key_frames_.at(optimized_index);
            optimized_index ++;
            all_index ++;
        }
    }

    while (all_index < all_key_frames_.size()) {
        all_key_frames_.at(all_index).pose = pose_to_optimize_ * all_key_frames_.at(all_index).pose;
        all_index ++;
    }

    return true;
}

bool Viewer::JointGlobalMap(CloudData<PointXYZIRT>::CLOUD_PTR& global_map_ptr) {
    JointCloudMap(optimized_key_frames_, global_map_ptr);
    return true;
}

bool Viewer::JointLocalMap(CloudData<PointXYZIRT>::CLOUD_PTR& local_map_ptr) {
    size_t begin_index = 0;
    if (all_key_frames_.size() > (size_t)local_frame_num_)
        begin_index = all_key_frames_.size() - (size_t)local_frame_num_;

    std::deque<KeyFrame,Eigen::aligned_allocator<KeyFrame>> local_key_frames;
    for (size_t i = begin_index; i < all_key_frames_.size(); ++i)
        local_key_frames.push_back(all_key_frames_.at(i));

    JointCloudMap(local_key_frames, local_map_ptr);
    return true;
}

bool Viewer::JointCloudMap(const std::deque<KeyFrame,Eigen::aligned_allocator<KeyFrame>>& key_frames, CloudData<PointXYZIRT>::CLOUD_PTR& map_cloud_ptr) {
    map_cloud_ptr.reset(new CloudData<PointXYZIRT>::CLOUD());

    CloudData<PointXYZIRT>::CLOUD_PTR cloud_ptr(new CloudData<PointXYZIRT>::CLOUD());
    std::string file_path = "";

    for (size_t i = 0; i < key_frames.size(); ++i) {
        file_path = key_frames_path_ + "/key_frame_" + std::to_string(key_frames.at(i).index) + ".pcd";
        pcl::io::loadPCDFile(file_path, *cloud_ptr);
        pcl::transformPointCloud(*cloud_ptr, *cloud_ptr, key_frames.at(i).pose);
        *map_cloud_ptr += *cloud_ptr;
    }
    return true;
}

bool Viewer::SaveMap() {
    if (optimized_key_frames_.size() == 0)
        return false;

    CloudData<PointXYZIRT>::CLOUD_PTR global_map_ptr(new CloudData<PointXYZIRT>::CLOUD());
    JointCloudMap(optimized_key_frames_, global_map_ptr);

    std::string map_file_path = map_path_ + "/map.pcd";
    pcl::io::savePCDFileBinary(map_file_path, *global_map_ptr);

    LOG(INFO) << "地图保存完成，地址是：" << std::endl << map_file_path << std::endl << std::endl;

    return true;
}

Eigen::Matrix4f& Viewer::GetCurrentPose() {
    return optimized_odom_.pose;
}

//内存不会泄露？
CloudData<PointXYZIRT>::CLOUD_PTR& Viewer::GetCurrentScan() {
    frame_filter_ptr_->Filter(optimized_cloud_.cloud_ptr_, optimized_cloud_.cloud_ptr_);
    return optimized_cloud_.cloud_ptr_;
}

bool Viewer::GetLocalMap(CloudData<PointXYZIRT>::CLOUD_PTR& local_map_ptr) {
    JointLocalMap(local_map_ptr);
    local_map_filter_ptr_->Filter(local_map_ptr, local_map_ptr);
    return true;
}

bool Viewer::GetGlobalMap(CloudData<PointXYZIRT>::CLOUD_PTR& global_map_ptr) {
    JointGlobalMap(global_map_ptr);
    global_map_filter_ptr_->Filter(global_map_ptr, global_map_ptr);
    return true;
}

bool Viewer::HasNewLocalMap() {
    return has_new_local_map_;
}

bool Viewer::HasNewGlobalMap() {
    return has_new_global_map_;
}
}