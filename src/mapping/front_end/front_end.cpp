#include "mapping/front_end/front_end.hpp"

#include <fstream>
#include <boost/filesystem.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include "glog/logging.h"
#include "global_settings/global_settings.hpp"


namespace localization {

FrontEnd::FrontEnd()

    :local_map_ptr_(boost::allocate_shared<CloudData<PointXYZIRT>::CLOUD>(
          Eigen::aligned_allocator<CloudData<PointXYZIRT>::CLOUD>())), 
      initialized_(false) {  // 添加一个标志位

    if (!InitWithConfig()) {
        std::cerr << "FrontEnd initialization failed." << std::endl;
    } else {
        initialized_ = true;
    }
}



bool FrontEnd::InitWithConfig() {
    std::string config_file_path = WORK_SPACE_PATH + "/config/front_end/config.yaml";
    std::cout << "Config file path: " << config_file_path << std::endl;

    YAML::Node config_node;
    try {
        config_node = YAML::LoadFile(config_file_path);
    } catch (const YAML::Exception& e) {
        std::cerr << "Failed to load configuration file: " << config_file_path << std::endl;
        std::cerr << "Exception: " << e.what() << std::endl;
        return false;
    }

    // 检查配置节点是否有效
    if (config_node.IsNull()) {
        std::cerr << "Configuration file is empty or invalid." << std::endl;
        return false;
    }

    // 初始化其他模块，并检查返回值
    if (!InitDataPath(config_node)) {
        std::cerr << "Failed to initialize data path." << std::endl;
        return false;
    }
    if (!InitRegistration(registration_ptr_, config_node)) {
        std::cerr << "Failed to initialize registration." << std::endl;
        return false;
    }
    if (!InitFilter("local_map", local_map_filter_ptr_, config_node)) {
        std::cerr << "Failed to initialize local map filter." << std::endl;
        return false;
    }
    if (!InitFilter("frame", frame_filter_ptr_, config_node)) {
        std::cerr << "Failed to initialize frame filter." << std::endl;
        return false;
    }
  

    return true;
}

bool FrontEnd::IsInitialized() const {
    return initialized_;
}

bool FrontEnd::InitParam(const YAML::Node& config_node) {
    key_frame_distance_ = config_node["key_frame_distance"].as<float>();
    local_frame_num_ = config_node["local_frame_num"].as<int>();

    return true;
}

bool FrontEnd::InitDataPath(const YAML::Node& config_node) {
    data_path_ = config_node["data_path"].as<std::string>();
    if (data_path_ == "./") {
        data_path_ = WORK_SPACE_PATH;
    }
    data_path_ += "/slam_data";

    if (boost::filesystem::is_directory(data_path_)) {
        boost::filesystem::remove_all(data_path_);
    }

    boost::filesystem::create_directory(data_path_);
    if (!boost::filesystem::is_directory(data_path_)) {
        LOG(WARNING) << "文件夹 " << data_path_ << " 未创建成功!";
        return false;
    } else {
        LOG(INFO) << "地图点云存放地址：" << data_path_;
    }

    std::string key_frame_path = data_path_ + "/key_frames";
    boost::filesystem::create_directory(data_path_ + "/key_frames");
    if (!boost::filesystem::is_directory(key_frame_path)) {
        LOG(WARNING) << "文件夹 " << key_frame_path << " 未创建成功!";
        return false;
    } else {
        LOG(INFO) << "关键帧点云存放地址：" << key_frame_path << std::endl << std::endl;
    }

    return true;
}

bool FrontEnd::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node) {
    std::string registration_method = config_node["registration_method"].as<std::string>();
    LOG(INFO) << "点云匹配方式为：" << registration_method;

    if (registration_method == "NDT") {
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    } else {
        LOG(ERROR) << "没找到与 " << registration_method << " 相对应的点云匹配方式!";
        return false;
    }

    return true;
}

bool FrontEnd::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node) {
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();
    LOG(INFO) << filter_user << "选择的滤波方法为：" << filter_mothod;

    if (filter_mothod == "voxel_filter") {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
    } else {
        LOG(ERROR) << "没有为 " << filter_user << " 找到与 " << filter_mothod << " 相对应的滤波方法!";
        return false;
    }

    return true;
}


bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose) {
    init_pose_ = init_pose;
    return true;
}


   bool FrontEnd::Update(const CloudData<PointXYZIRT>& cloud_data, Eigen::Matrix4f& cloud_pose) {
    current_frame_.cloud_data.SetTime(cloud_data.GetTime());
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_data.GetCloudData(), *current_frame_.cloud_data.GetCloudData(), indices);

    CloudData<PointXYZIRT>::CLOUD_PTR filtered_cloud_ptr(new CloudData<PointXYZIRT>::CLOUD());
    frame_filter_ptr_->Filter(current_frame_.cloud_data.GetCloudData(), filtered_cloud_ptr);

    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = init_pose_;

    // 局部地图容器中没有关键帧，代表是第一帧数据
    // 此时把当前帧数据作为第一个关键帧，并更新局部地图容器和全局地图容器
    //第一帧
    if (local_map_frames_.size() == 0) {
        //第一帧肯定很关键
        current_frame_.pose = init_pose_;
        UpdateWithNewFrame(current_frame_);
        cloud_pose = current_frame_.pose;
        return true;
    }

    // 不是第一帧，就正常匹配
    CloudData<PointXYZIRT>::CLOUD_PTR result_cloud_ptr(new CloudData<PointXYZIRT>::CLOUD());
    registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr, current_frame_.pose);
    cloud_pose = current_frame_.pose;

    // 更新相邻两帧的相对运动
    step_pose = last_pose.inverse() * current_frame_.pose;
    predict_pose = current_frame_.pose * step_pose;
    last_pose = current_frame_.pose;

    // 匹配之后根据距离判断是否需要生成新的关键帧，如果需要，则做相应更新
    if (fabs(last_key_frame_pose(0,3) - current_frame_.pose(0,3)) + 
        fabs(last_key_frame_pose(1,3) - current_frame_.pose(1,3)) +
        fabs(last_key_frame_pose(2,3) - current_frame_.pose(2,3)) > key_frame_distance_) {
        UpdateWithNewFrame(current_frame_);
        last_key_frame_pose = current_frame_.pose;
    }

    return true;
}

bool FrontEnd::UpdateWithNewFrame(const Frame& new_key_frame) {
    //新建一个关键帧
    Frame key_frame = new_key_frame;
    // 这一步的目的是为了把关键帧的点云保存下来
    // 由于用的是共享指针，所以直接复制只是复制了一个指针而已
    // 此时无论你放多少个关键帧在容器里，这些关键帧点云指针都是指向的同一个点云
    key_frame.cloud_data.GetCloudData().reset(new CloudData<PointXYZIRT>::CLOUD(*new_key_frame.cloud_data.GetCloudData()));
    
    
    
    // 更新局部地图  导入关键帧
    local_map_frames_.push_back(key_frame);
    //局部地图帧数不能比规定的多，限定在一个范围中
    while (local_map_frames_.size() > static_cast<size_t>(local_frame_num_)) {
        local_map_frames_.pop_front();
    }

    local_map_ptr_.reset(new CloudData<PointXYZIRT>::CLOUD());
    //转换后的点云
    CloudData<PointXYZIRT>::CLOUD_PTR transformed_cloud_ptr(new CloudData<PointXYZIRT>::CLOUD());
    for (size_t i = 0; i < local_map_frames_.size(); ++i) {
        //将当前地图中的所有帧全部变换到同一坐标系下
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.GetCloudData(), 
                                 *transformed_cloud_ptr, 
                                 local_map_frames_.at(i).pose);

        *local_map_ptr_ += *transformed_cloud_ptr;
    }

    // 更新ndt匹配的目标点云
    // 关键帧数量还比较少的时候不滤波，因为点云本来就不多，太稀疏影响匹配效果
    if (local_map_frames_.size() < 10) {
        registration_ptr_->SetInputTarget(local_map_ptr_);
    } else {
        CloudData<PointXYZIRT>::CLOUD_PTR filtered_local_map_ptr(new CloudData<PointXYZIRT>::CLOUD());
        local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);
        registration_ptr_->SetInputTarget(filtered_local_map_ptr);
    }

    return true;
}



}