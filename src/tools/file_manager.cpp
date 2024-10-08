#include "tools/file_manager.hpp"

#include <boost/filesystem.hpp>
#include "glog/logging.h"

namespace localization {
bool FileManager::CreateFile(std::ofstream& ofs, std::string file_path) {
    ofs.open(file_path.c_str(), std::ios::app);
    if (!ofs) {
        LOG(WARNING) << "无法生成文件: " << file_path;
        return false;
    }

    return true;
}
bool FileManager::InitDirectory(std::string directory_path, std::string use_for) {
    if (boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::remove_all(directory_path + "/tail");
        LOG(INFO) << use_for << "存放地址：" << std::endl << directory_path << std::endl << std::endl;
        return true;
    }

    return CreateDirectory(directory_path, use_for);
}

bool FileManager::CreateDirectory(std::string directory_path, std::string use_for) {
    if (!boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::create_directory(directory_path);
    }

    if (!boost::filesystem::is_directory(directory_path)) {
        LOG(WARNING) << "无法创建文件夹: " << std::endl << directory_path << std::endl << std::endl;
        return false;
    }

    LOG(INFO) << use_for << "存放地址：" << std::endl << directory_path << std::endl << std::endl;
    return true;
}
bool FileManager::CreateDirectory(std::string directory_path) {
    if (!boost::filesystem::is_directory(directory_path)) {
        boost::filesystem::create_directory(directory_path);
    }
    if (!boost::filesystem::is_directory(directory_path)) {
        LOG(WARNING) << "无法建立文件夹: " << directory_path;
        return false;
    }
    return true;
}
}