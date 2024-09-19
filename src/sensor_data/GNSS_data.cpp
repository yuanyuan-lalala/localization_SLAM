#include "sensor_data/GNSS_data.hpp"

#include "glog/logging.h"

//静态成员变量必须在类外初始化
bool localization::GNSSData::origin_position_inited = false;
GeographicLib::LocalCartesian localization::GNSSData::geo_converter;

namespace localization {

void GNSSData::InitOriginPosition() {
    geo_converter.Reset(latitude, longitude, altitude);
    origin_position_inited = true;
}

void GNSSData::UpdateXYZ() {
    if (!origin_position_inited) {
        LOG(WARNING) << "GeoConverter has not set origin position";
    }
    //将经纬高转化为笛卡尔坐标系
    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
}

// 分割NMEA句子
std::vector<std::string> GNSSData::SplitNMEAString(const std::string& sentence, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(sentence);
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

double GNSSData::ConvertNMEACoordinateToDecimal(const std::string& nmea_coord, const std::string& direction) {
    if (nmea_coord.empty()) {
        return 0.0;
    }

    // 纬度格式: ddmm.mmmm, 经度格式: dddmm.mmmm
    double degrees = std::atof(nmea_coord.substr(0, nmea_coord.find('.') - 2).c_str());
    double minutes = std::atof(nmea_coord.substr(nmea_coord.find('.') - 2).c_str());

    double decimal = degrees + minutes / 60.0;

    // 根据方向修正
    if (direction == "S" || direction == "W") {
        decimal = -decimal;
    }

    return decimal;
}

bool GNSSData::ParseFromNMEA(const std::string& sentence){
    // 检查是否是GPGGA句子，只处理GPGGA
    if (sentence.find("$GPGGA") != 0) {
        // 忽略非GPGGA句子
        return false;
    }

    // 分割NMEA句子
    std::vector<std::string> fields = SplitNMEAString(sentence, ',');

    if (fields.size() < 15) {
        std::cerr << "Incomplete GPGGA sentence." << std::endl;
        return false;
    }

    // 提取纬度、经度和海拔等信息
    std::string lat_str = fields[2];       // 纬度
    std::string lat_dir = fields[3];       // 纬度方向
    std::string lon_str = fields[4];       // 经度
    std::string lon_dir = fields[5];       // 经度方向
    std::string alt_str = fields[9];       // 海拔

    // 将纬度和经度从NMEA格式转换为十进制格式
    latitude = ConvertNMEACoordinateToDecimal(lat_str, lat_dir);
    longitude = ConvertNMEACoordinateToDecimal(lon_str, lon_dir);

    // 将海拔转换为浮点数
    altitude = std::atof(alt_str.c_str());

    return true;  // 解析成功
}

bool GNSSData::SyncData(std::deque<GNSSData,Eigen::aligned_allocator<GNSSData>>& UnsyncedData, std::deque<GNSSData,Eigen::aligned_allocator<GNSSData>>& SyncedData, double sync_time) {
    // 传感器数据按时间序列排列，在传感器数据中为同步的时间点找到合适的时间位置
    // 即找到与同步时间相邻的左右两个数据
    // 需要注意的是，如果左右相邻数据有一个离同步时间差值比较大，则说明数据有丢失，时间离得太远不适合做差值
    while (UnsyncedData.size() >= 2) {
        if (UnsyncedData.front().time > sync_time)
            return false;
        if (UnsyncedData.at(1).time < sync_time) {
            UnsyncedData.pop_front();
            continue;
        }
        if (sync_time - UnsyncedData.front().time > 0.2) {
            UnsyncedData.pop_front();
            break;
        }
        if (UnsyncedData.at(1).time - sync_time > 0.2) {
            UnsyncedData.pop_front();
            break;
        }
        break;
    }
    if (UnsyncedData.size() < 2)
        return false;

    GNSSData front_data = UnsyncedData.at(0);
    GNSSData back_data = UnsyncedData.at(1);
    GNSSData synced_data;

    double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
    double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
    synced_data.time = sync_time;
    synced_data.status = back_data.status;
    synced_data.longitude = front_data.longitude * front_scale + back_data.longitude * back_scale;
    synced_data.latitude = front_data.latitude * front_scale + back_data.latitude * back_scale;
    synced_data.altitude = front_data.altitude * front_scale + back_data.altitude * back_scale;
    synced_data.local_E = front_data.local_E * front_scale + back_data.local_E * back_scale;
    synced_data.local_N = front_data.local_N * front_scale + back_data.local_N * back_scale;
    synced_data.local_U = front_data.local_U * front_scale + back_data.local_U * back_scale;

    SyncedData.push_back(synced_data);
    
    return true;
}


}