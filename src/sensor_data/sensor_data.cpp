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
}