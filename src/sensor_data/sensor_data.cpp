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
    geo_converter.Forward(latitude, longitude, altitude, local_E, local_N, local_U);
}
}