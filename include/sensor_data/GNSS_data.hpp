#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_GNSS_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_GNSS_DATA_HPP_

#include <vector>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <iostream>
#include"global_settings/global_settings.hpp"
#include "Geocentric/LocalCartesian.hpp"
#include"Eigen/Dense"
namespace localization {
    class GNSSData {
         EIGEN_MAKE_ALIGNED_OPERATOR_NEW 
    public:
    
        double time = 0.0;
        double longitude = 0.0;
        double latitude = 0.0;
        double altitude = 0.0;
        double local_E = 0.0;
        double local_N = 0.0;
        double local_U = 0.0;
        int status = 0;
        int service = 0;

    private:
        static GeographicLib::LocalCartesian geo_converter;
        static bool origin_position_inited;

    public: 
    
        bool ParseFromNMEA(const std::string& sentence);
        double ConvertNMEACoordinateToDecimal(const std::string& nmea_coord, const std::string& direction);
        std::vector<std::string> SplitNMEAString(const std::string& sentence, char delimiter);
        void InitOriginPosition();
        void UpdateXYZ();
    };
}
#endif


