#include "sensor_data/key_frame.hpp"

namespace localization {
Eigen::Quaternionf KeyFrame::GetQuaternion() {
    Eigen::Quaternionf q;
    q = pose.block<3,3>(0,0);

    return q;
}
}