#include "models/scan_adjust/distortion_adjust.hpp"
#include "glog/logging.h"

namespace localization {
void DistortionAdjust::SetMotionInfo(float scan_period, VelocityData velocity_data) {
    scan_period_ = scan_period;
    velocity_ << velocity_data.linear_velocity.x, velocity_data.linear_velocity.y, velocity_data.linear_velocity.z;
    angular_rate_ << velocity_data.angular_velocity.x, velocity_data.angular_velocity.y, velocity_data.angular_velocity.z;
}

bool DistortionAdjust::AdjustCloud(const CloudData<PointXYZIRT>::CLOUD_PTR& input_cloud_ptr, const CloudData<PointXYZIRT>::CLOUD_PTR& output_cloud_ptr) {
    //进行了深拷贝，耗时较多，待优化
    CloudData<PointXYZIRT>::CLOUD_PTR origin_cloud_ptr(new CloudData<PointXYZIRT>::CLOUD(*input_cloud_ptr));
    output_cloud_ptr->points.clear();

    float orientation_space = 2.0 * M_PI;//360度
    float delete_space = 5.0 * M_PI / 180.0;//5度
    // start_orientation 是通过计算点云第一个点的角度（使用 atan2 函数）得到的扫描起始方向。
    float start_orientation = atan2(origin_cloud_ptr->points[0].y, origin_cloud_ptr->points[0].x);

    Eigen::AngleAxisf t_V(start_orientation, Eigen::Vector3f::UnitZ());//初始方向向量
    Eigen::Matrix3f rotate_matrix = t_V.matrix();
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix.block<3,3>(0,0) = rotate_matrix.inverse();
    //将点云转一下，此时起始点为0度
    pcl::transformPointCloud(*origin_cloud_ptr, *origin_cloud_ptr, transform_matrix);
    //将激光雷达的速度和角速度也旋转到新的参考坐标系下，以便在后续计算中使用。
    velocity_ = rotate_matrix * velocity_;
    angular_rate_ = rotate_matrix * angular_rate_;
// 遍历点云中的每个点，首先计算该点的扫描角度 orientation。
// 如果角度小于删除阈值 delete_space，则忽略该点。计算该点的实际扫描时间 real_time，该时间是基于点的角度和扫描周期 scan_period_ 计算出来的。
    for (size_t point_index = 1; point_index < origin_cloud_ptr->points.size(); ++point_index) {
        float orientation = atan2(origin_cloud_ptr->points[point_index].y, origin_cloud_ptr->points[point_index].x);
        if (orientation < 0.0)
            orientation += 2.0 * M_PI;
        
        if (orientation < delete_space || 2.0 * M_PI - orientation < delete_space)
            continue;

        float real_time = fabs(orientation) / orientation_space * scan_period_ - scan_period_ / 2.0;

        Eigen::Vector3f origin_point(origin_cloud_ptr->points[point_index].x,
                                     origin_cloud_ptr->points[point_index].y,
                                     origin_cloud_ptr->points[point_index].z);

        Eigen::Matrix3f current_matrix = UpdateMatrix(real_time);//根据陀螺仪计算出旋转的角度得到旋转矩阵
        Eigen::Vector3f rotated_point = current_matrix * origin_point;//先旋转
        Eigen::Vector3f adjusted_point = rotated_point + velocity_ * real_time;//再平移
        PointXYZIRT point;
        point.x = adjusted_point(0);
        point.y = adjusted_point(1);
        point.z = adjusted_point(2);
        output_cloud_ptr->points.push_back(point);
    }

    pcl::transformPointCloud(*output_cloud_ptr, *output_cloud_ptr, transform_matrix.inverse());
    return true;
}

Eigen::Matrix3f DistortionAdjust::UpdateMatrix(float real_time) {
    Eigen::Vector3f angle = angular_rate_ * real_time;
    Eigen::AngleAxisf t_Vz(angle(2), Eigen::Vector3f::UnitZ());
    Eigen::AngleAxisf t_Vy(angle(1), Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf t_Vx(angle(0), Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf t_V;
    t_V = t_Vz * t_Vy * t_Vx;
    return t_V.matrix();
}
} // namespace lidar_localization