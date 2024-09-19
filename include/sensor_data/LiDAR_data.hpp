

#ifndef LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_HPP_
#define LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_HPP_

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include"global_settings/global_settings.hpp"
struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT (VelodynePointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) (float, time, time)
)

struct LiovxPointCustomMsg
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float time;
    uint16_t ring;
    uint16_t tag;//多了一个tag
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW//内存对齐
} EIGEN_ALIGN16;//16字节对齐
POINT_CLOUD_REGISTER_POINT_STRUCT (LiovxPointCustomMsg,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity) (float, time, time)
    (uint16_t, ring, ring) (uint16_t, tag, tag)
)

struct OusterPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t noise;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OusterPointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint32_t, t, t) (uint16_t, reflectivity, reflectivity)
    (uint8_t, ring, ring) (uint16_t, noise, noise) (uint32_t, range, range)
)


// using PointXYZIRT = LiovxPointCustomMsg;
// using PointXYZIRT =VelodynePointXYZIRT;
using PointXYZIRT = pcl::PointXYZ;

namespace localization {

    // 模板类 CloudData 支持不同类型的点结构
    template<typename PointT>
    class CloudData {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    public:
        using CLOUD = pcl::PointCloud<PointT>;
        using CLOUD_PTR = typename CLOUD::Ptr;

    public:
        CloudData();
        CLOUD_PTR GetCloudData() ;
        CLOUD_PTR GetCloudData() const ;
        double GetTime() const ;
        void SetTime(double time);


    private:
        double time_ = 0.0;//fix???
        CLOUD_PTR cloud_ptr_;  
    };

    // 构造函数实现，初始化时间和点云数据
    template<typename PointT>
    CloudData<PointT>::CloudData() 
        : time_(0.0), cloud_ptr_(new CLOUD()) {  
    }

    // 获取点云数据
    template<typename PointT>
    typename CloudData<PointT>::CLOUD_PTR CloudData<PointT>::GetCloudData() {
        return cloud_ptr_;
    }

    template<typename PointT>
    typename CloudData<PointT>::CLOUD_PTR  CloudData<PointT>::GetCloudData() const {
    return cloud_ptr_;
    }

    template<typename PointT>
    double CloudData<PointT>::GetTime() const  {
        return time_;
    }

    template<typename PointT>
    void CloudData<PointT>::SetTime(double time) {
        time_ = time;  // 修改成员变量 time_
    }

} // namespace localization

#endif  // LIDAR_LOCALIZATION_SENSOR_DATA_CLOUD_DATA_HPP_


