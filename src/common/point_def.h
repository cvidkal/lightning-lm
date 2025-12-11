//
// Created by xiang on 25-3-12.
//

#pragma once
#ifndef LIGHTNING_POINT_DEF_H
#define LIGHTNING_POINT_DEF_H

#include "common/eigen_types.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/// 地图点云的一些定义

/// clang-format off
struct PointXYZIT {
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY
    double time;
    PointXYZIT() {}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, time,
                                                                                                       timestamp))

namespace velodyne_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D

    float intensity;
    float time;
    std::uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace velodyne_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                      (float, time, time)(std::uint16_t, ring, ring))

namespace ouster_ros {
struct EIGEN_ALIGN16 Point {
    PCL_ADD_POINT4D
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t ring;
    uint16_t ambient;
    uint32_t range;
};
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
                                  (float, x, x)
                                      (float, y, y)
                                      (float, z, z)
                                      (float, intensity, intensity)
                                      // use std::uint32_t to avoid conflicting with pcl::uint32_t
                                      (std::uint32_t, t, t)
                                      (std::uint16_t, reflectivity, reflectivity)
                                      (std::uint8_t, ring, ring)
                                      (std::uint16_t, ambient, ambient)
                                      (std::uint32_t, range, range)
)
namespace livox_ros
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;                // 4D点坐标类型,xyz+padding,float padding用于填补位数,以满足存储对齐要求
    float intensity;                // Reflectivity
    uint8_t tag;                    // Livox point tag
    uint8_t line;                   // Laser line id
    uint8_t reflectivity;           // reflectivity, 0~255
    uint32_t offset_time;           // offset time relative to the base time
    PCL_ADD_RGB;                    // RGB
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 进行内存对齐
  };
  struct EIGEN_ALIGN16 PointSkyland
  {
    PCL_ADD_POINT4D;                // 4D点坐标类型,xyz+padding,float padding用于填补位数,以满足存储对齐要求
    float intensity;                // Reflectivity
    uint8_t tag;                    // Livox point tag
    uint8_t line;                   // Laser line id
    double timestamp;           // offset time relative to the base time
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 进行内存对齐
  };
}
// 注册livox_ros的Point类型
POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, intensity, intensity)(std::uint8_t, tag, tag)(std::uint8_t, line, line)(std::uint8_t, reflectivity, reflectivity)(std::uint32_t, offset_time, offset_time)(float, rgb, rgb))
// 注册livox_ros_skyland的Point类型
POINT_CLOUD_REGISTER_POINT_STRUCT(livox_ros::PointSkyland,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, intensity, intensity)(std::uint8_t, tag, tag)(std::uint8_t, line, line)(double, timestamp, timestamp))
// clang-format on

namespace lightning {

/// 各类点云的缩写

using PointType = PointXYZIT;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;
using PointVec = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

inline Vec3f ToVec3f(const PointType& pt) { return pt.getVector3fMap(); }
inline Vec3d ToVec3d(const PointType& pt) { return pt.getVector3fMap().cast<double>(); }

/// 带ring, range等其他信息的全量信息点云
struct FullPointType {
    PCL_ADD_POINT4D

    float range = 0;
    float radius = 0;
    uint8_t intensity = 0;
    uint8_t ring = 0;
    uint8_t angle = 0;
    double time = 0;
    float height = 0;

    inline FullPointType() {}
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/// 全量点云的定义
using FullPointCloudType = pcl::PointCloud<FullPointType>;
using FullCloudPtr = FullPointCloudType::Ptr;

inline Vec3f ToVec3f(const FullPointType& pt) { return pt.getVector3fMap(); }
inline Vec3d ToVec3d(const FullPointType& pt) { return pt.getVector3fMap().cast<double>(); }

using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
constexpr double G_m_s2 = 9.81;  // Gravity const in GuangDong/China

}  // namespace lightning

#endif  // LIGHTNING_POINT_DEF_H
