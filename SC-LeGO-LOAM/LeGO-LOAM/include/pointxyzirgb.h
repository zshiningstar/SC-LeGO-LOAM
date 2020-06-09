#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/range_image/range_image.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

struct PointXYZIRGB
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    int r;
    int g;
    int b;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRGB,  
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (int, r, r) (int, g, g) (int, b, b)
)
typedef PointXYZIRGB  RGBIPoint;
typedef pcl::PointXYZI  PointType;