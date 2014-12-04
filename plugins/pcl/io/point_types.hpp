#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/io/impl/pcd_io.hpp>

struct XYZIRGBA
{
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t rgba;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;


POINT_CLOUD_REGISTER_POINT_STRUCT(XYZIRGBA,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint32_t, rgba, rgba)
                                 )

