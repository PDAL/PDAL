#pragma once

namespace pcl
{

template<typename POINT>
struct VoxelGrid
{
    template <typename T>
    void setInputCloud(T& cloud)
    { (void)cloud; }

    template <typename T>
    void filter(T& cloud)
    { (void)cloud; }

    template <typename T>
    void setLeafSize(T x, T y, T z)
    { (void)x; (void)y; (void)z; }
};

} //namespace pcl
