#pragma once

#include <memory>

namespace pcl
{

struct PointNormal
{
    int x, y, z;
};

template<typename T>
struct PointCloud
{
    typedef T PointType;
    typedef std::shared_ptr<PointCloud> Ptr;

    std::vector<PointType> points;
    double width;
    double height;
    bool is_dense;
};

namespace traits
{

template<typename T>
struct has_xyz
{
    T t;
    enum { value = true };
};

template<typename T>
struct has_intensity
{
    T t;
    enum { value = true };
};

} //namespace traits

} //namespace pcl
