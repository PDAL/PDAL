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

struct PCLHeader
{
};

struct PCLPointCloud2
{
    uint32_t height;
    uint32_t width;

  public:
    typedef std::shared_ptr<PCLPointCloud2> Ptr;
    typedef std::shared_ptr<PCLPointCloud2  const> ConstPtr;
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

template<typename T>
struct has_color
{
    T t;
    enum { value = true };
};

} //namespace traits

} //namespace pcl
