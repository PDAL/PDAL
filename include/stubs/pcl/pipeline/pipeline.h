#pragma once

namespace pcl
{

template<typename POINT>
struct Pipeline
{
    template <typename T>
    void setInputCloud(T& cloud)
    { (void)cloud; }

    template <typename T>
    void setFilename(T& name)
    { (void)name; }

    template <typename T>
    void filter(T& cloud)
    { (void)cloud; }

    template <typename T>
    void setOffsets(T x, T y, T z)
    { (void)x; (void)y; (void)z; }
};

} //namespace pcl
