#pragma once

#include <pcl/point_types.h>

namespace pcl
{
namespace visualization
{

template<typename T>
class PointCloudColorHandler
{
public:
    PointCloudColorHandler()
    {}
};

template<typename T>
class PointCloudColorHandlerGenericField : public PointCloudColorHandler<T>
{
public:
    PointCloudColorHandlerGenericField(typename pcl::PointCloud<T>::Ptr cloud, const std::string &field_name)
    {
        (void)cloud;
        (void)field_name;
    }
};

} //namespace visualization
} //namespace pcl
