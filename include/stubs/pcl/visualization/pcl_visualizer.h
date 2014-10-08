#pragma once

#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/point_types.h>

namespace pcl
{
namespace visualization
{

struct PCLVisualizer
{
    PCLVisualizer(const std::string &name = "", const bool create_interactor = true)
    {}

    void setBackgroundColor(const double &r, const double &g, const double &b, int viewport = 0)
    {
        (void)r;
        (void)g;
        (void)b;
        (void)viewport;
    }

    template <typename T>
    bool addPointCloud(typename pcl::PointCloud<T>::Ptr cloud, const PointCloudColorHandler<T>& color, const std::string &id = "cloud", int viewport = 0)
    {
        (void)cloud;
        (void)color;
        (void)id;
        (void)viewport;
    }

    bool const wasStopped()
    {}

    void spinOnce(int time = 1, bool force_redraw = false)
    {
        (void)time;
        (void)force_redraw;
    }
};

} //namespace visualization
} //namespace pcl
