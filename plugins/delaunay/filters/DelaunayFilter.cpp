// TODO: insert copyright notice

#include "DelaunayFilter.hpp"

namespace pdal
{

static PluginInfo const s_info
{
    "filters.delaunay",
    "Perform Delaunay triangulation of a pointcloud",
    "http://pdal.io/stages/filters.delaunay.html"
};

CREATE_SHARED_STAGE(DelaunayFilter, s_info)

std::string DelaunayFilter::getName() const
{
    return s_info.name;
}

PointViewSet DelaunayFilter::run(PointViewPtr pointView)
{
    // TODO Check for null (= already exists)
    TriangularMesh *mesh = pointView->createMesh("delaunay");
    
    //Testing...
    mesh->add(0, 1, 2);
    mesh->add(1, 2, 3);
    
    PointViewSet viewSet;
    viewSet.insert(pointView);
    
    return viewSet;
}

} // namespace pdal
