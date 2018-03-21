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

PointViewSet DelaunayFilter::run(PointViewPtr input)
{
    PointViewPtr output = input->makeNew();
    PointViewSet viewSet;
    viewSet.insert(input);
    viewSet.insert(output);
    
    // TODO Check for null (= already exists)
    TriangularMesh *mesh = output->createMesh("delaunay");
    
    //Testing...
    mesh->add(0, 1, 2);
    mesh->add(1, 2, 3);
    
    return viewSet;
}

} // namespace pdal
