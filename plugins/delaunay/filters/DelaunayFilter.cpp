// TODO: insert copyright notice

#include "DelaunayFilter.hpp"
#include "Delaunay_psm.h"

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
    std::cout << "Point view size is " << pointView->size() << "." << std::endl;
    
    std::vector<double> delaunayPoints;
    for (PointId i = 0; i < pointView->size(); i++)
    {
	PointRef point(*pointView, i);
	double x(point.getFieldAs<double>(Dimension::Id::X));
	double y(point.getFieldAs<double>(Dimension::Id::Y));
	double z(point.getFieldAs<double>(Dimension::Id::Z));
	std::cout << "(" << x << ", " << y << ", " << z <<  ")" << std::endl;
	
	delaunayPoints.push_back(x);
	delaunayPoints.push_back(y);
	delaunayPoints.push_back(z);
    }
    
    GEO::initialize();
    //GEO::CmdLine::import_arg_group("standard");
    
    int numDimensions = 3;
    GEO::Delaunay_var triangulation = GEO::Delaunay::create(GEO::coord_index_t(numDimensions));
    GEO::index_t numPoints = delaunayPoints.size() / numDimensions;
    //triangulation->set_vertices(numPoints, delaunayPoints.data());
    
    
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
