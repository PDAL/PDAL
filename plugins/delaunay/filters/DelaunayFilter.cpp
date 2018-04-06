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
	
	delaunayPoints.push_back(x);
	delaunayPoints.push_back(y);
    }
    
    GEO::initialize();
    
    GEO::index_t numDimensions = 2;
    GEO::Delaunay_var triangulation = GEO::Delaunay::create(GEO::coord_index_t(numDimensions), "BDEL2d");
    GEO::index_t numPoints = delaunayPoints.size() / numDimensions;
    triangulation->set_vertices(numPoints, delaunayPoints.data());
    
    // TODO Check for null (= already exists)
    TriangularMesh *mesh = pointView->createMesh("delaunay");
    
    std::cout << triangulation->nb_vertices() << " vertices in triangulation." << std::endl;
    std::cout << triangulation->nb_cells() << " cells in triangulation." << std::endl;
    for (GEO::index_t i = 0; i < triangulation->nb_cells(); i++)
    {
	GEO::index_t v_0 = triangulation->cell_vertex(i, 0);
	GEO::index_t v_1 = triangulation->cell_vertex(i, 1);
	GEO::index_t v_2 = triangulation->cell_vertex(i, 2);
	
	mesh->add((int)v_0, (int)v_1, (int)v_2);
    }
    
    PointViewSet viewSet;
    viewSet.insert(pointView);
    
    return viewSet;
}

} // namespace pdal
