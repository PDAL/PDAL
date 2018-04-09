/******************************************************************************
* Copyright (c) 2018, Danish Agency for Data Supply and Efficiency,
* sdfe@sdfe.dk
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of SDFE nor the names of its contributors may be
*       used to endorse or promote products derived from this software
*       without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

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
