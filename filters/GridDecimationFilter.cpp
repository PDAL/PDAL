/******************************************************************************
* Copyright (c) 2023, Antoine Lavenant (antoine.lavenant@ign.fr)
*
* All rights reserved.
*
****************************************************************************/

#include "GridDecimationFilter.hpp"

#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>

#include <pdal/private/gdal/GDALUtils.hpp>

#include "private/Point.hpp"
#include "private/pnp/GridPnp.hpp"

#include <sstream>
#include <cstdarg>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.gridDecimation",
    "keep max or min points in a grid",
    "http://pdal.io/stages/filters.GridDecimation.html"
};

CREATE_STATIC_STAGE(GridDecimationFilter, s_info)

std::string GridDecimationFilter::getName() const { return s_info.name; }

GridDecimationFilter::GridDecimationFilter() : m_args(new GridDecimationFilter::GridArgs)
{}


GridDecimationFilter::~GridDecimationFilter()
{}


void GridDecimationFilter::addArgs(ProgramArgs& args)
{
    args.add("resolution", "Cell edge size, in units of X/Y",m_args->m_edgeLength, 1.);
    args.add("output_type", "Point keept into the cells ('min', 'max')", m_args->m_methodKeep, "max" );
    args.add("value", "Value to assign to dimension based on expression.",m_args->m_statements);
}

void GridDecimationFilter::initialize()
{
}

void GridDecimationFilter::prepared(PointTableRef table)
{
    PointLayoutPtr layout(table.layout());

    for (expr::AssignStatement& expr : m_args->m_statements)
    {
        auto status = expr.prepare(layout);
        if (!status)
            throwError(status.what());
    }
}

void GridDecimationFilter::ready(PointTableRef table)
{
    if (m_args->m_edgeLength <=0)
        throwError("resolution must be positive.");

    if (m_args->m_methodKeep != "max" && m_args->m_methodKeep != "min")
        throwError("The output_type must be 'max' or 'min'.");
}


void GridDecimationFilter::processOne(BOX2D bounds, PointRef& point, PointViewPtr view)
{
    //get the grid cell
    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);
    int id = point.getFieldAs<double>(Dimension::Id::PointId);

    // if x==(xmax of the cell), we assume the point are in the upper cell
    // if y==(ymax of the cell), we assume the point are in the right cell
    int width = static_cast<int>((x - bounds.minx) / m_args->m_edgeLength);
    int height = static_cast<int>((y - bounds.miny) / m_args->m_edgeLength);

    // to avoid numeric pb due to the division (append if the point is on the grid)
    if (x < bounds.minx+width*m_args->m_edgeLength) width--;
    if (y < bounds.miny+height*m_args->m_edgeLength) height--;
    if (x >= bounds.minx+(width+1)*m_args->m_edgeLength) width++;
    if (y >= bounds.miny+(height+1)*m_args->m_edgeLength) height++;

    auto mptRefid = this->grid.find( std::make_pair(width,height) );
    assert( mptRefid !=  this->grid.end());
    auto ptRefid = mptRefid->second;

    if (ptRefid==-1)
    {
        this->grid[ std::make_pair(width,height) ] = (long) point.pointId();
        return;
    }
    
    PointRef ptRef = view->point(ptRefid);

    double z = point.getFieldAs<double>(Dimension::Id::Z);
    double zRef = ptRef.getFieldAs<double>(Dimension::Id::Z);

    if (this->m_args->m_methodKeep == "max" && z>zRef)
        this->grid[ std::make_pair(width,height) ] = (long) point.pointId();
    if (this->m_args->m_methodKeep == "min" && z<zRef)
        this->grid[ std::make_pair(width,height) ] = (long) point.pointId();
}

void GridDecimationFilter::createGrid(BOX2D bounds)
{
    // +2 to be sur to deal with all points (avoid some numeric precision issue due to the division)
    size_t d_width = std::floor((bounds.maxx - bounds.minx) / m_args->m_edgeLength) + 2;
    size_t d_height = std::floor((bounds.maxy - bounds.miny) / m_args->m_edgeLength) + 2;

    if (d_width < 0.0 || d_width > (std::numeric_limits<int>::max)())
        throwError("Grid width out of range.");
    if (d_height < 0.0 || d_height > (std::numeric_limits<int>::max)())
        throwError("Grid height out of range.");
    
    int width = static_cast<int>(d_width);
    int height = static_cast<int>(d_height);
    
    for (size_t l(0); l<d_height; l++)
        for (size_t c(0); c<d_width; c++)
            this->grid.insert( std::make_pair( std::make_pair(c,l), -1)  );
}

PointViewSet GridDecimationFilter::run(PointViewPtr view)
{
    PointViewSet viewSet;

    if (view->empty())
        return viewSet;
    
    BOX2D bounds;
    view->calculateBounds(bounds);
    createGrid(bounds);

    for (PointId i = 0; i < view->size(); ++i)
    {
        PointRef point = view->point(i);
        processOne(bounds,point,view);
    }
    
    std::set<PointId> keepPoint;
    for (auto it : this->grid)
        if (it.second != -1)
            keepPoint.insert(it.second);
    
    for (PointId i = 0; i < view->size(); ++i)
    {
        if (keepPoint.find(view->point(i).pointId()) != keepPoint.end())
        {
            PointRef point = view->point(i);
            for (expr::AssignStatement& expr : m_args->m_statements)
                if (expr.conditionalExpr().eval(point))
                    point.setField(expr.identExpr().eval(), expr.valueExpr().eval(point));
        }
    }
    
    viewSet.insert(view);
    return viewSet;
}

} // namespace pdal
