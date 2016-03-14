/*****************************************************************************

    (c) 2013 Hobu, Inc. hobu.inc@gmail.com

    Author: Howard Butler, hobu.inc@gmail.com

    This is free software; you can redistribute and/or modify it under the
    terms of the GNU Lesser General Licence as published by the Free Software
    Foundation. See the COPYING file for more information.

    This software is distributed WITHOUT ANY WARRANTY and without even the
    implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

*****************************************************************************/

#include "OGR.hpp"

#include <iostream>
#include <sstream>
#include <functional>

#include <pdal/GDALUtils.hpp>

#include <hexer/HexGrid.hpp>
#include <hexer/HexIter.hpp>

using namespace std;

namespace pdal
{

namespace hexdensity
{

namespace writer
{

OGR::OGR(std::string const& filename, std::string srs, std::string driver)
    : m_filename(filename)
    , m_driver(driver)
    , m_srs(srs)
    , m_ds(0)
    , m_layer(0)
{
    createLayer();
}

void OGR::createLayer()
{

    OGRSFDriverH driver = OGRGetDriverByName(m_driver.c_str());
    if (driver == NULL)
    {
        throw pdal::pdal_error("OGR Driver was null!");
    }


    m_ds = OGR_Dr_CreateDataSource(driver, m_filename.c_str(), NULL);
    if (m_ds == NULL)
    {
        throw pdal::pdal_error("Data source creation was null!");
    }

    pdal::gdal::SpatialRef spatialref(m_srs);
    OGRSpatialReferenceH ref = (OGRSpatialReferenceH)spatialref.get();

    m_layer = OGR_DS_CreateLayer(m_ds, m_filename.c_str(), ref, wkbMultiPolygon, NULL);
    if (m_layer == NULL)
    {
        throw pdal::pdal_error("Layer creation was null!");
    }

    OGRFieldDefnH hFieldDefn;
    hFieldDefn = OGR_Fld_Create("ID", OFTInteger);
    if (OGR_L_CreateField(m_layer, hFieldDefn, TRUE) != OGRERR_NONE)
    {
        std::ostringstream oss;
        oss << "Could not create ID field on layer with error '"
            << CPLGetLastErrorMsg() << "'";
        throw pdal::pdal_error(oss.str());
    }
    OGR_Fld_Destroy(hFieldDefn);

    hFieldDefn = OGR_Fld_Create("COUNT", OFTInteger);
    if (OGR_L_CreateField(m_layer, hFieldDefn, TRUE) != OGRERR_NONE)
    {
        std::ostringstream oss;
        oss << "Could not create COUNT field on layer with error '"
            << CPLGetLastErrorMsg() << "'";
        throw pdal::pdal_error(oss.str());
    }
    OGR_Fld_Destroy(hFieldDefn);


}

void OGR::writeBoundary(hexer::HexGrid *grid)
{
    OGRGeometryH multi = OGR_G_CreateGeometry(wkbMultiPolygon);

    const std::vector<hexer::Path *>& paths = grid->rootPaths();
    for (auto pi = paths.begin(); pi != paths.end(); ++pi)
    {
        OGRGeometryH polygon = OGR_G_CreateGeometry(wkbPolygon);
        collectPath(*pi, polygon);

        if( OGR_G_AddGeometryDirectly(multi, polygon ) != OGRERR_NONE )
        {
            std::ostringstream oss;
            oss << "Unable to add polygon to multipolygon with error '"
                << CPLGetLastErrorMsg() << "'";
            throw pdal::pdal_error(oss.str());
        }
    }

    OGRFeatureH hFeature;

    hFeature = OGR_F_Create(OGR_L_GetLayerDefn(m_layer));
    OGR_F_SetFieldInteger( hFeature, OGR_F_GetFieldIndex(hFeature, "ID"), 0);

    OGR_F_SetGeometry(hFeature, multi);
    OGR_G_DestroyGeometry(multi);

    if( OGR_L_CreateFeature( m_layer, hFeature ) != OGRERR_NONE )
    {
        std::ostringstream oss;
        oss << "Unable to create feature for multipolygon with error '"
            << CPLGetLastErrorMsg() << "'";
        throw pdal::pdal_error(oss.str());
    }
    OGR_F_Destroy( hFeature );
}

void OGR::collectPath(hexer::Path* path, OGRGeometryH polygon)
{
    OGRGeometryH ring = OGR_G_CreateGeometry(wkbLinearRing);

    vector<hexer::Point> pts = path->points();

    vector<hexer::Point>::const_iterator i;
    for (i = pts.begin(); i != pts.end(); ++i)
    {
        OGR_G_AddPoint_2D(ring, i->m_x, i->m_y);
    }

    if( OGR_G_AddGeometryDirectly(polygon, ring) != OGRERR_NONE )
    {
        std::ostringstream oss;
        oss << "Unable to add geometry with error '" <<
            CPLGetLastErrorMsg() << "'";
        throw pdal::pdal_error(oss.str());
    }

    vector<hexer::Path *> paths = path->subPaths();
    for (vector<hexer::Path*>::size_type pi = 0; pi != paths.size(); ++pi)
    {
        hexer::Path* p = paths[pi];
        collectPath(p, polygon);
    }
}

OGRGeometryH OGR::collectHexagon(hexer::HexInfo const& info, hexer::HexGrid const* grid)
{
    OGRGeometryH ring = OGR_G_CreateGeometry(wkbLinearRing);

    hexer::Point pos = info.m_center;
    pos += grid->origin();


    OGR_G_AddPoint_2D(ring, pos.m_x, pos.m_y);
    for (int i = 1; i <= 5; ++i)
    {
        hexer::Point p = pos + grid->offset(i);
        OGR_G_AddPoint_2D(ring, p.m_x, p.m_y);
    }
    OGR_G_AddPoint_2D(ring, pos.m_x, pos.m_y);

    OGRGeometryH polygon = OGR_G_CreateGeometry(wkbPolygon);
    if( OGR_G_AddGeometryDirectly(polygon, ring ) != OGRERR_NONE )
    {
        std::ostringstream oss;
        oss << "Unable to add ring to polygon in collectHexagon '"
            << CPLGetLastErrorMsg() << "'";
        throw pdal::pdal_error(oss.str());
    }

    return polygon;

}


void OGR::writeDensity(hexer::HexGrid *grid)
{
    int counter(0);
    for (hexer::HexIter iter = grid->hexBegin(); iter != grid->hexEnd(); ++iter)
    {

        hexer::HexInfo hi = *iter;
        OGRGeometryH polygon = collectHexagon(hi, grid);
        OGRFeatureH hFeature;

        hFeature = OGR_F_Create(OGR_L_GetLayerDefn(m_layer));
        OGR_F_SetFieldInteger( hFeature, OGR_F_GetFieldIndex(hFeature, "ID"),
            counter);
        OGR_F_SetFieldInteger( hFeature, OGR_F_GetFieldIndex(hFeature, "COUNT"),
            hi.m_density);

        OGR_F_SetGeometry(hFeature, polygon);
        OGR_G_DestroyGeometry(polygon);

        if( OGR_L_CreateFeature( m_layer, hFeature ) != OGRERR_NONE )
        {
            std::ostringstream oss;
            oss << "Unable to create feature for multipolygon with error '"
                << CPLGetLastErrorMsg() << "'";
            throw pdal::pdal_error(oss.str());
        }
        OGR_F_Destroy( hFeature );
        counter++;
    }
}


OGR::~OGR()
{
    OGR_DS_Destroy(m_ds);
}

} // namespace writer
} // namespace hexer
} // namespace pdal

