/******************************************************************************
* Copyright (c) 2016, Howard Butler (howard@hobu.co)
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
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
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

#include "OGR.hpp"

#include <iostream>
#include <sstream>
#include <functional>

#include <gdal.h>
#include <ogr_api.h>

#include <pdal/util/FileUtils.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>
#include <pdal/private/gdal/SpatialRef.hpp>

#include <filters/private/hexer/HexGrid.hpp>
#include <filters/private/hexer/HexIter.hpp>

using namespace std;

namespace pdal
{

namespace
{

void collectPath(hexer::Path* path, OGRGeometryH polygon)
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

OGRGeometryH collectHexagon(hexer::HexInfo const& info,
    hexer::HexGrid const* grid)
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

} // unnamed namespace


OGR::OGR(std::string const& filename, const std::string& wkt,
        std::string driver, std::string layerName)
    : m_filename(filename)
    , m_driver(driver)
    , m_ds(0)
    , m_layer(0)
    , m_layerName(layerName)
{
    createLayer(wkt);
}


OGR::~OGR()
{
    OGR_DS_Destroy(m_ds);
}


void OGR::createLayer(const std::string& wkt)
{
    gdal::registerDrivers();
    OGRSFDriverH driver = OGRGetDriverByName(m_driver.c_str());
    if (driver == NULL)
    {
        throw pdal::pdal_error("OGR Driver was null!");
    }

    if (FileUtils::fileExists(m_filename))
        m_ds = OGR_Dr_Open(driver, m_filename.c_str(), TRUE /*update*/);
    else
    {
        m_ds = OGR_Dr_CreateDataSource(driver, m_filename.c_str(), NULL);
        if (m_ds == NULL)
            throw pdal_error("Unable to create output file '" + m_filename +
                "' for density output.");
    }

    if (m_layerName.empty())
        m_layerName = m_filename;
    gdal::SpatialRef srs(wkt);
    m_layer = GDALDatasetCreateLayer(m_ds, m_layerName.c_str(), srs.get(),
        wkbMultiPolygon, NULL);
    if (m_layer == NULL)
        throw pdal_error("Layer creation was null!");

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

} // namespace pdal

