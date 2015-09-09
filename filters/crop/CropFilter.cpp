/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include "CropFilter.hpp"

#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/GDALUtils.hpp>

#include <sstream>
#include <cstdarg>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.crop",
    "Filter points inside or outside a bounding box or a polygon if PDAL was built with GEOS support.",
    "http://pdal.io/stages/filters.crop.html" );

CREATE_STATIC_PLUGIN(1, 0, CropFilter, Filter, s_info)

std::string CropFilter::getName() const { return s_info.name; }

#ifdef PDAL_HAVE_GEOS
namespace geos
{

static void _GEOSErrorHandler(const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    char buf[1024];

    vsnprintf(buf, sizeof(buf), fmt, args);
    std::cerr << "GEOS Error: " << buf << std::endl;

    va_end(args);
}

static void _GEOSWarningHandler(const char *fmt, ...)
{
    va_list args;

    char buf[1024];
    vsnprintf(buf, sizeof(buf), fmt, args);
    std::cout << "GEOS warning: " << buf << std::endl;

    va_end(args);
}

} // geos
#endif

CropFilter::CropFilter() : pdal::Filter()
{
    m_cropOutside = false;
#ifdef PDAL_HAVE_GEOS
    m_geosEnvironment = 0;
#endif
}


void CropFilter::processOptions(const Options& options)
{
    m_cropOutside = options.getValueOrDefault<bool>("outside", false);
    m_assignedSRS = options.getValueOrDefault<SpatialReference>("a_srs");
    try
    {
        m_bounds = options.getValues<BOX2D>("bounds");
    }
    catch (boost::bad_lexical_cast)
    {
        try
        {
            std::vector<BOX3D>  b3d = options.getValues<BOX3D>("bounds");
            for (auto& i: b3d)
                m_bounds.push_back(i.to2d());
//             m_bounds = b3d.to2d();
        }
        catch (boost::bad_lexical_cast)
        {
            std::ostringstream oss;
            oss << "Invalid bounds for " << getName() << ".  "
                "Format: '([xmin,xmax],[ymin,ymax])'.";
            throw pdal_error(oss.str());
        }
    }

    m_polys = options.getValues<std::string>("polygon");
#ifdef PDAL_HAVE_GEOS
    if (m_polys.size())
    {
        m_geosEnvironment = initGEOS_r(pdal::geos::_GEOSWarningHandler,
            pdal::geos::_GEOSErrorHandler);
        for (std::string poly : m_polys)
        {
            GeomPkg g;

            // Throws if invalid.
            g.m_geom = validatePolygon(poly);
            m_geoms.push_back(g);
        }
    }
#else
    if (m_polys.size())
    {
        std::ostringstream oss;

        oss << "Can't specify polygons for " << getName() <<
            " without PDAL built with GEOS.";
        throw pdal_error(oss.str());
    }
#endif
}


void CropFilter::ready(PointTableRef table)
{
#ifdef PDAL_HAVE_GEOS
    for (auto& g : m_geoms)
        preparePolygon(g, table.spatialRef());
#endif
}


#ifdef PDAL_HAVE_GEOS
GEOSGeometry *CropFilter::validatePolygon(const std::string& poly)
{
    GEOSGeometry *geom = GEOSGeomFromWKT_r(m_geosEnvironment, poly.c_str());
    if (!geom)
    {
        std::ostringstream oss;
        oss << "Invalid polygon specification for " << getName() <<
            ": " << poly << ".";
        throw pdal_error(oss.str());
    }

    int gtype = GEOSGeomTypeId_r(m_geosEnvironment, geom);
    if (gtype != GEOS_POLYGON && gtype != GEOS_MULTIPOLYGON)
    {
        std::ostringstream oss;
        oss << "Invalid polygon type for " << getName() << ": " <<
            poly << ".  Must be POLYGON or MULTIPOLYGON.";
        throw pdal_error(oss.str());
    }

    if (!GEOSisValid_r(m_geosEnvironment, geom))
    {
        char *reason = GEOSisValidReason_r(m_geosEnvironment, geom);
        std::ostringstream oss;
        oss << "WKT representation of (multi)polygon '" << poly <<
            "' invalid: " << reason << ".";
        GEOSFree_r(m_geosEnvironment, reason);
        throw pdal_error(oss.str());
    }
    return geom;
}


void CropFilter::preparePolygon(GeomPkg& g, const SpatialReference& to)
{
    char* out_wkt = GEOSGeomToWKT_r(m_geosEnvironment, g.m_geom);
    std::string poly(out_wkt);
    poly = transformWkt(poly, m_assignedSRS, to);
    log()->get(LogLevel::Debug2) << "Ingested WKT for filters.crop: " <<
        poly <<std::endl;
    GEOSFree_r(m_geosEnvironment, out_wkt);

    // Store transformed geometry.
    GEOSGeom_destroy_r(m_geosEnvironment, g.m_geom);
    g.m_geom = GEOSGeomFromWKT_r(m_geosEnvironment, poly.c_str());
    g.m_prepGeom = GEOSPrepare_r(m_geosEnvironment, g.m_geom);
    if (!g.m_prepGeom)
        throw pdal_error("unable to prepare geometry for index-accelerated "
            "intersection");
}
#endif


Options CropFilter::getDefaultOptions()
{
    Options options;
    options.add("bounds", BOX2D(), "bounds to crop to");
    options.add("polygon", std::string(""), "WKT POLYGON() string to "
        "use to filter points");
    options.add("inside", true, "Keep points that are inside or outside "
        "the given polygon");

    return options;
}


PointViewSet CropFilter::run(PointViewPtr view)
{
    PointViewSet viewSet;

    // Don't do anything if no bounds have been specified.
    if (m_geoms.empty() && m_bounds.empty())
    {
        viewSet.insert(view);
        return viewSet;
    }

#ifdef PDAL_HAVE_GEOS
    for (const auto& geom : m_geoms)
    {
        PointViewPtr outView = view->makeNew();
        crop(geom, *view, *outView);
        viewSet.insert(outView);
    }
#endif
    for (auto& box : m_bounds)
    {
        PointViewPtr outView = view->makeNew();
        crop(box, *view, *outView);
        viewSet.insert(outView);
    }
    return viewSet;
}

void CropFilter::crop(const BOX2D& box, PointView& input, PointView& output)
{
    for (PointId idx = 0; idx < input.size(); ++idx)
    {
        double x = input.getFieldAs<double>(Dimension::Id::X, idx);
        double y = input.getFieldAs<double>(Dimension::Id::Y, idx);

        bool contained = box.contains(x, y);
        if (m_cropOutside != box.contains(x, y))
            output.appendPoint(input, idx);
    }
}

#ifdef PDAL_HAVE_GEOS
GEOSGeometry *CropFilter::createPoint(double x, double y, double z)
{
    // precise filtering based on the geometry
    GEOSCoordSequence* coords = GEOSCoordSeq_create_r(m_geosEnvironment, 1, 3);
    if (!coords)
        throw pdal_error("Unable to allocate coordinate sequence");

    if (!GEOSCoordSeq_setX_r(m_geosEnvironment, coords, 0, x))
        throw pdal_error("unable to set x for coordinate sequence");
    if (!GEOSCoordSeq_setY_r(m_geosEnvironment, coords, 0, y))
        throw pdal_error("unable to set y for coordinate sequence");
    if (!GEOSCoordSeq_setZ_r(m_geosEnvironment, coords, 0, z))
        throw pdal_error("unable to set z for coordinate sequence");
    GEOSGeometry* p = GEOSGeom_createPoint_r(m_geosEnvironment, coords);
    if (!p)
        throw pdal_error("unable to allocate candidate test point");
    return p;
}



void CropFilter::crop(const GeomPkg& g, PointView& input, PointView& output)
{
    bool logOutput = (log()->getLevel() > LogLevel::Debug4);
    if (logOutput)
        log()->floatPrecision(8);

    for (PointId idx = 0; idx < input.size(); ++idx)
    {
        double x = input.getFieldAs<double>(Dimension::Id::X, idx);
        double y = input.getFieldAs<double>(Dimension::Id::Y, idx);
        double z = input.getFieldAs<double>(Dimension::Id::Z, idx);

        if (logOutput)
        {
            log()->floatPrecision(10);
            log()->get(LogLevel::Debug5) << "input: " << x << " y: " << y <<
                " z: " << z << std::endl;
        }

        GEOSGeometry *p = createPoint(x, y, z);
        bool covers = (bool)(GEOSPreparedCovers_r(m_geosEnvironment,
            g.m_prepGeom, p));
        if (m_cropOutside != covers)
            output.appendPoint(input, idx);
        GEOSGeom_destroy_r(m_geosEnvironment, p);
    }
}
#endif


void CropFilter::done(PointTableRef /*table*/)
{
#ifdef PDAL_HAVE_GEOS
    for (auto& g : m_geoms)
    {
        GEOSPreparedGeom_destroy_r(m_geosEnvironment, g.m_prepGeom);
        GEOSGeom_destroy_r(m_geosEnvironment, g.m_geom);
    }
    if (m_geosEnvironment)
        finishGEOS_r(m_geosEnvironment);
#endif
}

} // namespace pdal
