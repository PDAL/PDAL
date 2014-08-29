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

#include <pdal/filters/Crop.hpp>

#include <pdal/PointBuffer.hpp>
#include <sstream>
#include <cstdarg>

namespace pdal
{

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

namespace filters
{

Crop::Crop(const Options& options) : pdal::Filter(options)
{
    m_cropOutside = false;
    m_geosEnvironment = 0;
    m_geosGeometry = 0;
    m_geosPreparedGeometry = 0;
}


void Crop::processOptions(const Options& options)
{
    m_bounds =
        options.getValueOrDefault<Bounds<double>>("bounds", Bounds<double>());
    m_cropOutside = options.getValueOrDefault<bool>("outside", false);
    m_poly = options.getValueOrDefault<std::string>("polygon", "");
}


void Crop::ready(PointContext ctx)
{
#ifdef PDAL_HAVE_GEOS
    if (!m_poly.empty())
    {
        m_geosEnvironment = initGEOS_r(pdal::geos::_GEOSWarningHandler,
            pdal::geos::_GEOSErrorHandler);
        m_geosGeometry = GEOSGeomFromWKT_r(m_geosEnvironment, m_poly.c_str());
        if (!m_geosGeometry)
            throw pdal_error("unable to import polygon WKT");

        int gtype = GEOSGeomTypeId_r(m_geosEnvironment, m_geosGeometry);
        if (!(gtype == GEOS_POLYGON || gtype == GEOS_MULTIPOLYGON))
            throw pdal_error("input WKT was not a POLYGON or MULTIPOLYGON");

        char* out_wkt = GEOSGeomToWKT_r(m_geosEnvironment, m_geosGeometry);
        log()->get(LogLevel::Debug2) << "Ingested WKT for filters.crop: " <<
            std::string(out_wkt) <<std::endl;
        GEOSFree_r(m_geosEnvironment, out_wkt);

        if (!GEOSisValid_r(m_geosEnvironment, m_geosGeometry))
        {
            char* reason =
                GEOSisValidReason_r(m_geosEnvironment, m_geosGeometry);
            std::ostringstream oss;
            oss << "WKT is invalid: " << std::string(reason) << std::endl;
            GEOSFree_r(m_geosEnvironment, reason);
            throw pdal_error(oss.str());
        }

        m_geosPreparedGeometry =
            GEOSPrepare_r(m_geosEnvironment, m_geosGeometry);
        if (!m_geosPreparedGeometry)
            throw pdal_error("unable to prepare geometry for "
                "index-accellerated intersection");
        m_bounds = computeBounds(m_geosGeometry);
        log()->get(LogLevel::Debug) << "Computed bounds from given WKT: " <<
            m_bounds <<std::endl;
    }
    else
    {
        log()->get(LogLevel::Debug) << "Using simple bounds for "
            "filters.crop: " << m_bounds << std::endl;
    }

#endif
}


Options Crop::getDefaultOptions()
{
    Options options;
    Option bounds("bounds",Bounds<double>(),"bounds to crop to");
    Option polygon("polygon", std::string(""),
        "WKT POLYGON() string to use to filter points");

    Option inside("inside", true, "keep points that are inside or outside "
        "the given polygon");

    options.add(inside);
    options.add(polygon);
    options.add(bounds);
    return options;
}


Bounds<double> Crop::computeBounds(GEOSGeometry const *geometry)
{
    uint32_t numInputDims;
    Bounds<double> output;

#ifdef PDAL_HAVE_GEOS
    bool bFirst(true);

    GEOSGeometry const* ring = GEOSGetExteriorRing_r(m_geosEnvironment,
        geometry);
    GEOSCoordSequence const* coords = GEOSGeom_getCoordSeq_r(m_geosEnvironment,
        ring);

    GEOSCoordSeq_getDimensions_r(m_geosEnvironment, coords, &numInputDims);
    log()->get(LogLevel::Debug) << "Inputted WKT had " << numInputDims <<
        " dimensions" <<std::endl;

    uint32_t count(0);
    GEOSCoordSeq_getSize_r(m_geosEnvironment, coords, &count);
    pdal::Vector<double> p(0.0, 0.0, 0.0);

    double x(0.0);
    double y(0.0);
    double z(0.0);
    for (unsigned i = 0; i < count; ++i)
    {
        GEOSCoordSeq_getOrdinate_r(m_geosEnvironment, coords, i, 0, &x);
        GEOSCoordSeq_getOrdinate_r(m_geosEnvironment, coords, i, 1, &y);
        if (numInputDims > 2)
            GEOSCoordSeq_getOrdinate_r(m_geosEnvironment, coords, i, 2, &z);
        p.set(0, x);
        p.set(1, y);
        if (numInputDims > 2)
            p.set(2, z);
        if (bFirst)
        {
            output = Bounds<double>(p, p);
            bFirst = false;
        }
        output.grow(p);
    }
#else
    boost::ignore_unused_variable_warning(geometry);
#endif
    return output;
}


PointBufferSet Crop::run(PointBufferPtr buffer)
{
    PointBufferSet pbSet;
    PointBufferPtr output = buffer->makeNew();
    crop(*buffer, *output);
    pbSet.insert(output);
    return pbSet;
}


void Crop::crop(PointBuffer& input, PointBuffer& output)
{
    Bounds<double> buffer_bounds = input.calculateBounds();

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

        if (m_poly.empty())
        {
            // We don't have a polygon, just a bounds. Filter on that
            // by itself.
            Vector<double> p(x,y,z);

            if (!m_cropOutside && m_bounds.contains(p))
                output.appendPoint(input, idx);
        }
#ifdef PDAL_HAVE_GEOS
        else
        {
            int ret(0);

            // precise filtering based on the geometry
            GEOSCoordSequence* coords =
                GEOSCoordSeq_create_r(m_geosEnvironment, 1, 3);
            if (!coords)
                throw pdal_error("unable to allocate coordinate sequence");
            ret = GEOSCoordSeq_setX_r(m_geosEnvironment, coords, 0, x);
            if (!ret)
                throw pdal_error("unable to set x for coordinate sequence");
            ret = GEOSCoordSeq_setY_r(m_geosEnvironment, coords, 0, y);
            if (!ret)
                throw pdal_error("unable to set y for coordinate sequence");
            ret = GEOSCoordSeq_setZ_r(m_geosEnvironment, coords, 0, z);
            if (!ret)
                throw pdal_error("unable to set z for coordinate sequence");

            GEOSGeometry* p = GEOSGeom_createPoint_r(m_geosEnvironment, coords);
            if (!p)
                throw pdal_error("unable to allocate candidate test point");

            if (static_cast<bool>(GEOSPreparedContains_r(m_geosEnvironment,
                m_geosPreparedGeometry, p)) != m_cropOutside)
                output.appendPoint(input, idx);
            GEOSGeom_destroy_r(m_geosEnvironment, p);
        }
#endif
    }
}


void Crop::done(PointContext ctx)
{
#ifdef PDAL_HAVE_GEOS
    if (m_geosPreparedGeometry)
        GEOSPreparedGeom_destroy_r(m_geosEnvironment, m_geosPreparedGeometry);

    if (m_geosGeometry)
        GEOSGeom_destroy_r(m_geosEnvironment, m_geosGeometry);

    if (m_geosEnvironment)
        finishGEOS_r(m_geosEnvironment);
#endif
}


} // namespace filters
} // namespace pdal
