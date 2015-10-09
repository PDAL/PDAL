/******************************************************************************
* Copyright (c) 2014, Howard Butler, howard@hobu.co
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

#include "AttributeFilter.hpp"

#include <memory>
#include <vector>

#include <pdal/GlobalEnvironment.hpp>
#include <pdal/GDALUtils.hpp>

#include <pdal/StageFactory.hpp>
#include <pdal/QuadIndex.hpp>

#include <ogr_geometry.h>
#include <geos_c.h>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.attribute",
    "Assign values for a dimension using a specified value, \n" \
        "an OGR-readable data source, or an OGR SQL query.",
    "http://pdal.io/stages/filters.attribute.html" );

CREATE_SHARED_PLUGIN(1, 0, AttributeFilter, Filter, s_info)

struct OGRDataSourceDeleter
{
    template <typename T>
    void operator()(T* ptr)
    {
        if (ptr)
            ::OGR_DS_Destroy(ptr);
    }
};

namespace
{

static void _GEOSErrorHandler(const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    char buf[1024];

    vsnprintf(buf, sizeof(buf), fmt, args);

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

} // unnamed namespace

struct OGRFeatureDeleter
{
    template <typename T>
    void operator()(T* ptr)
    {
        if (ptr)
            ::OGR_F_Destroy(ptr);
    }
};

void AttributeFilter::initialize()
{
    GlobalEnvironment::get().initializeGDAL(log(), isDebug());
}


Options AttributeFilter::getDefaultOptions()
{
    Options options;

    options.add("dimension", "Classification");
    options.add("datasource", "source");

    return options;
}


void AttributeFilter::processOptions(const Options& options)
{
    if (!options.hasOption("dimension"))
    {
        std::ostringstream oss;
        oss << getName() << ": option 'dimension' must be specified.";
        throw pdal_error(oss.str());
    }
    if (options.hasOption("value") && options.hasOption("datasource"))
    {
        std::ostringstream oss;
        oss << getName() << ": options 'value' and 'datasource' mutually "
            "exclusive.";
        throw pdal_error(oss.str());
    }
    if (!options.hasOption("value") && !options.hasOption("datasource"))
    {
        std::ostringstream oss;
        oss << getName() << ": Either option 'value' or 'datasource' must "
            "be specified.";
        throw pdal_error(oss.str());
    }

    std::string ops[] = { "column", "query", "layer" };
    for (auto& op : ops)
    {
        if (options.hasOption("value") && options.hasOption(op))
        {
            std::ostringstream oss;
            oss << getName() << ": option 'op' invalid with option 'value'.";
            throw pdal_error(oss.str());
        }
    }

    m_dimName = options.getValueOrDefault<std::string>("dimension");
    m_value = options.getValueOrDefault<double>("value",
        std::numeric_limits<double>::quiet_NaN());
    m_datasource = options.getValueOrDefault<std::string>("datasource");
    m_column = options.getValueOrDefault<std::string>("column");
    m_query = options.getValueOrDefault<std::string>("query");
    m_layer = options.getValueOrDefault<std::string>("layer");
}


void AttributeFilter::prepared(PointTableRef table)
{
    m_dim = table.layout()->findDim(m_dimName);
    if (m_dim == Dimension::Id::Unknown)
    {
        std::ostringstream oss;
        oss << getName() << ": Dimension '" << m_dimName << "' not found.";
        throw pdal_error(oss.str());
    }
}


void AttributeFilter::ready(PointTableRef table)
{
    if (m_value != m_value)
    {
        m_ds = OGRDSPtr(OGROpen(m_datasource.c_str(), 0, 0),
            OGRDataSourceDeleter());
        if (!m_ds)
        {
            std::ostringstream oss;
            oss << getName() << ": Unable to open data source '" <<
                    m_datasource << "'";
            throw pdal_error(oss.str());
        }
    }
}


BOX3D computeBounds(GEOSContextHandle_t ctx, GEOSGeometry const *geometry)
{
    uint32_t numInputDims;
    BOX3D output;


    GEOSGeometry const* ring = GEOSGetExteriorRing_r(ctx,
        geometry);
    GEOSCoordSequence const* coords = GEOSGeom_getCoordSeq_r(ctx,
        ring);

    GEOSCoordSeq_getDimensions_r(ctx, coords, &numInputDims);

    uint32_t count(0);
    GEOSCoordSeq_getSize_r(ctx, coords, &count);

    double x(0.0);
    double y(0.0);
    double z(0.0);
    for (unsigned i = 0; i < count; ++i)
    {
        GEOSCoordSeq_getOrdinate_r(ctx, coords, i, 0, &x);
        GEOSCoordSeq_getOrdinate_r(ctx, coords, i, 1, &y);
        if (numInputDims > 2)
            GEOSCoordSeq_getOrdinate_r(ctx, coords, i, 2, &z);
        output.grow(x, y, z);
    }
    return output;
}


GEOSGeometry* createGEOSPoint(GEOSContextHandle_t ctx,
    double x, double y, double z)
{
    int ret(0);

    // precise filtering based on the geometry
    GEOSCoordSequence* coords = GEOSCoordSeq_create_r(ctx, 1, 3);
    if (!coords)
        throw pdal_error("unable to allocate coordinate sequence");
    ret = GEOSCoordSeq_setX_r(ctx, coords, 0, x);
    if (!ret)
        throw pdal_error("unable to set x for coordinate sequence");
    ret = GEOSCoordSeq_setY_r(ctx, coords, 0, y);
    if (!ret)
        throw pdal_error("unable to set y for coordinate sequence");
    ret = GEOSCoordSeq_setZ_r(ctx, coords, 0, z);
    if (!ret)
        throw pdal_error("unable to set z for coordinate sequence");

    GEOSGeometry* p = GEOSGeom_createPoint_r(ctx, coords);
    if (!p)
        throw pdal_error("unable to allocate candidate test point");
    return p;
}


void AttributeFilter::UpdateGEOSBuffer(PointView& view)
{
    QuadIndex idx(view);

    if (m_layer.size())
        m_lyr = OGR_DS_GetLayerByName(m_ds.get(), m_layer.c_str());
    else if (m_query.size())
        m_lyr = OGR_DS_ExecuteSQL(m_ds.get(), m_query.c_str(), 0, 0);
    else
        m_lyr = OGR_DS_GetLayer(m_ds.get(), 0);

    if (!m_lyr)
    {
        std::ostringstream oss;
        oss << getName() << ": Unable to select layer '" << m_layer << "'";
        throw pdal_error(oss.str());
    }

    OGRFeaturePtr feature = OGRFeaturePtr(OGR_L_GetNextFeature(m_lyr),
        OGRFeatureDeleter());

    int field_index(1); // default to first column if nothing was set
    if (m_column.size())
    {
        field_index = OGR_F_GetFieldIndex(feature.get(), m_column.c_str());
        if (field_index == -1)
        {
            std::ostringstream oss;
            oss << getName() << ": No column name '" << m_column <<
                "' was found.";
            throw pdal_error(oss.str());
        }
    }

    while (feature)
    {
        OGRGeometryH geom = OGR_F_GetGeometryRef(feature.get());
        OGRwkbGeometryType t = OGR_G_GetGeometryType(geom);
        int32_t fieldVal = OGR_F_GetFieldAsInteger(feature.get(), field_index);

        if (!(t == wkbPolygon ||
            t == wkbMultiPolygon ||
            t == wkbPolygon25D ||
            t == wkbMultiPolygon25D))
        {
            std::ostringstream oss;
            oss << getName() << ": Geometry is not Polygon or MultiPolygon!";
            throw pdal::pdal_error(oss.str());
        }

        OGRGeometry *ogr_g = (OGRGeometry*)geom;
        if (!m_geosEnvironment)
            m_geosEnvironment = initGEOS_r(_GEOSWarningHandler,
                _GEOSErrorHandler);

        // Convert the the GDAL geom to WKB in order to avoid the version
        // context issues with exporting directoly to GEOS.
        OGRwkbByteOrder bo =
            GEOS_getWKBByteOrder() == GEOS_WKB_XDR ? wkbXDR : wkbNDR;
        int wkbSize = ogr_g->WkbSize();
        std::vector<unsigned char> wkb(wkbSize);

        ogr_g->exportToWkb(bo, wkb.data());
        GEOSGeometry *geos_g = GEOSGeomFromWKB_buf_r(m_geosEnvironment,
            wkb.data(), wkbSize);

        GEOSPreparedGeometry const* geos_pg = GEOSPrepare_r(m_geosEnvironment,
            geos_g);
        if (!geos_pg)
        {
            std::ostringstream oss;
            oss << getName() << ": unable to prepare geometry for "
                "index-accelerated intersection";
            throw pdal_error(oss.str());
        }

        // Compute a total bounds for the geometry. Query the QuadTree to
        // find out the points that are inside the bbox. Then test each
        // point in the bbox against the prepared geometry.
        BOX3D box = computeBounds(m_geosEnvironment, geos_g);
        std::vector<PointId> ids = idx.getPoints(box);
        for (const auto& i : ids)
        {
            double x = view.getFieldAs<double>(Dimension::Id::X, i);
            double y = view.getFieldAs<double>(Dimension::Id::Y, i);
            double z = view.getFieldAs<double>(Dimension::Id::Z, i);

            GEOSGeometry* p = createGEOSPoint(m_geosEnvironment, x, y ,z);

            if ((bool)(GEOSPreparedCovers_r(m_geosEnvironment, geos_pg, p)))
            {
                // We're in the poly, write the attribute value
                view.setField(m_dim, i, fieldVal);
            }
            GEOSGeom_destroy_r(m_geosEnvironment, p);
        }
        feature = OGRFeaturePtr(OGR_L_GetNextFeature(m_lyr),
            OGRFeatureDeleter());
    }
}


void AttributeFilter::filter(PointView& view)
{
    if (m_value == m_value)
        for (PointId i = 0; i < view.size(); ++i)
            view.setField(m_dim, i, m_value);
    else
        UpdateGEOSBuffer(view);
}


void AttributeFilter::done(PointTableRef /*table*/)
{
    if (m_geosEnvironment)
        finishGEOS_r(m_geosEnvironment);
    m_geosEnvironment = 0;
}

} // namespace pdal
