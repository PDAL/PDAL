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

struct OGRGeometryDeleter
{
    template <typename T>
    void operator()(T* ptr)
    {
        if (ptr)
            ::OGR_G_DestroyGeometry(ptr);
    }
};


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

    pdal::Option red("dimension", "Classification", "");
    pdal::Option b0("value","0", "");
    pdal::Option geometry("geometry","POLYGON ((30 10, 40 40, 20 40, 10 20, 30 10))", "");
    pdal::Option query("query","", "");
    pdal::Option layer("layer","", "");
    pdal::Option datasource("datasource","", "");
    pdal::Options redO;
    redO.add(b0);
    redO.add(geometry);
    redO.add(query);
    redO.add(layer);
    redO.add(datasource);
    red.setOptions(redO);

    options.add(red);

    return options;
}


void AttributeFilter::processOptions(const Options& options)
{
    std::vector<Option> dimensions = options.getOptions("dimension");
    for (auto i = dimensions.begin(); i != dimensions.end(); ++i)
    {
        std::string name = i->getValue<std::string>();
        boost::optional<Options const&> dimensionOptions = i->getOptions();
        if (!dimensionOptions)
        {
            std::ostringstream oss;
            oss << "No options dimension given for dimension '" <<
                name << "'";
            throw pdal_error(oss.str());
        }

        AttributeInfo info;
        info.datasource = dimensionOptions->getValueOrDefault<std::string>("datasource", "");


        // If we have no datasource, then we're simply setting a value.
        // If we have no value, throw an exception.
        if (!info.datasource.size())
        {
            info.value = dimensionOptions->getValueOrThrow<std::string>("value");
            info.isogr = false;
        } else
        {
            info.column = dimensionOptions->getValueOrDefault<std::string>("column",""); // take first column
            info.query = dimensionOptions->getValueOrDefault<std::string>("query","");
            info.layer = dimensionOptions->getValueOrDefault<std::string>("layer",""); // take first layer

        }

        m_dimensions.insert(std::make_pair(name, info));
    }
}

void AttributeFilter::ready(PointTableRef table)
{
    for (auto& dim_par : m_dimensions)
    {
        Dimension::Id::Enum t = table.layout()->findDim(dim_par.first);
        dim_par.second.dim = t;

        if (dim_par.second.isogr)
        {

            OGRDSPtr ds = OGRDSPtr(OGROpen(dim_par.second.datasource.c_str(), 0, 0), OGRDataSourceDeleter());
            if (!ds)
            {
                std::ostringstream oss;
                oss << "Unable to open data source '" << dim_par.second.datasource <<"'";
                throw pdal_error(oss.str());
            }
            dim_par.second.ds = ds;
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


GEOSGeometry* createGEOSPoint(GEOSContextHandle_t ctx, double x, double y, double z)
{
    int ret(0);

    // precise filtering based on the geometry
    GEOSCoordSequence* coords =
        GEOSCoordSeq_create_r(ctx, 1, 3);
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

void AttributeFilter::UpdateGEOSBuffer(PointView& view, AttributeInfo& info)
{
    QuadIndex idx(view);

    if (!info.lyr) // wake up the layer
    {
        if (info.layer.size())
            info.lyr = OGR_DS_GetLayerByName(info.ds.get(), info.layer.c_str());
        else if (info.query.size())
        {
            info.lyr = OGR_DS_ExecuteSQL(info.ds.get(), info.query.c_str(), 0, 0);
        }
        else
            info.lyr = OGR_DS_GetLayer(info.ds.get(), 0);
        if (!info.lyr)
        {
            std::ostringstream oss;
            oss << "Unable to select layer '" << info.layer << "'";
            throw pdal_error(oss.str());
        }
    }

    OGRFeaturePtr feature = OGRFeaturePtr(OGR_L_GetNextFeature(info.lyr), OGRFeatureDeleter());

    int field_index(1); // default to first column if nothing was set
    if (info.column.size())
    {

        field_index = OGR_F_GetFieldIndex(feature.get(), info.column.c_str());
        if (field_index == -1)
        {
            std::ostringstream oss;
            oss << "No column name '" << info.column << "' was found.";
            throw pdal_error(oss.str());
        }
    }

    while(feature)
    {
        OGRGeometryH geom = OGR_F_GetGeometryRef(feature.get());
        OGRwkbGeometryType t = OGR_G_GetGeometryType(geom);

        if (!(t == wkbPolygon ||
            t == wkbMultiPolygon ||
            t == wkbPolygon25D ||
            t == wkbMultiPolygon25D))
        {
            std::ostringstream oss;
            oss << "Geometry is not Polygon or MultiPolygon!";
            throw pdal::pdal_error(oss.str());
        }

        OGRGeometry* ogr_g = (OGRGeometry*) geom;
        GEOSGeometry* geos_g (0);
        if (!m_geosEnvironment)
        {

#if (GDAL_VERSION_MINOR < 11) && (GDAL_VERSION_MAJOR == 1)
        geos_g = ogr_g->exportToGEOS();
#else
        m_geosEnvironment = ogr_g->createGEOSContext();
        geos_g = ogr_g->exportToGEOS(m_geosEnvironment);

#endif
        }

        GEOSPreparedGeometry const* geos_pg = GEOSPrepare_r(m_geosEnvironment, geos_g);
        if (!geos_pg)
            throw pdal_error("unable to prepare geometry for index-accelerated intersection");

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

            if (static_cast<bool>(GEOSPreparedContains_r(m_geosEnvironment, geos_pg, p)))
            {
                // We're in the poly, write the attribute value
                int32_t v = OGR_F_GetFieldAsInteger(feature.get(), field_index);
                view.setField(info.dim, i, v);
//                 log()->get(LogLevel::Debug) << "Setting value: " << v << std::endl;
            }

            GEOSGeom_destroy_r(m_geosEnvironment, p);

        }

        feature = OGRFeaturePtr(OGR_L_GetNextFeature(info.lyr), OGRFeatureDeleter());
    }
}

void AttributeFilter::filter(PointView& view)
{

    for (auto& dim_par : m_dimensions)
    {
        if (dim_par.second.isogr)
        {
            UpdateGEOSBuffer(view, dim_par.second);
        }  else
        {
            for (PointId i = 0; i < view.size(); ++i)
            {
                double v = boost::lexical_cast<double>(dim_par.second.value);
                view.setField(dim_par.second.dim, i, v);
            }

        }
    }
}

} // namespace pdal
