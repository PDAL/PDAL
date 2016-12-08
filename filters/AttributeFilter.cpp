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

#include <pdal/GDALUtils.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/Polygon.hpp>
#include <pdal/QuadIndex.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "filters.attribute",
    "Assign values for a dimension using a specified value, \n" \
        "an OGR-readable data source, or an OGR SQL query.",
    "http://pdal.io/stages/filters.attribute.html" );

CREATE_STATIC_PLUGIN(1, 0, AttributeFilter, Filter, s_info)

struct OGRDataSourceDeleter
{
    template <typename T>
    void operator()(T* ptr)
    {
        if (ptr)
            ::OGR_DS_Destroy(ptr);
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


void AttributeFilter::addArgs(ProgramArgs& args)
{
    args.add("dimension", "Dimension on which to filter", m_dimName).
        setPositional();
    m_valArg = &args.add("value", "Value to set on matching points", m_value,
        std::numeric_limits<double>::quiet_NaN());
    m_dsArg = &args.add("datasource", "OGR-readable datasource for Polygon or "
        "Multipolygon data", m_datasource);
    m_colArg = &args.add("column", "OGR datasource column from which to "
        "read the attribute.", m_column);
    m_queryArg = &args.add("query", "OGR SQL query to execute on the "
        "datasource to fetch geometry and attributes", m_query);
    m_layerArg = &args.add("layer", "Datasource layer to use", m_layer);
}


void AttributeFilter::initialize()
{
    if (m_valArg->set() && m_dsArg->set())
    {
        std::ostringstream oss;
        oss << getName() << ": options 'value' and 'datasource' mutually "
            "exclusive.";
        throw pdal_error(oss.str());
    }

    if (!m_valArg->set() && !m_dsArg->set())
    {
        std::ostringstream oss;
        oss << getName() << ": Either option 'value' or 'datasource' must "
            "be specified.";
        throw pdal_error(oss.str());
    }

    Arg *args[] = { m_colArg, m_queryArg, m_layerArg };
    for (auto& a : args)
    {
        if (m_valArg->set() && a->set())
        {
            std::ostringstream oss;
            oss << getName() << ": option '" << a->longname() << "' invalid "
                "with option 'value'.";
            throw pdal_error(oss.str());
        }
    }
    gdal::registerDrivers();
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

        pdal::Polygon p(geom, view.spatialReference());

        // Compute a total bounds for the geometry. Query the QuadTree to
        // find out the points that are inside the bbox. Then test each
        // point in the bbox against the prepared geometry.
        BOX3D box = p.bounds();
        std::vector<PointId> ids = idx.getPoints(box);


        for (const auto& i : ids)
        {
            PointRef ref(view, i);
            if (p.covers(ref))
                view.setField(m_dim, i, fieldVal);
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

} // namespace pdal

