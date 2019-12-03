/******************************************************************************
* Copyright (c) 2017, Hobu Inc., info@hobu.co
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

#include "OverlayFilter.hpp"

#include <vector>

#include <pdal/GDALUtils.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.overlay",
    "Assign values to a dimension based on the extent of an OGR-readable data "
        " source or an OGR SQL query.",
    "http://pdal.io/stages/filters.overlay.html"
};

CREATE_STATIC_STAGE(OverlayFilter, s_info)


void OverlayFilter::addArgs(ProgramArgs& args)
{
    args.add("dimension", "Dimension on which to filter", m_dimName).
        setPositional();
    args.add("datasource", "OGR-readable datasource for Polygon or "
        "Multipolygon data", m_datasource).setPositional();
    args.add("column", "OGR datasource column from which to "
        "read the attribute.", m_column);
    args.add("query", "OGR SQL query to execute on the "
        "datasource to fetch geometry and attributes", m_query);
    args.add("layer", "Datasource layer to use", m_layer);
}


void OverlayFilter::initialize()
{
    gdal::registerDrivers();
}


void OverlayFilter::prepared(PointTableRef table)
{
    m_dim = table.layout()->findDim(m_dimName);
    if (m_dim == Dimension::Id::Unknown)
        throwError("Dimension '" + m_dimName + "' not found.");
}


void OverlayFilter::ready(PointTableRef table)
{
    m_ds = OGRDSPtr(OGROpen(m_datasource.c_str(), 0, 0),
            [](void *p){ if (p) ::OGR_DS_Destroy(p); });
    if (!m_ds)
        throwError("Unable to open data source '" + m_datasource + "'");

    if (m_layer.size())
        m_lyr = OGR_DS_GetLayerByName(m_ds.get(), m_layer.c_str());
    else if (m_query.size())
        m_lyr = OGR_DS_ExecuteSQL(m_ds.get(), m_query.c_str(), 0, 0);
    else
        m_lyr = OGR_DS_GetLayer(m_ds.get(), 0);

    if (!m_lyr)
        throwError("Unable to select layer '" + m_layer + "'");

    auto featureDeleter = [](void *p)
    {
        if (p)
            ::OGR_F_Destroy(p);
    };
    OGRFeaturePtr feature = OGRFeaturePtr(OGR_L_GetNextFeature(m_lyr),
        featureDeleter);

    int field_index(1); // default to first column if nothing was set
    if (m_column.size())
    {
        field_index = OGR_F_GetFieldIndex(feature.get(), m_column.c_str());
        if (field_index == -1)
            throwError("No column name '" + m_column + "' was found.");
    }

    do
    {
        OGRGeometryH geom = OGR_F_GetGeometryRef(feature.get());
        int32_t fieldVal = OGR_F_GetFieldAsInteger(feature.get(), field_index);

        m_polygons.push_back(
            { Polygon(geom, table.anySpatialReference()), fieldVal} );

        feature = OGRFeaturePtr(OGR_L_GetNextFeature(m_lyr), featureDeleter);
    }
    while (feature);
}


void OverlayFilter::spatialReferenceChanged(const SpatialReference& srs)
{
    if (srs.empty())
        return;
    for (auto& poly : m_polygons)
    {
        try
        {
            poly.geom.transform(srs);
        }
        catch (pdal_error& err)
        {
            throwError(err.what());
        }
    }
}


bool OverlayFilter::processOne(PointRef& point)
{
    for (const auto& poly : m_polygons)
    {
        double x = point.getFieldAs<double>(Dimension::Id::X);
        double y = point.getFieldAs<double>(Dimension::Id::Y);
        if (poly.geom.contains(x, y))
            point.setField(m_dim, poly.val);
    }
    return true;
}


void OverlayFilter::filter(PointView& view)
{
    PointRef point(view, 0);
    for (PointId id = 0; id < view.size(); ++id)
    {
        point.setPointId(id);
        processOne(point);
    }
}

} // namespace pdal

