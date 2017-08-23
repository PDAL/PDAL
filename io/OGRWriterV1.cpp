/******************************************************************************
* Copyright (c) 2017, Hobu Inc. <info@hobu.co>
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
*     * Neither the name of Hobu, Inc. nor the names of its contributors
*       may be used to endorse or promote products derived from this
*       software without specific prior written permission.
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

#include "OGRWriter.hpp"

#include <sstream>

#include <pdal/GDALUtils.hpp>
#include <pdal/PointView.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/util/FileUtils.hpp>

#include <ogr_core.h>
#include <ogrsf_frmts.h>

#ifndef PDAL_GDAL2_1

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
        "writers.ogr",
        "Write a point cloud as a set of OGR points/multipoints",
        "http://pdal.io/stages/writers.ogr.html");


CREATE_STATIC_PLUGIN(1, 0, OGRWriter, Writer, s_info);

OGRWriter::OGRWriter() : m_driver(nullptr), m_ds(nullptr), m_layer(nullptr),
    m_feature(nullptr)
{}


std::string OGRWriter::getName() const
{
    return s_info.name;
}


void OGRWriter::addArgs(ProgramArgs& args)
{
    args.add("filename", "Output filename", m_filename).setPositional();
    args.add("multicount", "Group 'multicount' points into a structure",
        m_multiCount, (size_t)1);
    args.add("measure_dim", "Use dimensions as a measure value",
        m_measureDimName);
    args.add("ogrdriver", "OGR writer driver name", m_driverName, m_driverName);
}


void OGRWriter::initialize()
{
    gdal::registerDrivers();
    if (m_multiCount < 1)
        throwError("'m_multicount' must be greater than 0.");
}


void OGRWriter::prepared(PointTableRef table)
{
    if (m_measureDimName.size())
        throwError("Option 'measure_dim' not supported until GDAL "
            "version 2.1.");

    if (m_driverName.empty())
    {
        if (FileUtils::extension(m_filename) == ".geojson")
            m_driverName = "GeoJSON";
        else
            m_driverName = "ESRI Shapefile";
    }
}


void OGRWriter::readyTable(PointTableRef table)
{
    m_driver = OGRSFDriverRegistrar::GetRegistrar()->GetDriverByName(
        m_driverName.data());
    m_geomType = (m_multiCount == 1 ? wkbPoint : wkbMultiPoint);
}


void OGRWriter::readyFile(const std::string& filename,
    const SpatialReference& srs)
{
    m_outputFilename = filename;
    m_ds = m_driver->CreateDataSource(filename.data(), nullptr);
    if (!m_ds)
        throwError("Unable to open OGR datasource '" + filename + "'.\n");
    OGRSpatialReference ogrSrs(srs.getWKT().data());
    m_layer = m_ds->CreateLayer("points", &ogrSrs, m_geomType, nullptr);
    if (!m_layer)
        throwError("Can't create OGR layer for points.\n");
    m_feature = OGRFeature::CreateFeature(m_layer->GetLayerDefn());
}

void OGRWriter::writeView(const PointViewPtr view)
{
    m_curCount = 0;
    PointRef point(*view, 0);
    for (PointId idx = 0; idx < view->size(); ++idx)
    {
        point.setPointId(idx);
        processOne(point);
    }
}


bool OGRWriter::processOne(PointRef& point)
{
    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);
    double z = point.getFieldAs<double>(Dimension::Id::Z);
    double m = point.getFieldAs<double>(m_measureDim);

    OGRPoint pt(x, y, z);
    m_curCount++;

    if (m_multiCount > 1)
        m_multiPoint.addGeometry(&pt);
    if (m_curCount == m_multiCount)
    {
        if (m_multiCount > 1)
        {
            m_feature->SetGeometry(&m_multiPoint);
            m_multiPoint.empty();
        }
        else
            m_feature->SetGeometry(&pt);
        if (m_layer->CreateFeature(m_feature))
            throwError("Couldn't create feature.");
        m_curCount = 0;
    }
    return true;
}


void OGRWriter::doneFile()
{
    if (m_curCount)
        if (m_layer->CreateFeature(m_feature))
            throwError("Couldn't create feature.");
    OGRFeature::DestroyFeature(m_feature);
    GDALClose(m_ds);
    m_layer = nullptr;
    m_ds = nullptr;
}

} // namespace pdal

#endif // PDAL_GDAL2_1
