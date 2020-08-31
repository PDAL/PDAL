/******************************************************************************
* Copyright (c) 2020, Hobu Inc. <info@hobu.co>
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

#include "RasterWriter.hpp"

#include <sstream>

#include <pdal/PointView.hpp>
#include <pdal/private/gdal/Raster.hpp>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "writers.raster",
    "Write a raster.",
    "http://pdal.io/stages/writers.raster.html",
    {}
};

CREATE_STATIC_STAGE(RasterWriter, s_info)

std::string RasterWriter::getName() const
{
    return s_info.name;
}

RasterWriter::RasterWriter()
{}


RasterWriter::~RasterWriter()
{}


void RasterWriter::addArgs(ProgramArgs& args)
{
    args.add("filename", "Output filename", m_filename).setPositional();
    args.add("gdaldriver", "GDAL driver name", m_drivername, "GTiff");
    args.add("gdalopts", "GDAL driver options (name=value,name=value...)",
        m_options);
    args.add("rasters", "List of raster names to write as bands.", m_rasterNames);
    args.add("data_type", "Data type for output grid (\"int8\", \"uint64\", "
        "\"float\", etc.)", m_dataType, Dimension::Type::Double);
    // Nan is a sentinal value to say that no value was set for nodata.
    args.add("nodata", "No data value", m_noData,
        std::numeric_limits<double>::quiet_NaN());
}


void RasterWriter::write(const PointViewPtr view)
{
    // If we're using the default raster, check if this view has one and use it unless we
    // already have one.
    if (m_rasterNames.empty())
    {
        if (m_rasters.empty() && view->raster())
            m_rasters.push_back(view->raster());
    }
    else
    {
        for (std::string& name : m_rasterNames)
        {
            Rasterd *r = view->raster(name);
            if (r)
                m_rasters.push_back(r);
        }
    }
}

void RasterWriter::done(PointTableRef table)
{
    if (m_rasters.empty())
        return;

    if (m_rasterNames.size() == 1 && m_rasterNames[0] == "")
        m_rasterNames.clear();

    for (const std::string& name : m_rasterNames)
        if (std::find_if(m_rasters.begin(), m_rasters.end(),
            [name](const Rasterd *r){return r->name() == name;}) == m_rasters.end())
        {
            throwError("Raster '" + name + "' not found.");
        }

    // Stick rasters whose limits match the first raster in our final raster list.
    std::vector<Rasterd *> rasters;
    for (Rasterd *r : m_rasters)
    {
        if (r->limits() != m_rasters.front()->limits())
        {
            log()->get(LogLevel::Error) << getName() << ": Ignoring raster '" <<
                r->name() << "'.  Raster limits don't match that of raster '" <<
                    m_rasters.front()->name() << "'." << std::endl;
            continue;
        }
        rasters.push_back(r);
    }

    std::array<double, 6> pixelToPos;
    RasterLimits limits = rasters.front()->limits();
    pixelToPos[0] = limits.xOrigin;
    pixelToPos[1] = limits.edgeLength;
    pixelToPos[2] = 0;
    pixelToPos[3] = limits.yOrigin + (limits.edgeLength * limits.height);
    pixelToPos[4] = 0;
    pixelToPos[5] = -limits.edgeLength;
    gdal::Raster rasterFile(m_filename, m_drivername, table.anySpatialReference(), pixelToPos);

    gdal::GDALError err = rasterFile.open(limits.width, limits.height,
        rasters.size(), m_dataType, m_noData, m_options);

    if (err != gdal::GDALError::None)
        throwError(rasterFile.errorMsg());
    int bandNum = 1;

    for (Rasterd *r : rasters)
    {
        err = rasterFile.writeBand(r->begin(), r->initializer(), bandNum++, r->name());
        if (err != gdal::GDALError::None)
            throwError(rasterFile.errorMsg());
    }

    getMetadata().addList("filename", m_filename);
}

} // namespace pdal
