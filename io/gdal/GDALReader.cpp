/******************************************************************************
* Copyright (c) 2015, Howard Butler <howard@hobu.co>
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

#include "GDALReader.hpp"

#include <sstream>


#include <pdal/PointView.hpp>
#include <pdal/GlobalEnvironment.hpp>

namespace pdal
{
namespace
{



}


static PluginInfo const s_info = PluginInfo(
        "readers.gdal",
        "Read GDAL rasters as point clouds.",
        "http://pdal.io/stages/reader.gdal.html");


CREATE_STATIC_PLUGIN(1, 0, GDALReader, Reader, s_info);


std::string GDALReader::getName() const
{
    return s_info.name;
}


GDALReader::GDALReader()
{}


void GDALReader::initialize()
{
    GlobalEnvironment::get().initializeGDAL(log());
}

void GDALReader::processOptions(const Options& options)
{
    m_rasterFilename = options.getValueOrThrow<std::string>("raster");

}
void GDALReader::addDimensions(PointLayoutPtr layout)
{
//     band.m_dim = layout->registerOrAssignDim(band.m_name,
//         Dimension::defaultType(Dimension::Id::Red));
    for (auto it : m_vertexDimensions)
    {
        layout->registerDim(it.second);
    }
}


void GDALReader::ready(PointTableRef table)
{
//     m_ply = openGDAL(m_filename);
}


point_count_t GDALReader::read(PointViewPtr view, point_count_t num)
{
    return view->size();
}


void GDALReader::done(PointTableRef table)
{
}

}
