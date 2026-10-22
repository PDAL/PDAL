/******************************************************************************
* Copyright (c) 2026, Hobu, Inc. (howard@hobu.co)
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

#include "StacIndexKernel.hpp"

#include <kernels/tindex/TIndexCommon.hpp>

namespace pdal
{

static PluginInfo const s_info
{
    "kernels.stac-index",
    "STAC tile index kernel",
    "https://pdal.org/apps/tindex.html"
};

CREATE_SHARED_KERNEL(StacIndexKernel, s_info);

StacIndexKernel::StacIndexKernel() : TIndexKernel()
{}

void StacIndexKernel::addExtraSwitches(ProgramArgs& args,
    const std::string& subcommand)
{
    if (subcommand == "create")
    {
        args.add("pc_type", "Pointcloud type for STAC generation (lidar, "
            "eopc, radar, sonar, other).", m_pcType, "lidar");
        args.add("statistics", "Show stats on all points for STAC generation "
            "(reads entire dataset)", m_showStats);
        args.add("static_fields", "JSON object of field names and values to include "
            "in tile index; specified as { \"fieldName\": value, ...}", m_staticFields);
    }
}

void StacIndexKernel::validateExtraSwitches(ProgramArgs& args)
{
#ifndef PDAL_HAVE_OGR_PARQUET
    throw pdal_error("Can't output to STAC-GeoParquet; GDAL was "
        "built without the OGR Parquet driver.");
#endif
    if (args.set("ogrdriver") && m_driverName != "Parquet")
        throw pdal_error("Can't specify OGR driver when outputting "
            "to STAC GeoParquet.");
    if (args.set("t_srs") && m_tgtSrsString != "EPSG:4326")
        throw pdal_error("Can't specify target SRS when outputting "
            "to STAC GeoParquet; must be EPSG:4326.");
    if (args.set("tindex_name"))
        throw pdal_error("Can't specify tile index column name when "
            "outputting to STAC GeoParquet.");
    // If the static fields arg was a file, read it into m_staticFields
    if (args.set("static_fields"))
    {
        if (FileUtils::fileExists(m_staticFields))
            m_staticFields = FileUtils::readFileIntoString(
                m_staticFields);
    }
    m_args->lcOptions.push_back("WRITE_COVERING_BBOX=YES");
}

void StacIndexKernel::create()
{
    m_tindex.reset(new tindex::StacIndexBuilder(*m_args, m_pcType, m_showStats,
        m_staticFields));
    try
    {
        m_tindex->create(m_files, m_manager);
    }
    catch(const tindex::TIndexError& e)
    {
        throw pdal_error(e.what());
    }
}

} // namespace pdal