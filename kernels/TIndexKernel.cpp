/******************************************************************************
* Copyright (c) 2015, Howard Butler (howard@hobu.co)
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

#include "TIndexKernel.hpp"

#include <memory>
#include <vector>

#include <pdal/PDALUtils.hpp>
#include <pdal/Polygon.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>
#include <pdal/private/gdal/SpatialRef.hpp>
#include <kernels/private/tindex/TileIndex.hpp>
#include <kernels/private/tindex/TIndexError.hpp>
#include <kernels/private/tindex/StacIndex.hpp>

#include "../io/LasWriter.hpp"

namespace pdal
{

static StaticPluginInfo const s_info
{
    "kernels.tindex",
    "TIndex Kernel",
    "https://pdal.org/apps/tindex.html"
};

CREATE_STATIC_KERNEL(TIndexKernel, s_info)

std::string TIndexKernel::getName() const { return s_info.name; }

TIndexKernel::TIndexKernel() : SubcommandKernel()
    , m_srsColumnName("srs")
    , m_args(new tindex::Args())
    , m_maxFieldSize(0)
{}


StringList TIndexKernel::subcommands() const
{
    return { "create", "merge" };
}


void TIndexKernel::addSubSwitches(ProgramArgs& args,
    const std::string& subcommand)
{
    if (subcommand == "create")
    {
        args.add("tindex", "OGR-readable/writeable tile index output",
            m_args->idxFilename).setPositional();
        args.add("glob", "Pattern of files to index",
            m_filespec).setOptionalPositional();
        args.addSynonym("glob", "filespec");
        args.add("filelist", "Text file containing list of files to index", m_listfile);
        args.add("fast_boundary", "Use extent instead of exact boundary",
            m_args->fastBoundary);
        args.add("lyr_name", "OGR layer name to write into datasource",
            m_args->layerName);
        args.add("tindex_name", "Tile index column name", m_tileIndexColumnName,
            "location");
        args.add("stac-geoparquet", "Whether to write tile index as a STAC "
            "GeoParquet file", m_writeStacGeoparquet);
        args.add("pc_type", "Pointcloud type for STAC generation (lidar, "
            "eopc, radar, sonar, other).", m_pcType, "lidar");
        args.add("statistics", "Show stats on all points for STAC generation "
            "(reads entire dataset)", m_showStats);
        args.add("ogrdriver,f", "OGR driver name to use ", m_driverName,
            "ESRI Shapefile");
        args.add("lco", "Driver-specific NAME=VALUE OGR layer creation options",
            m_args->lcOptions);
        args.add("t_srs", "Target SRS of tile index", m_tgtSrsString,
            "EPSG:4326");
        args.add("a_srs", "Assign SRS of tile with no SRS to this value",
            m_assignSrsString, "EPSG:4326");
        args.add("write_absolute_path",
            "Write absolute rather than relative file paths", m_args->absPath);
        args.add("stdin,s", "Read filespec pattern from standard input",
            m_usestdin);
        args.add("path_prefix", "Prefix to be added to file paths when writing "
            "output", m_args->pathPrefix);
        args.add("threads", "Number of threads to use for file boundary creation",
            m_args->numThreads, 1);
        args.addSynonym("threads", "requests");
        args.add("skip_different_srs", "Reject files to be indexed with "
            "different SRS values", m_args->skipMultiSrs);
        args.add("simplify", "Simplify the file's exact boundary", m_args->doSmooth,
            true);
        args.addSynonym("simplify", "smooth");
        args.add("threshold", "Number of points a cell must contain to be "
            "declared positive space, when creating exact boundaries", m_args->density,
            15);
        args.add("resolution", "cell edge length to be used when creating exact "
            "boundaries", m_args->edgeLength);
        args.addSynonym("resolution", "edge_length");
        args.add("sample_size", "Sample size for auto-edge length calculation in "
            "internal hexbin filter (exact boundary)", m_args->sampleSize, 5000U);
        args.add("where", "Expression describing points to be processed for exact "
            "boundary creation", m_args->boundaryExpr);
    }
    else if (subcommand == "merge")
    {
        args.add("tindex", "OGR-readable/writeable tile index output",
            m_args->idxFilename).setPositional();
        args.add("filespec", "Output filename",
            m_filespec).setPositional();
        args.add("lyr_name", "OGR layer name to write into datasource",
            m_args->layerName);
        args.add("tindex_name", "Tile index column name", m_tileIndexColumnName,
            "location");
        args.add("ogrdriver,f", "OGR driver name to use ", m_driverName,
            "ESRI Shapefile");
        args.add("bounds", "Extent (in XYZ) to clip output to", m_bounds);
        args.add("polygon", "Well-known text of polygon to clip output", m_args->wkt);
        args.add("t_srs", "Spatial reference of the clipping geometry",
            m_tgtSrsString, "EPSG:4326");
    }
}


void TIndexKernel::validateSwitches(ProgramArgs& args)
{
    if (m_subcommand == "merge")
    {
        if (!m_args->wkt.empty() && !m_bounds.empty())
            throw pdal_error("Can't specify both 'polygon' and "
                "'bounds' options.");
        if (!m_bounds.empty())
            m_args->wkt = m_bounds.toWKT();
    }
    else
    {
        int argc = static_cast<int>(!m_filespec.empty()) +
            static_cast<int>(!m_listfile.empty()) + static_cast<int>(m_usestdin);
        if (argc > 1)
            throw pdal_error("Can't specify more than one of --glob, "
                "--filelist or --stdin.");
        if (!argc)
            throw pdal_error("Must specify either --glob, --filelist or"
                " --stdin.");
        if (m_args->pathPrefix.size() && m_args->absPath)
            throw pdal_error("Can't specify both --write_absolute_path and "
                "--path_prefix options.");
        if (m_writeStacGeoparquet)
        {
            if (args.set("ogrdriver") && m_driverName != "Parquet")
                throw pdal_error("Can't specify OGR driver when outputting "
                    "to STAC GeoParquet.");
            if (args.set("t_srs") && m_tgtSrsString != "EPSG:4326")
                throw pdal_error("Can't specify target SRS when outputting "
                    "to STAC GeoParquet; must be EPSG:4326.");
            if (args.set("tindex_name"))
                throw pdal_error("Can't specify tile index column name when "
                    "outputting to STAC GeoParquet.");

            m_args->lcOptions.push_back("WRITE_COVERING_BBOX=YES");
        }
        if (args.set("a_srs"))
            m_args->overrideASrs = true;
        if (m_driverName == "ESRI Shapefile")
            m_maxFieldSize = 254;
    }
}


int TIndexKernel::execute()
{
    if (m_subcommand == "merge")
        mergeFile();
    else
        createFile();

    return 0;
}


StringList readSTDIN()
{
    std::string line;
    StringList output;
    while (std::getline(std::cin, line))
    {
        output.push_back(line);
    }
    return output;
}

StringList readFileList(const std::string& filename)
{
    std::istream* in = Utils::openFile(filename);
    if (!in)
        throw pdal_error("Unable to open filelist '" + filename + "'");
    std::string line;
    StringList output;
    while (std::getline(*in, line))
    {
        Utils::trim(line);
        if (!line.empty())
            output.push_back(line);
    }
    FileUtils::closeFile(in);
    return output;
}

void TIndexKernel::mergeFile()
{
    using namespace gdal;

    std::ostringstream out;

    std::vector<tindex::FileInfo> files;
    try
    {
        m_tindex.reset(new tindex::TIndexProcessor(*m_args, m_tileIndexColumnName,
            m_srsColumnName, m_driverName, m_tgtSrsString, m_assignSrsString));
        files = m_tindex->readIndex();
    }
    catch (tindex::TIndexError& e)
    {
        throw pdal_error(e.what());    
    }

    m_log->get(LogLevel::Info) << "Merge filecount: " <<
        files.size() << std::endl;

    Options cropOptions;
    if (!m_bounds.empty())
        cropOptions.add("bounds", m_bounds);
    else
        cropOptions.add("polygon", m_args->wkt);

    Stage& merge = makeFilter("filters.merge");
    size_t filecount(0);
    for (auto f : files)
    {
        Stage& reader = makeReader(f.m_filename, m_driverOverride);
        Stage *premerge = &reader;

        if (m_tgtSrsString != f.m_srs)
        {
            Options reproOptions;
            reproOptions.add("out_srs", m_tgtSrsString);
            reproOptions.add("in_srs", f.m_srs);
            Stage& repro = makeFilter("filters.reprojection", reader,
                reproOptions);
            premerge = &repro;
        }

        // WKT is set, even if we're using a bounding box for fitering, so
        // can be used as a test here.
        if (!m_args->wkt.empty())
        {
            Stage& crop = makeFilter("filters.crop", *premerge, cropOptions);
            premerge = &crop;
        }
        merge.setInput(*premerge);
    }

    Stage& writer = makeWriter(m_filespec, merge, "");
    try
    {
        (void)dynamic_cast<LasWriter &>(writer);
        Options options;
        options.add("offset_x", "auto");
        options.add("offset_y", "auto");
        options.add("offset_z", "auto");
        writer.addOptions(options);
    }
    catch (std::bad_cast&)
    {}

    ColumnPointTable table;
    writer.prepare(table);
    writer.execute(table);
}

void TIndexKernel::createFile()
{
    if (!m_usestdin && m_listfile.empty())
        m_files = Utils::glob(m_filespec);
    else if (m_listfile.size())
        m_files = readFileList(m_listfile);
    else
        m_files = readSTDIN();

    if (m_files.empty())
    {
        std::ostringstream out;
        out << "Couldn't find files to index: " << m_filespec << ".";
        throw pdal_error(out.str());
    }

    if (m_writeStacGeoparquet)
        m_tindex.reset(new tindex::StacIndexBuilder(*m_args, m_pcType, m_showStats));
    else
        m_tindex.reset(new tindex::TileIndexBuilder(*m_args, m_tileIndexColumnName,
            m_srsColumnName, m_driverName, m_tgtSrsString, m_assignSrsString));
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
