/******************************************************************************
* Copyright (c) 2014-2015, RadiantBlue Technologies, Inc.
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

#include "RialtoWriter.hpp"

#include <pdal/BufferReader.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/Options.hpp>
#include <pdal/pdal_error.hpp>
#include <pdal/pdal_types.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/PointContext.hpp>
#include <pdal/util/Bounds.hpp>
#include <pdal/util/FileUtils.hpp>

#include "stats/StatsFilter.hpp"

#include "RialtoCommon.hpp"

#include <cstdint>

namespace pdal
{

char* getPointData(const PointBuffer& buf, PointId& idx)
{
    char* p = new char[buf.pointSize()];
    char* q = p;

    for (const auto& dim : buf.dims())
    {
        buf.getRawField(dim, idx, q);
        q += buf.dimSize(dim);
    }

    return p;
}

void writeHeader(const char* dir, const PointBuffer& buf, PointContextRef ctx, const Rectangle& rect, int32_t xt, int32_t yt)
{
    char* filename = new char[strlen(dir) + 64];
    filename[0] = 0;
    strcat(filename, dir);
    strcat(filename, "/");
    strcat(filename, "header.json");

    FILE* fp = fopen(filename, "wt");

    std::unique_ptr<StatsFilter> stats(new StatsFilter);
    std::unique_ptr<BufferReader> br(new BufferReader);
    PointBufferPtr input_buffer = std::make_shared<PointBuffer>(buf);
    br->addBuffer(input_buffer);
    stats->setInput(br.get());
    stats->prepare(ctx);
    stats->execute(ctx);

    double minx, maxx;
    double miny, maxy;
    const stats::Summary& x_stats = stats->getStats(Dimension::Id::X);
    const stats::Summary& y_stats = stats->getStats(Dimension::Id::Y);
    minx = x_stats.minimum();
    maxx = x_stats.maximum();
    miny = y_stats.minimum();
    maxy = y_stats.maximum();

    fprintf(fp, "{\n");
    fprintf(fp, "    \"version\": 3,\n");

    fprintf(fp, "    \"tilebbox\": [%f, %f, %f, %f],\n",
            rect.m_west,
            rect.m_south,
            rect.m_east,
            rect.m_north);

    fprintf(fp, "    \"numTilesX\": %d,\n", xt);
    fprintf(fp, "    \"numTilesY\": %d,\n", yt);

    fprintf(fp, "    \"databbox\": [%f, %f, %f, %f],\n",
            minx, miny, maxx, maxy);

    fprintf(fp, "    \"numPoints\": %lu,\n", buf.size());

    const size_t numDims = ctx.dims().size();
    fprintf(fp, "    \"dimensions\": [\n");

    size_t i = 0;
    for (const auto& dim : ctx.dims())
    {
        const Dimension::Type::Enum dataType = ctx.dimType(dim);
        std::string dataTypeName = Dimension::interpretationName(dataType);
        std::string name = Dimension::name(dim);

        double mind, meand, maxd;
        const stats::Summary& d_stats = stats->getStats(dim);
        mind = d_stats.minimum();
        meand = d_stats.average();
        maxd = d_stats.maximum();

        fprintf(fp, "        {\n");
        fprintf(fp, "            \"datatype\": \"%s\",\n", dataTypeName.c_str());
        fprintf(fp, "            \"name\": \"%s\",\n", name.c_str());
        fprintf(fp, "            \"min\": %f,\n", mind);
        fprintf(fp, "            \"mean\": %f,\n", meand);
        fprintf(fp, "            \"max\": %f\n", maxd);
        fprintf(fp, "        }%s\n", i++==numDims-1 ? "" : ",");
    }
    fprintf(fp, "    ]\n");
    fprintf(fp, "}\n");

    fclose(fp);

    delete[] filename;
}

void RialtoWriter::processOptions(const Options& options)
{
    m_maxLevel = options.getValueOrDefault<int32_t>("max_level", 16);
    m_overwrite = options.getValueOrDefault<bool>("overwrite", false);
}

Options RialtoWriter::getDefaultOptions()
{
    Options options;
    options.add("max_level", 16, "Max number of levels");
    options.add("overwrite", false, "Overwrite existing files?");
    return options;
}

void RialtoWriter::ready(PointContextRef ctx)
{
    m_context = ctx;

    if (FileUtils::directoryExists(m_filename))
    {
        if (!m_overwrite)
            throw pdal_error("RialtoWriter: Requested directory already exists. "\
                "Use writers.rialto.overwrite to delete the existing directory.\n");
        else
            FileUtils::deleteDirectory(m_filename);
    }

    if (!FileUtils::createDirectory(m_filename))
        throw pdal_error("RialtoWriter: Error creating directory.\n");

    m_numTilesX = 2;
    m_numTilesY = 1;

    m_rectangle = Rectangle(-180, -90, 180, 90);

    Rectangle r00(-180, -90, 0, 90);
    Rectangle r10(0, -90, 180, 90);
    m_roots = new Tile*[2];
    m_roots[0] = new Tile(0, 0, 0, r00, m_maxLevel, m_context, log());
    m_roots[1] = new Tile(0, 1, 0, r10, m_maxLevel, m_context, log());
}

void RialtoWriter::write(const PointBuffer& buf)
{
    // build the tiles
    for (PointId idx = 0; idx < buf.size(); ++idx)
    {
        char* p = getPointData(buf, idx);

        double lon = buf.getFieldAs<double>(Dimension::Id::X, idx);
        double lat = buf.getFieldAs<double>(Dimension::Id::Y, idx);

        if (lon < 0)
            m_roots[0]->add(idx, p, lon, lat);
        else
            m_roots[1]->add(idx, p, lon, lat);
    }

    // dump tile info
    if (log()->getLevel() >= LogLevel::Debug)
    {
        int32_t numTilesPerLevel[m_maxLevel+1];
        int64_t numPointsPerLevel[m_maxLevel+1];

        for (int i=0; i<=m_maxLevel; ++i)
        {
            numTilesPerLevel[i] = 0;
            numPointsPerLevel[i] = 0;
        }

        m_roots[0]->collectStats(numTilesPerLevel, numPointsPerLevel);
        m_roots[1]->collectStats(numTilesPerLevel, numPointsPerLevel);

        for (int i=0; i<=m_maxLevel; ++i)
            log()->get(LogLevel::Debug) << "L" << i << ": " << numTilesPerLevel[i] << " tiles, " << numPointsPerLevel[i] << " points\n";
    }

    // write the tiles and the header
    m_roots[0]->write(m_filename.c_str());
    m_roots[1]->write(m_filename.c_str());

    writeHeader(m_filename.c_str(), buf, m_context, m_rectangle, m_numTilesX, m_numTilesY);
}

void RialtoWriter::done(PointContextRef ctx)
{
    delete m_roots[0];
    delete m_roots[1];
    delete[] m_roots;
}

} // namespace pdal

