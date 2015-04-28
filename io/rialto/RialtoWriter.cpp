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
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/Bounds.hpp>
#include <pdal/util/FileUtils.hpp>

#include "stats/StatsFilter.hpp"

#include "RialtoCommon.hpp"

#include <cstdint>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "writers.rialto",
    "Rialto Writer",
    "http://pdal.io/stages/writers.rialto.html" );

CREATE_STATIC_PLUGIN(1, 0, RialtoWriter, Writer, s_info)

namespace
{
    char* getPointData(const PointView& buf, PointId& idx)
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

    void writeHeader(
            std::string dir,
            const PointViewPtr view,
            PointTableRef table,
            const Rectangle& rect,
            int32_t xt,
            int32_t yt)
    {
        std::string filename(dir + "/header.json");
        FILE* fp = fopen(filename.c_str(), "wt");

        std::unique_ptr<StatsFilter> stats(new StatsFilter);
        BufferReader br;
        br.addView(view);
        stats->setInput(br);
        stats->prepare(table);
        stats->execute(table);

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

        fprintf(fp, "    \"numPoints\": %lu,\n",
                static_cast<unsigned long>(view->size()));

        const PointLayoutPtr layout(table.layout());
        const size_t numDims = layout->dims().size();
        fprintf(fp, "    \"dimensions\": [\n");

        size_t i = 0;
        for (const auto& dim : layout->dims())
        {
            const Dimension::Type::Enum dataType = layout->dimType(dim);
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
    }
} // anonymous namespace

std::string RialtoWriter::getName() const
{
    return s_info.name;
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

void RialtoWriter::ready(PointTableRef table)
{
    m_table = &table;

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
    m_roots[0] = new Tile(0, 0, 0, r00, m_maxLevel, *m_table, log());
    m_roots[1] = new Tile(0, 1, 0, r10, m_maxLevel, *m_table, log());
}

void RialtoWriter::write(const PointViewPtr view)
{
    const PointView& viewRef(*view.get());

    // build the tiles
    for (PointId idx = 0; idx < viewRef.size(); ++idx)
    {
        char* p = getPointData(viewRef, idx);

        double lon = viewRef.getFieldAs<double>(Dimension::Id::X, idx);
        double lat = viewRef.getFieldAs<double>(Dimension::Id::Y, idx);

        if (lon < 0)
            m_roots[0]->add(idx, p, lon, lat);
        else
            m_roots[1]->add(idx, p, lon, lat);
    }

    // dump tile info
    if (log()->getLevel() >= LogLevel::Debug)
    {
        int32_t num_levels = m_maxLevel+1;
        std::vector<int32_t> numTilesPerLevel(num_levels, 0);
        std::vector<int64_t> numPointsPerLevel(num_levels, 0);

        m_roots[0]->collectStats(numTilesPerLevel, numPointsPerLevel);
        m_roots[1]->collectStats(numTilesPerLevel, numPointsPerLevel);

        for (int32_t i=0; i < num_levels; ++i)
            log()->get(LogLevel::Debug) << "L" << i << ": " << numTilesPerLevel[i] << " tiles, " << numPointsPerLevel[i] << " points\n";
    }

    // write the tiles and the header
    m_roots[0]->write(m_filename.c_str());
    m_roots[1]->write(m_filename.c_str());

    writeHeader(
            m_filename,
            view,
            *m_table,
            m_rectangle,
            m_numTilesX,
            m_numTilesY);
}

void RialtoWriter::done(PointTableRef table)
{
    delete m_roots[0];
    delete m_roots[1];
    delete[] m_roots;
}

} // namespace pdal

