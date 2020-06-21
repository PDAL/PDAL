/******************************************************************************
* Copyright (c) 2018, Hobu Inc. (info@hobu.co)
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

#include "TileKernel.hpp"

#include <pdal/StageFactory.hpp>
#include <pdal/StageWrapper.hpp>
#include <pdal/Writer.hpp>
#include <pdal/util/FileUtils.hpp>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "kernels.tile",
    "Tile Kernel",
    "http://pdal.io/apps/tile.html"
};

CREATE_STATIC_KERNEL(TileKernel, s_info)

TileKernel::TileKernel() : m_table(10000), m_repro(nullptr)
{}


std::string TileKernel::getName() const
{
    return s_info.name;
}


void TileKernel::addSwitches(ProgramArgs& args)
{
    args.add("input,i", "Input file/path name", m_inputFile).setPositional();
    args.add("output,o", "Output filename template",
        m_outputFile).setPositional();
    args.add("length", "Edge length for cells", m_length, 1000.0);
    args.add("origin_x", "Origin in X axis for cells", m_xOrigin,
        std::numeric_limits<double>::quiet_NaN());
    args.add("origin_y", "Origin in Y axis for cells", m_yOrigin,
        std::numeric_limits<double>::quiet_NaN());
    args.add("buffer", "Size of buffer (overlap) to include around each tile",
        m_buffer);
    args.add("out_srs", "Output SRS to which points will be reprojected",
        m_outSrs);
}


void TileKernel::validateSwitches(ProgramArgs& args)
{
    m_hashPos = Writer::handleFilenameTemplate(m_outputFile);
    if (m_hashPos == std::string::npos)
        throw pdal_error("Output filename must contain a single '#' "
            "template placeholder.");
}


int TileKernel::execute()
{
    const StringList& files = FileUtils::glob(m_inputFile);
    if (files.empty())
        throw pdal_error("No input files found for path '" +
            m_inputFile + "'.");

    Readers readers;
    for (auto&& file : files)
        readers[file] = prepareReader(file);
    checkReaders(readers);
    if (m_repro)
        m_repro->prepare(m_table);
    Options opts;
    opts.add("length", m_length);
    opts.add("buffer", m_buffer);
    m_splitter.setOptions(opts);
    m_splitter.prepare(m_table);

    m_table.finalize();
    process(readers);
    StageWrapper::done(m_splitter, m_table);
    for (auto&& wp : m_writers)
        StageWrapper::done(*wp.second, m_table);
    return 0;
}


void TileKernel::checkReaders(const Readers& readers)
{
    SpatialReference tempSrs;
    SpatialReference srs;
    bool needRepro(false);

    for (auto& rp : readers)
    {
        const std::string& filename = rp.first;
        Streamable *r = rp.second;

        tempSrs = r->getSpatialReference();

        // No SRS
        if (tempSrs.empty())
        {
            if (!m_outSrs.empty())
                throw pdal_error("Can't reproject file '" + filename +
                    "' with no SRS.");
            continue;
        }

        if (srs.empty())
            srs = tempSrs;

        // Two SRSes
        if (tempSrs != srs)
        {
            if (m_outSrs.empty())
            {
                static bool warned(false);
                if (!warned)
                {
                    m_log->get(LogLevel::Warning) << "No 'out_srs' specified "
                        "and input files have multiple SRSs.  Using SRS of "
                        "first input file as output SRS." << std::endl;
                    warned = true;
                    m_outSrs = srs;
                }
            }
            needRepro = true;
        }
    }

    // Non-matching SRS and we requested reprojection
    if (!m_outSrs.empty() && srs != m_outSrs)
        needRepro = true;

    if (needRepro)
    {
        Options opts;
        opts.add("out_srs", m_outSrs);

        m_repro = dynamic_cast<Streamable *>(
            &m_manager.makeFilter("filters.reprojection", opts));
    }
}


Streamable *TileKernel::prepareReader(const std::string& filename)
{
    Stage* r = &(m_manager.makeReader(filename, ""));

    if (!r)
        throw pdal_error("Couldn't create reader for input file '" +
            filename + "'.");

    Streamable *sr = dynamic_cast<Streamable *>(r);
    if (!sr)
        throw pdal_error("Driver '" + r->getName() + "' for input file '" +
            filename + "' is not streamable.");

    sr->prepare(m_table);
    return sr;
}


// We calculate the origin specially in order to avoid a "first point"
// check for every point iteration, seeing as we might have BILLIONS
// of points to process.
void TileKernel::process(const Readers& readers)
{
    using std::placeholders::_1;
    using std::placeholders::_2;
    using std::placeholders::_3;
    SplitterFilter::PointAdder adder =
        std::bind(&TileKernel::adder, this, _1, _2, _3);

    bool haveOrigin(false);
    StageWrapper::ready(m_splitter, m_table);
    for (auto&& rp : readers)
    {
        Streamable& r = *(rp.second);
        std::vector<bool> skips(m_table.capacity());
        PointId idx(0);
        PointRef point(m_table, idx);

        StreamableWrapper::ready(r, m_table);
        if (m_repro)
            StreamableWrapper::spatialReferenceChanged(*m_repro,
                r.getSpatialReference());

        // Read first point.
        bool finished(false);
        finished = !StreamableWrapper::processOne(r, point);
        if (!haveOrigin && !finished)
        {
            if (std::isnan(m_xOrigin))
                m_xOrigin = point.getFieldAs<double>(Dimension::Id::X);
            if (std::isnan(m_yOrigin))
                m_yOrigin = point.getFieldAs<double>(Dimension::Id::Y);
            m_splitter.setOrigin(m_xOrigin, m_yOrigin);
            haveOrigin = true;
        }

        idx++;
        while (!finished)
        {
            // Read subsequent points.
            while (true)
            {
                point.setPointId(idx);
                finished = !StreamableWrapper::processOne(r, point);
                idx++;
                if (idx == m_table.capacity() || finished)
                    break;
            }
            PointId last = idx - 1;

            SpatialReference srs = r.getSpatialReference();
            if (!srs.empty())
                m_table.setSpatialReference(srs);
            // Reproject if necessary.
            if (m_repro)
            {
                for (idx = 0; idx < last; ++idx)
                {
                    point.setPointId(idx);
                    if (!StreamableWrapper::processOne(*m_repro, point))
                        skips[idx] = true;
                }
                SpatialReference srs = r.getSpatialReference();
                if (!srs.empty())
                    m_table.setSpatialReference(srs);
            }

            // Split and write.
            for (idx = 0; idx < last; ++idx)
            {
                if (skips[idx])
                    continue;

                point.setPointId(idx);
                m_splitter.processPoint(point, adder);

            }
            for (size_t i = 0; i < skips.size(); ++i)
                skips[i] = false;
            idx = 0;
        }
        StreamableWrapper::done(r, m_table);
        if (m_repro)
            StreamableWrapper::done(*m_repro, m_table);
    }
}


void TileKernel::adder(PointRef& point, int xpos, int ypos)
{
    Coord loc(xpos, ypos);

    Stage *w;
    Streamable *sw;

    auto wi = m_writers.find(loc);
    if (wi == m_writers.end())
    {
        std::string filename(m_outputFile);
        std::string xname(std::to_string(xpos));
        std::string yname(std::to_string(ypos));
        filename.replace(m_hashPos, 1, (xname + "_" + yname));

        w = &m_manager.makeWriter(filename, "");
        if (!w)
            throw pdal_error("Couldn't create writer for output file '" +
                m_outputFile + "'.");
        sw = dynamic_cast<Streamable *>(w);
        if (!sw)
            throw pdal_error("Driver '" + w->getName() + "' for input file '" +
                m_outputFile + "' is not streamable.");  
        m_writers[loc] = sw;

        sw->prepare(m_table);
        StreamableWrapper::spatialReferenceChanged(*sw, m_outSrs);
        StreamableWrapper::ready(*sw, m_table);
    }
    else
        sw = wi->second;
    StreamableWrapper::processOne(*sw, point);
}

} // namespace pdal
