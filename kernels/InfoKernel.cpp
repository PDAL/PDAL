/******************************************************************************
* Copyright (c) 2013, Howard Butler (hobu.inc@gmail.com)
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

#include "InfoKernel.hpp"

#include <algorithm>

#include <pdal/pdal_config.hpp>
#include <pdal/pdal_features.hpp>

#include <filters/InfoFilter.hpp>
#include <pdal/KDIndex.hpp>
#include <pdal/PipelineWriter.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/StageFactory.hpp>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "kernels.info",
    "Info Kernel",
    "http://pdal.io/apps/info.html"
};

CREATE_STATIC_KERNEL(InfoKernel, s_info)

std::string InfoKernel::getName() const { return s_info.name; }

InfoKernel::InfoKernel() : m_showStats(false), m_showSchema(false),
    m_showAll(false), m_showMetadata(false), m_boundary(false),
    m_showSummary(false), m_needPoints(false), m_statsStage(nullptr),
    m_hexbinStage(nullptr), m_infoStage(nullptr), m_reader(nullptr)
{}


void InfoKernel::validateSwitches(ProgramArgs& args)
{
    int functions = 0;

    if (!m_usestdin && m_inputFile.empty())
        throw pdal_error("No input file specified.");

    // All isn't really all.
    if (m_showAll)
    {
        m_showStats = true;
        m_showMetadata = true;
        m_showSchema = true;
        m_boundary = true;
    }

    if (m_boundary)
    {
        functions++;
        m_needPoints = true;
    }
    if (m_queryPoint.size())
    {
        functions++;
        m_needPoints = true;
    }
    if (m_pointIndexes.size())
    {
        functions++;
        m_needPoints = true;
    }
    if (m_showSchema)
        functions++;
    if (m_showMetadata)
        functions++;
    if (m_showSummary)
        functions++;
    if (m_pipelineFile.size())
        functions++;
    if (m_showStats || functions == 0 )
    {
        functions++;
        m_showStats = true;
        m_needPoints = true;
    }

    if (m_pointIndexes.size() && m_queryPoint.size())
        throw pdal_error("'point' option incompatible with 'query' option.");

    if (m_showSummary && functions > 1)
        throw pdal_error("'summary' option incompatible with other "
            "specified options.");

    if (!m_showStats && m_enumerate.size())
        throw pdal_error("'enumerate' option requires 'stats' option.");
    if (!m_showStats && m_dimensions.size())
        throw pdal_error("'dimensions' option requires 'stats' option.");
}


void InfoKernel::addSwitches(ProgramArgs& args)
{
    args.add("input,i", "Input file name", m_inputFile).setOptionalPositional();
    args.add("all", "Dump statistics, schema and metadata", m_showAll);
    args.add("point,p", "Point to dump\n--point=\"1-5,10,100-200\" (0 indexed)",
        m_pointIndexes);
    args.add("query",
         "Return points in order of distance from the specified "
         "location (2D or 3D)\n"
         "--query Xcoord,Ycoord[,Zcoord][/count]",
         m_queryPoint);
    args.add("stats", "Dump stats on all points (reads entire dataset)",
        m_showStats);
    args.add("boundary", "Compute a hexagonal hull/boundary of dataset",
        m_boundary);
    args.add("dimensions", "Dimensions on which to compute statistics",
        m_dimensions);
    args.add("enumerate", "Dimensions whose values should be enumerated",
        m_enumerate);
    args.add("schema", "Dump the schema", m_showSchema);
    args.add("pipeline-serialization", "Output filename for pipeline "
        "serialization", m_pipelineFile);
    args.add("summary", "Dump summary of the info", m_showSummary);
    args.add("metadata", "Dump file metadata info", m_showMetadata);
    args.add("stdin,s", "Read a pipeline file from standard input", m_usestdin);
}


// Note that the same information can come from the info filter, but
// this avoids point reads.
MetadataNode InfoKernel::dumpSummary(const QuickInfo& qi)
{
    MetadataNode summary;
    summary.add("num_points", qi.m_pointCount);
    if (qi.m_srs.valid())
    {
        MetadataNode srs = qi.m_srs.toMetadata();
        summary.add(srs);
    }
    if (qi.m_bounds.valid())
    {
        MetadataNode bounds = Utils::toMetadata(qi.m_bounds);
        summary.add(bounds.clone("bounds"));
    }

    std::string dims;
    auto di = qi.m_dimNames.begin();
    while (di != qi.m_dimNames.end())
    {
        dims += *di;
        ++di;
        if (di != qi.m_dimNames.end())
           dims += ", ";
    }
    if (dims.size())
        summary.add("dimensions", dims);
    return summary;
}

void InfoKernel::makeReader(const std::string& filename)
{
    Options rOps;
    if (!m_needPoints)
        rOps.add("count", 0);
    m_reader = &(m_manager.makeReader(filename, m_driverOverride, rOps));
}


void InfoKernel::makePipeline()
{
    Stage *stage = m_reader;

    Options iOps;
    if (m_queryPoint.size())
        iOps.add("query", m_queryPoint);
    if (m_pointIndexes.size())
        iOps.add("point", m_pointIndexes);
    stage = m_infoStage =
        &(m_manager.makeFilter("filters.info", *stage, iOps));

    if (m_showStats)
    {
        Options filterOptions;
        if (m_dimensions.size())
            filterOptions.add({"dimensions", m_dimensions});
        if (m_enumerate.size())
            filterOptions.add({"enumerate", m_enumerate});
        stage = m_statsStage =
            &m_manager.makeFilter("filters.stats", *stage, filterOptions);
    }
    if (m_boundary)
        m_hexbinStage = &m_manager.makeFilter("filters.hexbin", *stage);
}


MetadataNode InfoKernel::run(const std::string& filename)
{
    MetadataNode root;

    makeReader(filename);
    if (m_showSummary)
    {
        QuickInfo qi = m_manager.getStage()->preview();
        if (!qi.valid())
            throw pdal_error("No summary data available for '" +
                filename + "'.");
        root.add(dumpSummary(qi).clone("summary"));
    }
    else
    {
        makePipeline();
        if (m_needPoints || m_showMetadata)
            m_manager.execute(ExecMode::PreferStream);
        else
            m_manager.prepare();
        dump(root);
    }
    root.add("filename", filename);
    root.add("pdal_version", Config::fullVersionString());

    std::time_t now
    = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream t;
    t << std::put_time( std::localtime( &now ), "%FT%T%z" );
    root.add("reader", m_reader->getName());
    root.add("now", t.str());
    
    uintmax_t size = Utils::fileSize(filename);
    if (size)
        root.add("file_size", size);
    
    return root;
}


void InfoKernel::dump(MetadataNode& root)
{
    if (m_pipelineFile.size() > 0)
        PipelineWriter::writePipeline(m_manager.getStage(), m_pipelineFile);

    // Reader stage.
    if (m_showMetadata)
        root.add(m_reader->getMetadata().clone("metadata"));

    // Info stage.
    auto info = dynamic_cast<InfoFilter *>(m_infoStage);
    MetadataNode node = info->getMetadata();
    MetadataNode points = node.findChild("points");
    if (points)
        root.add(points);

    if (m_showSchema)
        root.add(node.findChild("schema"));

    // Stats stage.
    if (m_showStats)
        root.add(m_statsStage->getMetadata().clone("stats"));

    // Hexbin stage.
    if (m_hexbinStage)
    {
        MetadataNode node = m_hexbinStage->getMetadata();
        if (node.findChild("error"))
        {
            std::string poly = info->bounds().to2d().toWKT();

            MetadataNode m("boundary");
            m.add("boundary", poly, "Simple boundary of polygon");
            root.add(m);
        }
        else
            root.add(m_hexbinStage->getMetadata().clone("boundary"));
    }
}


int InfoKernel::execute()
{
    std::string filename = (m_usestdin ? std::string("STDIN") : m_inputFile);
    MetadataNode root = run(filename);
    Utils::toJSON(root, std::cout);

    return 0;
}


} // namespace pdal
