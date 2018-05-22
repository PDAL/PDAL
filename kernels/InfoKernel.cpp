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

#include <pdal/KDIndex.hpp>
#include <pdal/PipelineWriter.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/StageFactory.hpp>
#ifdef PDAL_HAVE_LIBXML2
#include <pdal/XMLSchema.hpp>
#endif

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

InfoKernel::InfoKernel()
    : m_showStats(false)
    , m_showSchema(false)
    , m_showAll(false)
    , m_showMetadata(false)
    , m_boundary(false)
    , m_showSummary(false)
    , m_needPoints(false)
    , m_statsStage(NULL)
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
    if (m_showStats || functions == 0 )
    {
        functions++;
        m_showStats = true;
        m_needPoints = true;
    }

    if (m_pointIndexes.size() && m_queryPoint.size())
        throw pdal_error("--point option incompatible with --query option.");

    if (m_showSummary && functions > 1)
        throw pdal_error("--summary option incompatible with other "
            "specified options.");
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
    args.add("pointcloudschema", "Dump PointCloudSchema XML output",
        m_PointCloudSchemaOutput).setHidden();
    args.add("stdin,s", "Read a pipeline file from standard input", m_usestdin);
}

// Support for parsing point numbers.  Points can be specified singly or as
// dash-separated ranges.  i.e. 6-7,8,19-20
namespace {

using namespace std;

uint32_t parseInt(const string& s)
{
    uint32_t i;

    if (!Utils::fromString(s, i))
        throw pdal_error(string("Invalid integer: ") + s);
    return i;
}


void addRange(const string& begin, const string& end, vector<PointId>& points)
{
    PointId low = parseInt(begin);
    PointId high = parseInt(end);
    if (low > high)
        throw pdal_error(string("Range invalid: ") + begin + "-" + end);
    while (low <= high)
        points.push_back(low++);
}


vector<PointId> getListOfPoints(std::string p)
{
    vector<PointId> output;

    //Remove whitespace from string with awful remove/erase idiom.
    p.erase(remove_if(p.begin(), p.end(), ::isspace), p.end());

    vector<string> ranges = Utils::split2(p, ',');
    for (string s : ranges)
    {
        vector<string> limits = Utils::split(s, '-');
        if (limits.size() == 1)
            output.push_back(parseInt(limits[0]));
        else if (limits.size() == 2)
            addRange(limits[0], limits[1], output);
        else
            throw pdal_error(string("Invalid point range: ") + s);
    }
    return output;
}

} //namespace

MetadataNode InfoKernel::dumpPoints(PointViewPtr inView) const
{
    MetadataNode root;
    PointViewPtr outView = inView->makeNew();

    // Stick points in a inViewfer.
    std::vector<PointId> points = getListOfPoints(m_pointIndexes);
    bool oorMsg = false;
    for (size_t i = 0; i < points.size(); ++i)
    {
        PointId id = (PointId)points[i];
        if (id < inView->size())
            outView->appendPoint(*inView.get(), id);
        else if (!oorMsg)
        {
            m_log->get(LogLevel::Warning) << "Attempt to display points with "
                "IDs not available in input dataset." << std::endl;
            oorMsg = true;
        }
    }

    MetadataNode tree = outView->toMetadata();
    for (size_t i = 0; i < outView->size(); ++i)
    {
        MetadataNode n = tree.findChild(std::to_string(i));
        n.add("PointId", points[i]);
        root.add(n.clone("point"));
    }
    return root;
}


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
        MetadataNode bounds = summary.add("bounds");
        MetadataNode x = bounds.add("X");
        x.add("min", qi.m_bounds.minx);
        x.add("max", qi.m_bounds.maxx);
        MetadataNode y = bounds.add("Y");
        y.add("min", qi.m_bounds.miny);
        y.add("max", qi.m_bounds.maxy);
        MetadataNode z = bounds.add("Z");
        z.add("min", qi.m_bounds.minz);
        z.add("max", qi.m_bounds.maxz);
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


void InfoKernel::makePipeline(const std::string& filename, bool noPoints)
{
    if (!pdal::Utils::fileExists(filename))
        throw pdal_error("File not found: " + filename);

    if (filename == "STDIN")
    {
        m_manager.readPipeline(std::cin);
        m_reader = m_manager.getStage();
    }
    else if (FileUtils::extension(filename) == ".xml" ||
        FileUtils::extension(filename) == ".json")
    {
        m_manager.readPipeline(filename);
        m_reader = m_manager.getStage();
    }
    else
    {
        Options ops;
        if (noPoints)
            ops.add("count", 0);
        Stage& reader = m_manager.makeReader(filename, m_driverOverride, ops);
        m_reader = &reader;
    }
    if (!m_reader)
        throw pdal_error("Pipeline contains no valid stages.");
}


void InfoKernel::setup(const std::string& filename)
{
    makePipeline(filename, !m_needPoints);

    Stage *stage = m_reader;
    if (m_showStats)
    {
        Options filterOptions;
        if (m_dimensions.size())
            filterOptions.add({"dimensions", m_dimensions});
        if (m_enumerate.size())
            filterOptions.add({"enumerate", m_enumerate});
        m_statsStage = &m_manager.makeFilter("filters.stats", *stage,
            filterOptions);
        stage = m_statsStage;
    }
    if (m_boundary)
    {
        try
        {
            m_hexbinStage = &m_manager.makeFilter("filters.hexbin", *stage);
        } catch (pdal::pdal_error&)
        {
            m_hexbinStage = nullptr;

        }
    }
}


MetadataNode InfoKernel::run(const std::string& filename)
{
    MetadataNode root;

    root.add("filename", filename);
    if (m_showSummary)
    {
        QuickInfo qi = m_reader->preview();
        if (!qi.valid())
            throw pdal_error("No summary data available for '" +
                filename + "'.");
        MetadataNode summary = dumpSummary(qi).clone("summary");
        root.add(summary);
    }
    else
    {
        if (m_needPoints || m_showMetadata)
            m_manager.execute();
        else
            m_manager.prepare();
        dump(root);
    }
    root.add("pdal_version", Config::fullVersionString());
    return root;
}


void InfoKernel::dump(MetadataNode& root)
{
    if (m_showSchema)
        root.add(m_manager.pointTable().layout()->toMetadata().clone("schema"));

    if (m_PointCloudSchemaOutput.size() > 0)
    {
#ifdef PDAL_HAVE_LIBXML2
        XMLSchema schema(m_manager.pointTable().layout());

        std::ostream *out = Utils::createFile(m_PointCloudSchemaOutput);
        std::string xml(schema.xml());
        out->write(xml.c_str(), xml.size());
        Utils::closeFile(out);
#else
        std::cerr << "libxml2 support not enabled, no schema is produced" <<
            std::endl;
#endif

    }
    if (m_showStats)
        root.add(m_statsStage->getMetadata().clone("stats"));

    if (m_pipelineFile.size() > 0)
        PipelineWriter::writePipeline(m_manager.getStage(), m_pipelineFile);

    if (m_pointIndexes.size())
    {
        PointViewSet viewSet = m_manager.views();
        assert(viewSet.size() == 1);
        MetadataNode points = dumpPoints(*viewSet.begin());
        if (points.valid())
            root.add(points.clone("points"));
    }

    if (m_queryPoint.size())
    {
        PointViewSet viewSet = m_manager.views();
        assert(viewSet.size() == 1);
        root.add(dumpQuery(*viewSet.begin()));
    }

    if (m_showMetadata)
    {
        // If we have a reader cached, this means we
        // weren't reading a pipeline file directly. In that
        // case, use the metadata from the reader (old behavior).
        // Otherwise, return the full metadata of the entire pipeline
        if (m_reader)
            root.add(m_reader->getMetadata().clone("metadata"));
        else
            root.add(m_manager.getMetadata().clone("metadata"));
    }

    if (m_boundary)
    {
        PointViewSet viewSet = m_manager.views();
        assert(viewSet.size() == 1);
        if (m_hexbinStage)
            root.add(m_hexbinStage->getMetadata().clone("boundary"));
        else
        {
            pdal::BOX2D bounds;
            for (auto const &v: viewSet)
            {
                pdal::BOX2D b;
                v->calculateBounds(b);
                bounds.grow(b);
            }
            std::stringstream polygon;
            polygon << "POLYGON ((";

            polygon <<         bounds.minx << " " << bounds.miny;
            polygon << ", " << bounds.maxx << " " << bounds.miny;
            polygon << ", " << bounds.maxx << " " << bounds.maxy;
            polygon << ", " << bounds.minx << " " << bounds.maxy;
            polygon << ", " << bounds.minx << " " << bounds.miny;
            polygon << "))";

            MetadataNode m("boundary");
            m.add("boundary",polygon.str(), "Simple boundary of polygon");
            root.add(m);

        }
    }
}


MetadataNode InfoKernel::dumpQuery(PointViewPtr inView) const
{
    int count;
    std::string location;

    // See if there's a provided point count.
    StringList parts = Utils::split2(m_queryPoint, '/');
    if (parts.size() == 2)
    {
        location = parts[0];
        count = atoi(parts[1].c_str());
    }
    else if (parts.size() == 1)
    {
        location = parts[0];
        count = inView->size();
    }
    else
        count = 0;
    if (count == 0)
        throw pdal_error("Invalid location specification. "
            "--query=\"X,Y[/count]\"");

    auto seps = [](char c){ return (c == ',' || c == '|' || c == ' '); };

    std::vector<std::string> tokens = Utils::split2(location, seps);
    std::vector<double> values;
    for (auto ti = tokens.begin(); ti != tokens.end(); ++ti)
    {
        double d;
        if (Utils::fromString(*ti, d))
            values.push_back(d);
    }

    if (values.size() != 2 && values.size() != 3)
        throw pdal_error("--points must be two or three values");

    PointViewPtr outView = inView->makeNew();

    std::vector<PointId> ids;
    if (values.size() >= 3)
    {
        KD3Index kdi(*inView);
        kdi.build();
        ids = kdi.neighbors(values[0], values[1], values[2], count);
    }
    else
    {
        KD2Index kdi(*inView);
        kdi.build();
        ids = kdi.neighbors(values[0], values[1], count);
    }

    for (auto i = ids.begin(); i != ids.end(); ++i)
        outView->appendPoint(*inView.get(), *i);

    return outView->toMetadata();
}


int InfoKernel::execute()
{
    std::string filename = m_usestdin ? std::string("STDIN") : m_inputFile;
    setup(filename);
    MetadataNode root = run(filename);
    Utils::toJSON(root, std::cout);

    return 0;
}


} // namespace pdal
