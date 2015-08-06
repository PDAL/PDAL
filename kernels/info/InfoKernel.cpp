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

#include <pdal/KDIndex.hpp>
#include <pdal/PipelineWriter.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/pdal_config.hpp>
#ifdef PDAL_HAVE_LIBXML2
#include <pdal/XMLSchema.hpp>
#endif

#include <boost/program_options.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "kernels.info",
    "Info Kernel",
    "http://pdal.io/kernels/kernels.info.html" );

CREATE_STATIC_PLUGIN(1, 0, InfoKernel, Kernel, s_info)

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


void InfoKernel::validateSwitches()
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


void InfoKernel::addSwitches()
{
    namespace po = boost::program_options;

    po::options_description* file_options =
        new po::options_description("file options");

    file_options->add_options()
        ("input,i", po::value<std::string>(&m_inputFile)->default_value(""),
         "input file name")
        ;

    addSwitchSet(file_options);

    po::options_description* processing_options =
        new po::options_description("processing options");

    processing_options->add_options()
        ("all",
         po::value<bool>(&m_showAll)->zero_tokens()->implicit_value(true),
         "dump the schema")
        ("point,p", po::value<std::string >(&m_pointIndexes), "point to dump")
        ("query", po::value< std::string>(&m_queryPoint),
         "Return points in order of distance from the specified "
         "location (2D or 3D)\n"
         "--query Xcoord,Ycoord[,Zcoord][/count]")
        ("stats",
         po::value<bool>(&m_showStats)->zero_tokens()->implicit_value(true),
         "dump stats on all points (reads entire dataset)")
        ("boundary",
         po::value<bool>(&m_boundary)->zero_tokens()->implicit_value(true),
         "compute a hexagonal hull/boundary of dataset")
        ("dimensions", po::value<std::string >(&m_dimensions),
         "dimensions on which to compute statistics")
        ("schema",
         po::value<bool>(&m_showSchema)->zero_tokens()->implicit_value(true),
         "dump the schema")
        ("pipeline-serialization",
         po::value<std::string>(&m_pipelineFile)->default_value(""), "")
        ("summary",
         po::value<bool>(&m_showSummary)->zero_tokens()->implicit_value(true),
        "dump summary of the info")
        ("metadata",
         po::value<bool>(&m_showMetadata)->zero_tokens()->implicit_value(true),
        "dump file metadata info")
        ;

    po::options_description* hidden =
        new po::options_description("Hidden options");
    hidden->add_options()
        ("pointcloudschema",
         po::value<std::string>(&m_PointCloudSchemaOutput),
        "dump PointCloudSchema XML output")
            ;

    addSwitchSet(processing_options);
    addHiddenSwitchSet(hidden);
    addPositionalSwitch("input", 1);
}

// Support for parsing point numbers.  Points can be specified singly or as
// dash-separated ranges.  i.e. 6-7,8,19-20
namespace {

using namespace std;

uint32_t parseInt(const string& s)
{
    try
    {
        return boost::lexical_cast<uint32_t>(s);
    }
    catch (boost::bad_lexical_cast)
    {
        throw app_runtime_error(string("Invalid integer: ") + s);
    }
}


void addRange(const string& begin, const string& end, vector<PointId>& points)
{
    PointId low = parseInt(begin);
    PointId high = parseInt(end);
    if (low > high)
        throw app_runtime_error(string("Range invalid: ") + begin + "-" + end);
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
            throw app_runtime_error(string("Invalid point range: ") + s);
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
    for (size_t i = 0; i < points.size(); ++i)
    {
        PointId id = (PointId)points[i];
        if (id < inView->size())
            outView->appendPoint(*inView.get(), id);
    }

    MetadataNode tree = Utils::toMetadata(outView);
    std::string prefix("point ");
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
    summary.add("spatial_reference", qi.m_srs.getWKT());
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

    std::string dims;
    auto di = qi.m_dimNames.begin();
    while (di != qi.m_dimNames.end())
    {
        dims += *di;
        ++di;
        if (di != qi.m_dimNames.end())
           dims += ", ";
    }
    summary.add("dimensions", dims);
    return summary;
}


void InfoKernel::setup(const std::string& filename)
{
    Options readerOptions;

    readerOptions.add("filename", filename);
    if (!m_needPoints)
        readerOptions.add("count", 0);

    m_manager = std::unique_ptr<PipelineManager>(
        KernelSupport::makePipeline(filename));
    m_reader = m_manager->getStage();
    Stage *stage = m_reader;

    if (m_dimensions.size())
        m_options.add("dimensions", m_dimensions, "List of dimensions");

    Options options = m_options + readerOptions;
    m_reader->setOptions(options);

    if (m_showStats)
    {
        m_statsStage = &(m_manager->addFilter("filters.stats"));
        m_statsStage->setOptions(options);
        m_statsStage->setInput(*stage);
        stage = m_statsStage;
    }
    if (m_boundary)
    {
        m_hexbinStage = &(m_manager->addFilter("filters.hexbin"));
        m_hexbinStage->setOptions(options);
        m_hexbinStage->setInput(*stage);
        stage = m_hexbinStage;
        Options readerOptions;
    }
}


MetadataNode InfoKernel::run(const std::string& filename)
{
    MetadataNode root;

    root.add("filename", filename);
    if (m_showSummary)
    {
        QuickInfo qi = m_reader->preview();
        MetadataNode summary = dumpSummary(qi).clone("summary");
        root.add(summary);
    }
    else
    {
        applyExtraStageOptionsRecursive(m_manager->getStage());
        if (m_needPoints || m_showMetadata)
            m_manager->execute();
        else
            m_manager->prepare();
        dump(root);
    }
    root.add("pdal_version", pdal::GetFullVersionString());
    return root;
}


void InfoKernel::dump(MetadataNode& root)
{
    if (m_showSchema)
        root.add(Utils::toMetadata(m_manager->pointTable()).clone("schema"));

    if (m_PointCloudSchemaOutput.size() > 0)
    {
#ifdef PDAL_HAVE_LIBXML2
        XMLSchema schema(m_manager->pointTable().layout());

        std::ostream *out = FileUtils::createFile(m_PointCloudSchemaOutput);
        std::string xml(schema.xml());
        out->write(xml.c_str(), xml.size());
        FileUtils::closeFile(out);
#else
        std::cerr << "libxml2 support not enabled, no schema is produced" <<
            std::endl;
#endif

    }
    if (m_showStats)
        root.add(m_statsStage->getMetadata().clone("stats"));

    if (m_pipelineFile.size() > 0)
        PipelineWriter(*m_manager).writePipeline(m_pipelineFile);

    if (m_pointIndexes.size())
    {
        PointViewSet viewSet = m_manager->views();
        assert(viewSet.size() == 1);
        root.add(dumpPoints(*viewSet.begin()).clone("points"));
    }
    if (m_queryPoint.size())
    {
        PointViewSet viewSet = m_manager->views();
        assert(viewSet.size() == 1);
        root.add(dumpQuery(*viewSet.begin()));
    }
    if (m_showMetadata)
        root.add(m_reader->getMetadata().clone("metadata"));
    if (m_boundary)
    {
        PointViewSet viewSet = m_manager->views();
        assert(viewSet.size() == 1);
        root.add(m_hexbinStage->getMetadata().clone("boundary"));
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
        throw pdal_error("Invalid location specificiation. "
            "--query=\"X,Y[/count]\"");

    auto seps = [](char c){ return (c == ',' || c == '|' || c == ' '); };

    std::vector<std::string> tokens = Utils::split2(location, seps);
    std::vector<double> values;
    for (auto ti = tokens.begin(); ti != tokens.end(); ++ti)
        values.push_back(boost::lexical_cast<double>(*ti));

    if (values.size() != 2 && values.size() != 3)
        throw app_runtime_error("--points must be two or three values");

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

    return Utils::toMetadata(outView);
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
