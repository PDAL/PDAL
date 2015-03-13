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
    , m_useJSON(false)
    , m_showSummary(false)
    , m_statsStage(NULL)
{}


void InfoKernel::validateSwitches()
{
    int functions = 0;

    if (m_showStats)
        functions++;
    if (m_boundary)
        functions++;
    if (m_showSummary)
        functions++;
    if (m_QueryPoint.size())
        functions++;
    if (m_pointIndexes.size())
        functions++;
    if (m_showSchema)
        functions++;
    if (m_showMetadata)
        functions++;

    if (functions > 1)
        throw pdal_error("Incompatible options.");
    else if (functions == 0)
        m_showStats = true;
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
        ("query", po::value< std::string>(&m_QueryPoint),
         "A 2d or 3d point query point")
        ("stats",
         po::value<bool>(&m_showStats)->zero_tokens()->implicit_value(true),
         "dump stats on all points (reads entire dataset)")
        ("boundary",
         po::value<bool>(&m_boundary)->zero_tokens()->implicit_value(true),
         "compute a hexagonal hull/boundary of dataset")
        ("dimensions", po::value<std::string >(&m_Dimensions),
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

    addSwitchSet(processing_options);
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

    MetadataNode tree = utils::toMetadata(outView);
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


void InfoKernel::dump(std::ostream& o, const std::string& filename)
{
    MetadataNode root;
    root.add("filename", filename);

    bool bPrepared(false);
    if (m_showSummary || m_showAll)
    {
        QuickInfo qi = m_reader->preview();
        MetadataNode summary = dumpSummary(qi).clone("summary");
        root.add(summary);
    }
    if (m_showSchema || m_showAll)
    {
        m_manager->prepare();
        bPrepared = true;
        MetadataNode schema =
            utils::toMetadata(m_manager->pointTable()).clone("schema");
        root.add(schema);
    }
    if (!bPrepared)
        m_manager->prepare();

    m_manager->execute();
    if (m_showStats || m_showAll)
    {
        MetadataNode stats = m_statsStage->getMetadata().clone("stats");
        root.add(stats);
    }

    if (m_pipelineFile.size() > 0)
        PipelineWriter(*m_manager).writePipeline(m_pipelineFile);

    if (m_pointIndexes.size())
    {
        PointViewSet viewSet = m_manager->views();
        assert(viewSet.size() == 1);
        MetadataNode points = dumpPoints(*viewSet.begin()).clone("points");
        root.add(points);
    }
    if (m_QueryPoint.size())
    {
        PointViewSet viewSet = m_manager->views();
        assert(viewSet.size() == 1);
        root = dumpQuery(*viewSet.begin());
    }
    if (m_showMetadata || m_showAll)
    {
        MetadataNode metadata = m_reader->getMetadata().clone("metadata");
        root.add(metadata);
    }
    if (m_boundary || m_showAll)
    {
        PointViewSet viewSet = m_manager->views();
        assert(viewSet.size() == 1);
        MetadataNode boundary = m_hexbinStage->getMetadata().clone("boundary");
        root.add(boundary);
    }
    if (!root.valid())
        return;

    root.add("pdal_version", pdal::GetFullVersionString());
    utils::toJSON(root, o);
}


MetadataNode InfoKernel::dumpQuery(PointViewPtr inView) const
{
    auto seps = [](char c){ return (c == ',' || c == '|' || c == ' '); };

    std::vector<std::string> tokens = Utils::split2(m_QueryPoint, seps);
    std::vector<double> values;
    for (auto ti = tokens.begin(); ti != tokens.end(); ++ti)
        values.push_back(boost::lexical_cast<double>(*ti));

    if (values.size() != 2 && values.size() != 3)
        throw app_runtime_error("--points must be two or three values");

    bool is3d = (values.size() >= 3);

    double x = values[0];
    double y = values[1];
    double z = is3d ? values[2] : 0.0;

    PointViewPtr outView = inView->makeNew();

    KDIndex kdi(*inView);
    kdi.build(is3d);
    std::vector<PointId> ids = kdi.neighbors(x, y, z, inView->size());
    for (auto i = ids.begin(); i != ids.end(); ++i)
        outView->appendPoint(*inView.get(), *i);

    return utils::toMetadata(outView);
}


int InfoKernel::execute()
{
    Options readerOptions;

    std::string filename = m_usestdin ? std::string("STDIN") : m_inputFile;
    readerOptions.add("filename", filename);
    if (m_showMetadata)
        readerOptions.add("count", 0);

    m_manager = std::unique_ptr<PipelineManager>(
        KernelSupport::makePipeline(filename));
    m_reader = m_manager->getStage();
    Stage *stage = m_reader;

    if (m_Dimensions.size())
        m_options.add("dimensions", m_Dimensions, "List of dimensions");

    Options options = m_options + readerOptions;
    m_reader->setOptions(options);

    if (m_showStats || m_showAll)
    {
        m_statsStage = &(m_manager->addFilter("filters.stats"));
        m_statsStage->setOptions(options);
        m_statsStage->setInput(*stage);
        stage = m_statsStage;
    }
    if (m_boundary || m_showAll)
    {
        m_hexbinStage = &(m_manager->addFilter("filters.hexbin"));
        m_hexbinStage->setOptions(options);
        m_hexbinStage->setInput(*stage);
        stage = m_hexbinStage;
    }

    dump(std::cout, filename);

    return 0;
}


} // namespace pdal
