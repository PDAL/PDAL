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

#include <algorithm>
#include <boost/foreach.hpp>

#include <InfoKernel.hpp>
#include <pdal/KDIndex.hpp>
#include <pdal/PipelineWriter.hpp>
#include <pdal/PDALUtils.hpp>

namespace pdal
{

InfoKernel::InfoKernel()
    : m_showStats(false)
    , m_showSchema(false)
    , m_showMetadata(false)
    , m_computeBoundary(false)
    , m_useXML(false)
    , m_useJSON(false)
    , m_useRST(false)
    , m_showSummary(false)
    , m_statsStage(NULL)
{}


void InfoKernel::validateSwitches()
{
    int functions = 0;

    if (m_showStats)
        functions++;
    if (m_showMetadata)
        functions++;
    if (m_computeBoundary)
        functions++;
    if (m_showSummary)
        functions++;
    if (m_QueryPoint.size())
        functions++;
    if (m_pointIndexes.size())
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
        ("point,p", po::value<std::string >(&m_pointIndexes), "point to dump")
        ("query", po::value< std::string>(&m_QueryPoint),
         "A 2d or 3d point query point")
        ("stats",
         po::value<bool>(&m_showStats)->zero_tokens()->implicit_value(true),
         "dump stats on all points (reads entire dataset)")
        ("boundary",
         po::value<bool>(&m_computeBoundary)->zero_tokens()->implicit_value(true),
         "compute a hexagonal hull/boundary of dataset")
        ("dimensions", po::value<std::string >(&m_Dimensions),
         "dimensions on which to compute statistics")
        ("schema",
         po::value<bool>(&m_showSchema)->zero_tokens()->implicit_value(true),
         "dump the schema")
        ("metadata,m",
         po::value<bool>(&m_showMetadata)->zero_tokens()->implicit_value(true),
         "dump the metadata")
        ("pipeline-serialization",
         po::value<std::string>(&m_pipelineFile)->default_value(""), "")
        ("xml", po::value<bool>(&m_useXML)->zero_tokens()->implicit_value(true),
         "dump XML")
        ("json",
         po::value<bool>(&m_useJSON)->zero_tokens()->implicit_value(true),
         "dump JSON")
        ("rst", po::value<bool>(&m_useRST)->zero_tokens()->implicit_value(true),
         "dump RST")
        ("summary",
         po::value<bool>(&m_showSummary)->zero_tokens()->implicit_value(true),
        "dump summary of the info")
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

MetadataNode InfoKernel::dumpPoints(PointBufferPtr buf) const
{
    MetadataNode root;
    PointBufferPtr outbuf = buf->makeNew();

    // Stick points in a buffer.
    std::vector<PointId> points = getListOfPoints(m_pointIndexes);
    for (size_t i = 0; i < points.size(); ++i)
    {
        PointId id = (PointId)points[i];
        if (id < buf->size())
            outbuf->appendPoint(*buf, id);
    }

    MetadataNode tree = utils::toMetadata(*outbuf);
    std::string prefix("point ");
    for (size_t i = 0; i < outbuf->size(); ++i)
    {
        MetadataNode n = tree.findChild(std::to_string(i));
        root.add(n.clone(prefix + std::to_string(points[i])));
    }
    return root;
}


MetadataNode InfoKernel::dumpSummary()
{
    QuickInfo qi = m_reader->preview();

    MetadataNode summary("summary");
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


void InfoKernel::dump(PointContext ctx, PointBufferPtr buf)
{
    if (m_showStats)
        m_statsStage->getMetadata();

    if (m_pipelineFile.size() > 0)
        PipelineWriter(*m_manager).writePipeline(m_pipelineFile);

    if (m_pointIndexes.size())
        dumpPoints(buf);

    if (m_showSchema)
        MetadataNode root = utils::toMetadata(ctx);

    if (m_QueryPoint.size())
        dumpQuery(buf);

    if (m_showSummary)
        dumpSummary();

    if (m_computeBoundary)
        m_hexbinStage->getMetadata();
}


MetadataNode InfoKernel::dumpQuery(PointBufferPtr buf) const
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

    PointBufferPtr outbuf = buf->makeNew();

    KDIndex kdi(*buf);
    kdi.build(is3d);
    std::vector<size_t> ids = kdi.neighbors(x, y, z, 0.0, buf->size());
    for (auto i = ids.begin(); i != ids.end(); ++i)
        outbuf->appendPoint(*buf, *i);

    return utils::toMetadata(*outbuf);
}


void InfoKernel::dumpMetadata(PointContext ctx, const Stage& stage) const
{
    boost::property_tree::ptree tree = stage.serializePipeline();
    std::ostream& ostr = std::cout;

    if (m_useXML)
        write_xml(ostr, tree);
    else if (m_useRST)
        pdal::utils::reST::write_rst(ostr, tree);
    else
        write_json(ostr, tree);
}


int InfoKernel::execute()
{
    Options readerOptions;

    std::string filename = m_usestdin ? std::string("STDIN") : m_inputFile;
    readerOptions.add("filename", filename);

    m_manager = std::unique_ptr<PipelineManager>(
        KernelSupport::makePipeline(filename));
    m_reader = static_cast<Reader *>(m_manager->getStage());
    Stage *stage = m_reader;

    if (m_Dimensions.size())
        m_options.add("dimensions", m_Dimensions, "List of dimensions");

    Options options = m_options + readerOptions;
    m_reader->setOptions(options);
    if (m_showStats)
    {
        m_statsStage = m_manager->addFilter("filters.stats", stage);
        m_statsStage->setOptions(options);
        stage = m_statsStage;
    }
    if (m_computeBoundary)
    {
        m_hexbinStage = m_manager->addFilter("filters.hexbin", stage);
        stage->setOptions(options);
        stage = m_hexbinStage;
    }

    std::ostream& ostr = std::cout;
    m_manager->execute();
    PointBufferSet pbSet = m_manager->buffers();
    assert(pbSet.size() == 1);
    dump(m_manager->context(), *pbSet.begin());

//ABELL
/**
    if (m_useXML)
        write_xml(ostr, *m_tree);
    else if (m_useRST)
        pdal::utils::reST::write_rst(ostr, *m_tree);
    else
        write_json(ostr, *m_tree);
**/

    return 0;
}


} // namespace pdal
