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

#include <pdal/kernel/Info.hpp>
#include <pdal/KDIndex.hpp>
#include <pdal/PipelineWriter.hpp>

namespace pdal
{
namespace kernel
{

Info::Info(int argc, const char* argv[])
    : Application(argc, argv, "info")
    , m_inputFile("")
    , m_showStats(false)
    , m_showSchema(false)
    , m_showStage(false)
    , m_showMetadata(false)
    , m_showSDOPCMetadata(false)
    , m_computeBoundary(false)
    , m_useXML(false)
    , m_useJSON(false)
    , m_useREST(true)
    , m_QueryDistance(0.0)
    , m_numPointsToWrite(0)
    , m_showSample(false)
{}


void Info::validateSwitches()
{
    const bool got_something =
        m_showStats ||
        m_showSchema ||
        m_showMetadata ||
        m_showSDOPCMetadata ||
        m_computeBoundary ||
        m_showStage || 
        m_QueryPoint.size() > 0 ||
        m_pointIndexes.size() > 0;
    if (!got_something)
    {
        m_showStats = true;
        m_computeBoundary = true;
        m_showSchema = true;
        
    }
}


void Info::addSwitches()
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
        ("distance", po::value< double>(&m_QueryDistance), "A query distance")
        ("stats",
         po::value<bool>(&m_showStats)->zero_tokens()->implicit_value(true),
         "dump stats on all points (reads entire dataset)")
        ("boundary",
         po::value<bool>(&m_computeBoundary)->zero_tokens()->implicit_value(true),
         "compute a hexagonal hull/boundary of dataset")             
        ("count", po::value<uint64_t>(&m_numPointsToWrite)->default_value(0),
         "How many points should we write?")
        ("dimensions", po::value<std::string >(&m_Dimensions),
         "dump stats on all points (reads entire dataset)")
        ("schema",
         po::value<bool>(&m_showSchema)->zero_tokens()->implicit_value(true),
         "dump the schema")
        ("metadata,m",
         po::value<bool>(&m_showMetadata)->zero_tokens()->implicit_value(true),
         "dump the metadata")
        ("sdo_pc",
         po::value<bool>(&m_showSDOPCMetadata)->zero_tokens()->
             implicit_value(true),
         "dump the SDO_PC Oracle Metadata")
        ("stage,r",
         po::value<bool>(&m_showStage)->zero_tokens()->implicit_value(true),
         "dump the stage info")
        ("pipeline-serialization",
         po::value<std::string>(&m_pipelineFile)->default_value(""), "")
        ("xml", po::value<bool>(&m_useXML)->zero_tokens()->implicit_value(true),
         "dump XML")
        ("json",
         po::value<bool>(&m_useJSON)->zero_tokens()->implicit_value(true),
         "dump JSON")
        ("sample",
         po::value<bool>(&m_showSample)->zero_tokens()->implicit_value(true),
         "randomly sample dimension for stats")
        ("seed", po::value<uint32_t>(&m_seed)->default_value(0),
         "Seed value for random sample")
        ("sample_size",
         po::value<uint32_t>(&m_sample_size)->default_value(1000),
         "Sample size for random sample")
        ;
    
    addSwitchSet(processing_options);
    addPositionalSwitch("input", 1);
}

// Support for parsing point numbers.  Points can be specified singly or as
// dash-separated ranges.  i.e. 6-7,8,19-20
namespace {

using namespace std;

vector<string> tokenize(const string s, char c)
{
    string::const_iterator begin;
    string::const_iterator end;
    vector<string> strings;
    begin = s.begin();
    while (true)
    {
        end = find(begin, s.end(), c);
        strings.push_back(string(begin, end));
        if (end == s.end())
            break;
        begin = end + 1;
    }
    return strings;
}

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


void addSingle(const string& s, vector<uint32_t>& points)
{
    points.push_back(parseInt(s));
}


void addRange(const string& begin, const string& end,
    vector<uint32_t>& points)
{
    uint32_t low = parseInt(begin);
    uint32_t high = parseInt(end);
    if (low > high)
        throw app_runtime_error(string("Range invalid: ") + begin + "-" + end);
    while (low <= high)
        points.push_back(low++);
}


vector<boost::uint32_t> getListOfPoints(std::string p)
{
    vector<boost::uint32_t> output;

    //Remove whitespace from string with awful remove/erase idiom.
    p.erase(remove_if(p.begin(), p.end(), ::isspace), p.end());

    vector<string> ranges = tokenize(p, ',');
    for (string s : ranges)
    {
        vector<string> limits = tokenize(s, '-');
        if (limits.size() == 1)
            addSingle(limits[0], output);
        else if (limits.size() == 2)
            addRange(limits[0], limits[1], output);
        else
            throw app_runtime_error(string("Invalid point range: ") + s);
    }
    return output;
}

} //namespace

void Info::dumpPoints(PointBufferPtr buf) const
{
    PointBufferPtr outbuf = buf->makeNew();

    std::vector<uint32_t> points = getListOfPoints(m_pointIndexes);
    for (size_t i = 0; i < points.size(); ++i)
    {
        PointId id = (PointId)points[i];
        if (id < buf->size())
            outbuf->appendPoint(*buf, id);
    }

    boost::property_tree::ptree buffer_tree = outbuf->toPTree();
    m_tree->add_child("point", buffer_tree.get_child("0"));
}


void Info::dumpStats()
{
    PipelineWriter* writer = NULL;

    if (m_pipelineFile.size() > 0)
        writer = new pdal::PipelineWriter(*m_manager);

    MetadataNode statsNode("stats");

    statsNode.add(m_manager->getMetadata());

    boost::property_tree::ptree stats;
    std::stringstream strm;
    strm << statsNode.toJSON();
    boost::property_tree::read_json(strm, stats);
    m_tree->add_child("stats", stats);
    
    if (m_pipelineFile.size() > 0)
        writer->writePipeline(m_pipelineFile);
    delete writer;
}

void Info::dump(PointContext ctx, PointBufferPtr buf)
{
    if (m_showStats)
        dumpStats();

    if (m_pointIndexes.size())
        dumpPoints(buf);
    if (m_showSchema)
    {
        boost::property_tree::ptree schema;
        std::string json = ctx.dimsJson();
        std::stringstream strm;
        strm << json;        
        boost::property_tree::read_json(strm, schema);
        
        m_tree->add_child("schema", schema);
    }

    if (m_showSDOPCMetadata)
    {
        boost::property_tree::ptree metadata =
            m_manager->getStage()->serializePipeline();

        boost::property_tree::ptree output;
        m_tree->add_child("stage", output);
    }

    if (m_QueryPoint.size())
        dumpQuery(buf);
}


void Info::dumpQuery(PointBufferPtr buf) const
{
#define SEPARATORS ",| "
    boost::char_separator<char> sep(SEPARATORS);
    tokenizer tokens(m_QueryPoint, sep);
    std::vector<double> values;
    for (tokenizer::iterator t = tokens.begin(); t != tokens.end(); ++t)
        values.push_back(boost::lexical_cast<double>(*t));
    
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

    boost::property_tree::ptree tree = outbuf->toPTree();
    m_tree->add_child("point", tree);
}


void Info::dumpSDO_PCMetadata(PointContext ctx, const Stage& stage) const
{
    std::ostream& ostr = std::cout;
    // std::string xml = pdal::Schema::to_xml(*ctx.schema(), stage.getMetadata());
    // ostr << xml;
}



void Info::dumpMetadata(PointContext ctx, const Stage& stage) const
{
    boost::property_tree::ptree tree = stage.serializePipeline();
    std::ostream& ostr = std::cout;

    if (m_useXML)
        write_xml(ostr, tree);
    else
        write_json(ostr, tree);
}


int Info::execute()
{
    Options readerOptions;
    {
        if (m_usestdin)
            m_inputFile = "STDIN";
        readerOptions.add<std::string>("filename", m_inputFile);
        readerOptions.add<bool>("debug", isDebug());
        readerOptions.add<uint32_t>("verbose", getVerboseLevel());
    }

    m_manager = std::unique_ptr<PipelineManager>(
        AppSupport::makePipeline(readerOptions));
    
    if (m_seed != 0)
    {
        Option seed_option("seed", m_seed, "seed value");
        m_options.add(seed_option);
    }
    
    m_options.add("sample_size", m_sample_size,
        "sample size for random sample");
    m_options.add("exact_count", "Classification",
        "use exact counts for classification stats");
    m_options.add("exact_count", "ReturnNumber",
        "use exact counts for ReturnNumber stats");
    m_options.add("exact_count", "NumberOfReturns",
        "use exact counts for ReturnNumber stats");
    m_options.add("do_sample", m_showSample, "Don't do sampling");
    if (m_Dimensions.size())
        m_options.add("dimensions", m_Dimensions,
            "Use explicit list of dimensions");
    
    Options options = m_options + readerOptions;
    
    Stage* stage = m_manager->getStage();
    if (m_showStats)
        stage = m_manager->addFilter("filters.stats", stage, options);
    if (m_computeBoundary)
        stage = m_manager->addFilter("filters.hexbin", stage, options);
    
    m_tree = std::unique_ptr<boost::property_tree::ptree>(
        new boost::property_tree::ptree);

    std::ostream& ostr = std::cout;
    m_manager->execute();
    PointBufferSet pbSet = m_manager->buffers();
    assert(pbSet.size() == 1);
    dump(m_manager->context(), *pbSet.begin());

    //
    // if (m_showStats)
    //     dumpStats(ctx, *dynamic_cast<pdal::filters::Stats*>(filter), manager);
    // if (m_showSchema)
    //     dumpSchema(ctx);
    // if (m_showMetadata)
    //     dumpMetadata(ctx, *filter);
    // if (m_showStage)
    //     dumpStage(*filter);
    // if (m_showSDOPCMetadata)
    //     dumpSDO_PCMetadata(ctx, *filter);
    //
    // if (m_QueryPoint.size())
    //     dumpQuery(ctx, *filter);
    
    if (m_useXML)
        write_xml(ostr, *m_tree);
    else
        write_json(ostr, *m_tree);

    
    return 0;
}

}} // pdal::kernel
