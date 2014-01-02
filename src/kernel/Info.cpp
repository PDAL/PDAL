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

#include <pdal/kernel/Info.hpp>
#include <pdal/PipelineWriter.hpp>


#include <boost/algorithm/string/find.hpp>

namespace pdal { namespace kernel {

Info::Info(int argc, const char* argv[])
    : Application(argc, argv, "info")
    , m_inputFile("")
    , m_showStats(false)
    , m_showSchema(false)
    , m_showStage(false)
    , m_showMetadata(false)
    , m_showSDOPCMetadata(false)
    , m_outputStream(0)
    , m_useXML(false)
    , m_useJSON(false)
    , m_useREST(true)
    , m_QueryDistance(0.0)
    , m_numPointsToWrite(0)
    , m_showSample(false)
{
    return;
}


void Info::validateSwitches()
{

    const bool got_something =
        m_showStats ||
        m_showSchema ||
        m_showMetadata ||
        m_showSDOPCMetadata ||
        m_showStage || 
        m_QueryPoint.size() > 0 ||
        m_pointIndexes.size() > 0;
    if (!got_something)
    {
        throw app_usage_error("no action option specified");
    }

    return;
}


void Info::addSwitches()
{
    namespace po = boost::program_options;

    po::options_description* file_options = new po::options_description("file options");

    file_options->add_options()
        ("input,i", po::value<std::string>(&m_inputFile)->default_value(""), "input file name")
        ;

    addSwitchSet(file_options);

    po::options_description* processing_options = new po::options_description("processing options");
    
    processing_options->add_options()
        ("point,p", po::value<std::string >(&m_pointIndexes), "point to dump")
        ("query", po::value< std::string>(&m_QueryPoint), "A 2d or 3d point query point")
        ("distance", po::value< double>(&m_QueryDistance), "A query distance")
        ("stats,a", po::value<bool>(&m_showStats)->zero_tokens()->implicit_value(true), "dump stats on all points (reads entire dataset)")
        ("count", po::value<boost::uint64_t>(&m_numPointsToWrite)->default_value(0), "How many points should we write?")
        ("dimensions", po::value<std::string >(&m_Dimensions), "dump stats on all points (reads entire dataset)")
        ("schema,s", po::value<bool>(&m_showSchema)->zero_tokens()->implicit_value(true), "dump the schema")
        ("metadata,m", po::value<bool>(&m_showMetadata)->zero_tokens()->implicit_value(true), "dump the metadata")
        ("sdo_pc", po::value<bool>(&m_showSDOPCMetadata)->zero_tokens()->implicit_value(true), "dump the SDO_PC Oracle Metadata")
        ("stage,r", po::value<bool>(&m_showStage)->zero_tokens()->implicit_value(true), "dump the stage info")
        ("pipeline-serialization", po::value<std::string>(&m_pipelineFile)->default_value(""), "")
        ("xml", po::value<bool>(&m_useXML)->zero_tokens()->implicit_value(true), "dump XML")
        ("json", po::value<bool>(&m_useJSON)->zero_tokens()->implicit_value(true), "dump JSON")
        ("sample", po::value<bool>(&m_showSample)->zero_tokens()->implicit_value(true), "randomly sample dimension for stats")
        ("seed", po::value<boost::uint32_t>(&m_seed)->default_value(0), "Seed value for random sample")
        ("sample_size", po::value<boost::uint32_t>(&m_sample_size)->default_value(1000), "Sample size for random sample")

        ;
    
    addSwitchSet(processing_options);

    addPositionalSwitch("input", 1);

    return;
}

std::vector<boost::uint32_t>  getListOfPoints(std::string const& p)
{
    
    typedef const boost::iterator_range<std::string::const_iterator> StringRange;
    
    std::vector<boost::uint32_t> output;
    
    std::string comma(",");
    std::string dash("-");

    boost::char_separator<char> sep_comma(",");

    StringRange iDash = boost::algorithm::ifind_first(  StringRange(p.begin(), p.end()),
                                                        StringRange(dash.begin(), dash.end()));
    bool bHaveDash = !iDash.empty();

    StringRange iComma = boost::algorithm::ifind_first(  StringRange(p.begin(), p.end()),
                                                        StringRange(comma.begin(), comma.end()));
    bool bHaveComma = !iComma.empty();
    
    if (bHaveComma && bHaveDash)
    {
        std::ostringstream oss;
        oss << "list of points cannot contain both a dash for ranges and comma for explicit points";
        throw app_runtime_error(oss.str());
    }

    if (bHaveDash)
    {
        typedef boost::tokenizer<boost::char_separator<char>  > tokenizer;
        boost::char_separator<char> sep_dash("-");        
        tokenizer beg_end(p, sep_dash);
        unsigned i(0);
        std::vector<boost::uint32_t> rng;
        for (tokenizer::iterator t = beg_end.begin(); t != beg_end.end(); ++t)
        {
            boost::uint32_t v = boost::lexical_cast<boost::uint32_t>(*t);
            rng.push_back(v);
            i++;
        }

        if (rng.size() != 2)
        {
            std::ostringstream oss;
            oss << "multiple ranges cannot be specified";
            throw app_runtime_error(oss.str());            
        }
        
        for (unsigned i = rng[0]; i != rng[1]; ++i)
        {
            output.push_back(i);
        }

    }

    if (bHaveComma)
    {
        typedef boost::tokenizer<boost::char_separator<char>  > tokenizer;
        boost::char_separator<char> sep_comma(",");        
        tokenizer indexes(p, sep_comma);
        for (tokenizer::iterator t = indexes.begin(); t != indexes.end(); ++t)
        {
            boost::uint32_t v = boost::lexical_cast<boost::uint32_t>(*t);
            output.push_back(v);
        }
    }
    
    // Put our string on as an integer and hope for the best
    output.push_back(boost::lexical_cast<boost::uint32_t>(p));

    return output;
}

void Info::dumpPoints(const Stage& stage, std::string const& points_string) const
{
    const Schema& schema = stage.getSchema();

    std::vector<boost::uint32_t> points = getListOfPoints(points_string);
    
    PointBuffer output_data(schema, points.size());
    PointBuffer read_data(schema, 1);
        
    StageRandomIterator* iter = stage.createRandomIterator(read_data);
    
    if (!iter)
        throw app_runtime_error("Unable to create random iterator for stage!");
    
    for (size_t i = 0; i < points.size(); ++i )
    {
        boost::uint64_t s = iter->seek(points[i]);
        if (s != points[i])
        {
            std::ostringstream oss;
            oss << "Unable to seek to " << points[i] << "due to size of file being " << stage.getNumPoints();
            throw app_runtime_error(oss.str());
        }
        const boost::uint32_t numRead = iter->read(read_data);
        if (numRead != 1)
        {
            std::ostringstream oss;
            oss << "problem reading point number " << points[i];
            throw app_runtime_error(oss.str());
        }
        
        output_data.copyPointFast(i, 0, read_data);
    }
    output_data.setNumPoints(points.size());



    boost::property_tree::ptree tree = output_data.toPTree();
   
    std::ostream& ostr = m_outputStream ? *m_outputStream : std::cout;

    boost::property_tree::ptree output;
    output.add_child("point", tree.get_child("0"));
    if (m_useXML)
        write_xml(ostr, tree);
    else if (m_useJSON)
        write_json(ostr, tree);
    else if (m_useREST)
        output_data.toRST(ostr) << std::endl;
    
    delete iter;
    return;
}


void Info::dumpStats(pdal::filters::Stats& filter, pdal::PipelineManager* manager) const
{

    const Schema& schema = filter.getSchema();
    
    boost::uint32_t chunkSize(131072);
    if (filter.getNumPoints() > 0 )
    {
        chunkSize = filter.getNumPoints();
    } 
    
    pdal::PipelineWriter* writer(0);
    
    PointBuffer data(schema, chunkSize);

    if (m_pipelineFile.size() > 0)
    {
         writer = new pdal::PipelineWriter(*manager);
         writer->setPointBuffer(&data);
    }

    StageSequentialIterator* iter = filter.createSequentialIterator(data);

    boost::uint64_t totRead = 0;
    while (!iter->atEnd())
    {

        const boost::uint32_t numRead = iter->read(data);
        totRead += numRead;
    }
    
    pdal::Metadata output = static_cast<pdal::filters::iterators::sequential::Stats*>(iter)->toMetadata();
    delete iter;
    boost::property_tree::ptree tree;
    tree.add_child("stats", output.toPTree());
    std::ostream& ostr = m_outputStream ? *m_outputStream : std::cout;

    if (m_useXML)
        write_xml(ostr, tree);
    else
        write_json(ostr, tree);

    if (m_pipelineFile.size() > 0)
    {
        writer->writePipeline(m_pipelineFile);
        delete writer;
    }

    
    return;
}


void Info::dumpSchema(const Stage& stage) const
{
    const Schema& schema = stage.getSchema();

    boost::property_tree::ptree schema_tree = schema.toPTree();
    
    std::ostream& ostr = m_outputStream ? *m_outputStream : std::cout;

    boost::property_tree::ptree tree;
    tree.add_child("schema", schema_tree);
    
    if (m_useXML)
        write_xml(ostr, tree);
    else
        write_json(ostr, tree);
    
    return;
}

void Info::dumpQuery(Stage const& stage, IndexedPointBuffer& data) const
{

    boost::char_separator<char> sep(SEPARATORS);
    tokenizer tokens(m_QueryPoint, sep);
    std::vector<double> values;
    for (tokenizer::iterator t = tokens.begin(); t != tokens.end(); ++t) {
        values.push_back(boost::lexical_cast<double>(*t));
    }
    
    if (values.size() < 2)
        throw app_runtime_error("--points must be two or three values");

    boost::scoped_ptr<StageSequentialIterator> iter(stage.createSequentialIterator(data));

    const boost::uint32_t numRead = iter->read(data);
    
    bool is3D(true);
    if (values.size() < 3) 
        is3D = false;

    data.build(is3D);

    Schema const& schema = data.getSchema();
    Dimension const& dimX = schema.getDimension("X");
    Dimension const& dimY = schema.getDimension("Y");
    Dimension const& dimZ = schema.getDimension("Z");

    double x = values[0];
    double y = values[1];
    
    double z(0.0);
    if (is3D)
        z = values[2];                
    
    boost::uint32_t count(m_numPointsToWrite);
    if (!m_numPointsToWrite)
        count = 1;
    
    double d(0.0);
    std::vector<std::size_t> ids = data.neighbors(x, y, z, d, count);
    
    PointBuffer response(data.getSchema(), count);
    typedef std::vector<std::size_t>::const_iterator Iterator;
    std::vector<std::size_t>::size_type pos(0);
    for (Iterator i = ids.begin(); i != ids.end(); ++i)
    {
        response.copyPointFast(pos, *i, data);
        response.setNumPoints(response.getNumPoints() + 1);
        pos++;
    }

    boost::property_tree::ptree tree = response.toPTree();
   
    std::ostream& ostr = m_outputStream ? *m_outputStream : std::cout;

    boost::property_tree::ptree output;
    output.add_child("point", tree);
    if (m_useXML)
        write_xml(ostr, output);
    else
        write_json(ostr, tree);
    
    return;
}

void Info::dumpSDO_PCMetadata(const Stage& stage) const
{
    boost::property_tree::ptree metadata = stage.serializePipeline();

    const Schema& schema = stage.getSchema();
    
    std::string xml = pdal::Schema::to_xml(schema, &metadata);  
    
    std::ostream& ostr = m_outputStream ? *m_outputStream : std::cout;
    
    ostr << xml;
    
}

void Info::dumpStage(const Stage& stage) const
{
    // FIXME: change this to dumpPipeline now that we've removed toPTree stuff for Stage::
    // boost::property_tree::ptree tree = stage.toPTree();
    // 
    // std::ostream& ostr = m_outputStream ? *m_outputStream : std::cout;
    // 
    // boost::property_tree::ptree output;
    // output.add_child("stage", tree);
    // if (m_useXML)
    //     write_xml(ostr, output);
    // else
    //     write_json(ostr, tree);

    return;
}

void Info::dumpMetadata(const Stage& stage) const
{
    boost::property_tree::ptree tree = stage.serializePipeline();
    std::ostream& ostr = m_outputStream ? *m_outputStream : std::cout;

    if (m_useXML)
        write_xml(ostr, tree);
    else
        write_json(ostr, tree);
         
    return;
}


int Info::execute()
{


    Options readerOptions;
    {
        if (m_usestdin)
            m_inputFile = "STDIN";
        readerOptions.add<std::string>("filename", m_inputFile);
        readerOptions.add<bool>("debug", isDebug());
        readerOptions.add<boost::uint32_t>("verbose", getVerboseLevel());
    }

    PipelineManager* manager = AppSupport::makePipeline(readerOptions);
    
    // Stage* reader = AppSupport::makeReader(readerOptions);

    if (m_seed != 0)
    {
        Option seed_option("seed", m_seed, "seed value");
        m_options.add(seed_option);
    }
    
    Option sample_size("sample_size", m_sample_size, "sample size for random sample");
    m_options.add(sample_size);
    
    Option cls("exact_count", "Classification", "use exact counts for classification stats");
    Option rn("exact_count", "ReturnNumber", "use exact counts for ReturnNumber stats");
    Option nr("exact_count", "NumberOfReturns", "use exact counts for ReturnNumber stats");
    m_options.add(cls);
    m_options.add(rn);
    m_options.add(nr);
    Option do_sample("do_sample", m_showSample, "Dont do sampling");
    m_options.add(do_sample);
    if (m_Dimensions.size())
    {
        Option dimensions("dimensions", m_Dimensions, "Use explicit list of dimensions");
        m_options.add(dimensions);
    }
    
    pdal::Options options = m_options + readerOptions;
    
    Stage* reader = manager->getStage();
    manager->addFilter("filters.stats", *reader, options);
    // pdal::filters::Stats* filter = new pdal::filters::Stats(*reader, options);
    
    Stage* filter = manager->getStage();

    filter->initialize();

    if (m_pointIndexes.size())
    {
        dumpPoints(*filter, m_pointIndexes);
    }

    if (m_showStats)
    {
        dumpStats(*dynamic_cast<pdal::filters::Stats*>(filter), manager);
    }
    
    if (m_showSchema)
    {
        dumpSchema(*filter);
    }
    
    if (m_showMetadata)
    {
        dumpMetadata(*filter);
    }
    if (m_showStage)
    {
        dumpStage(*filter);
    }
    
    if (m_showSDOPCMetadata)
    {
        dumpSDO_PCMetadata(*filter);
    }
    
    if (m_QueryPoint.size())
    {
        IndexedPointBuffer buffer(filter->getSchema(), filter->getNumPoints());
        dumpQuery(*filter, buffer);
    }
    
    std::ostream& ostr = m_outputStream ? *m_outputStream : std::cout;
    ostr << std::endl;
    
    delete manager;

    if (m_outputStream)
    {
        FileUtils::closeFile(m_outputStream);
    }
    
    return 0;
}

}} // pdal::kernel
