/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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


#include <iostream>

#include <boost/scoped_ptr.hpp>

#include <pdal/Stage.hpp>
#include <pdal/StageIterator.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/filters/Stats.hpp>

#ifdef PDAL_HAVE_LIBXML2
#include <pdal/XMLSchema.hpp>
#endif

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/tokenizer.hpp>

#include "AppSupport.hpp"
#include "Application.hpp"

#define SEPARATORS ",| "
typedef boost::tokenizer<boost::char_separator<char> > tokenizer;

using namespace pdal;


class PcInfo : public Application
{
public:
    PcInfo(int argc, char* argv[]);
    int execute(); // overrride

private:
    void addSwitches(); // overrride
    void validateSwitches(); // overrride

    void dumpOnePoint(const Stage&) const;
    void dumpStats(pdal::filters::Stats& filter) const;
    void dumpSchema(const Stage&) const;
    void dumpStage(const Stage&) const;
    void dumpQuery(Stage const&, IndexedPointBuffer&) const;
    void dumpMetadata(const Stage&) const;
    void dumpSDO_PCMetadata(Stage const&) const;

    std::string m_inputFile;
    bool m_showStats;
    bool m_showSchema;
    bool m_showStage;
    bool m_showMetadata;
    bool m_showSDOPCMetadata;
    pdal::Options m_options;
    boost::uint64_t m_pointNumber;
    std::ostream* m_outputStream;
    boost::uint32_t m_seed;
    boost::uint32_t m_sample_size;
    bool m_useXML;
    std::string m_Dimensions;
    std::string m_QueryPoint;
    double m_QueryDistance;
    boost::uint64_t m_numPointsToWrite;
};


PcInfo::PcInfo(int argc, char* argv[])
    : Application(argc, argv, "pcinfo")
    , m_inputFile("")
    , m_showStats(false)
    , m_showSchema(false)
    , m_showStage(false)
    , m_showMetadata(false)
    , m_showSDOPCMetadata(false)
    , m_pointNumber((std::numeric_limits<boost::uint64_t>::max)())
    , m_outputStream(0)
    , m_useXML(false)
    , m_QueryDistance(0.0)
    , m_numPointsToWrite(0)
{
    return;
}


void PcInfo::validateSwitches()
{

    const bool got_something =
        (m_pointNumber != (std::numeric_limits<boost::uint64_t>::max)()) ||
        m_showStats ||
        m_showSchema ||
        m_showMetadata ||
        m_showSDOPCMetadata ||
        m_showStage || 
        m_QueryPoint.size() > 0;
    if (!got_something)
    {
        throw app_usage_error("no action option specified");
    }

    return;
}


void PcInfo::addSwitches()
{
    namespace po = boost::program_options;

    po::options_description* file_options = new po::options_description("file options");

    file_options->add_options()
        ("input,i", po::value<std::string>(&m_inputFile)->default_value(""), "input file name")
        ;

    addSwitchSet(file_options);

    po::options_description* processing_options = new po::options_description("processing options");
    
    processing_options->add_options()
        ("point,p", po::value<boost::uint64_t>(&m_pointNumber)->implicit_value(0), "point to dump")
        ("query", po::value< std::string>(&m_QueryPoint), "A 2d or 3d point query point")
        ("distance", po::value< double>(&m_QueryDistance), "A query distance")
        ("stats,a", po::value<bool>(&m_showStats)->zero_tokens()->implicit_value(true), "dump stats on all points (reads entire dataset)")
        ("count", po::value<boost::uint64_t>(&m_numPointsToWrite)->default_value(0), "How many points should we write?")
        ("dimensions", po::value<std::string >(&m_Dimensions), "dump stats on all points (reads entire dataset)")
        ("schema,s", po::value<bool>(&m_showSchema)->zero_tokens()->implicit_value(true), "dump the schema")
        ("metadata,m", po::value<bool>(&m_showMetadata)->zero_tokens()->implicit_value(true), "dump the metadata")
        ("sdo_pc", po::value<bool>(&m_showSDOPCMetadata)->zero_tokens()->implicit_value(true), "dump the SDO_PC Oracle Metadata")
        ("stage,r", po::value<bool>(&m_showStage)->zero_tokens()->implicit_value(true), "dump the stage info")
        ("xml", po::value<bool>(&m_useXML)->zero_tokens()->implicit_value(true), "dump XML instead of JSON")
        ("seed", po::value<boost::uint32_t>(&m_seed)->default_value(0), "Seed value for random sample")
        ("sample_size", po::value<boost::uint32_t>(&m_sample_size)->default_value(1000), "Sample size for random sample")
        ;
    
    addSwitchSet(processing_options);

    addPositionalSwitch("input", 1);

    return;
}


void PcInfo::dumpOnePoint(const Stage& stage) const
{
    const Schema& schema = stage.getSchema();

    PointBuffer data(schema, 1);
    
    boost::scoped_ptr<StageSequentialIterator> iter(stage.createSequentialIterator(data));
    iter->skip(m_pointNumber);

    const boost::uint32_t numRead = iter->read(data);
    if (numRead != 1)
    {
        std::ostringstream oss;
        oss << "problem reading point number " << m_pointNumber;
        throw app_runtime_error(oss.str());
    }

    boost::property_tree::ptree tree = data.toPTree();
   
    std::ostream& ostr = m_outputStream ? *m_outputStream : std::cout;

    boost::property_tree::ptree output;
    output.add_child("point", tree.get_child("0"));
    if (m_useXML)
        write_xml(ostr, output);
    else
        write_json(ostr, tree.get_child("0"));
        
    return;
}


void PcInfo::dumpStats(pdal::filters::Stats& filter) const
{

    const Schema& schema = filter.getSchema();
    
    boost::uint32_t chunkSize(131072);
    if (filter.getNumPoints() > 0 )
    {
        chunkSize = filter.getNumPoints();
    } 

    PointBuffer data(schema, chunkSize);

    boost::scoped_ptr<StageSequentialIterator> iter(filter.createSequentialIterator(data));

    boost::uint64_t totRead = 0;
    while (!iter->atEnd())
    {

        const boost::uint32_t numRead = iter->read(data);
        totRead += numRead;
    }

    pdal::Metadata output = static_cast<pdal::filters::iterators::sequential::Stats*>(iter.get())->toMetadata();
    
    boost::property_tree::ptree tree;
    tree.add_child("stats", output.toPTree());
    std::ostream& ostr = m_outputStream ? *m_outputStream : std::cout;

    if (m_useXML)
        write_xml(ostr, tree);
    else
        write_json(ostr, tree);
    
    return;
}


void PcInfo::dumpSchema(const Stage& stage) const
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

void PcInfo::dumpQuery(Stage const& stage, IndexedPointBuffer& data) const
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

void PcInfo::dumpSDO_PCMetadata(const Stage& stage) const
{
    boost::property_tree::ptree metadata = stage.serializePipeline();

    const Schema& schema = stage.getSchema();
    
    std::string xml = pdal::Schema::to_xml(schema, &metadata);  
    
    std::ostream& ostr = m_outputStream ? *m_outputStream : std::cout;
    
    ostr << xml;
    
}

void PcInfo::dumpStage(const Stage& stage) const
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

void PcInfo::dumpMetadata(const Stage& stage) const
{
    boost::property_tree::ptree tree = stage.serializePipeline();
    std::ostream& ostr = m_outputStream ? *m_outputStream : std::cout;

    if (m_useXML)
        write_xml(ostr, tree);
    else
        write_json(ostr, tree);
         
    return;
}


int PcInfo::execute()
{


    Options readerOptions;
    {
        if (m_usestdin)
            m_inputFile = "STDIN";
        readerOptions.add<std::string>("filename", m_inputFile);
        readerOptions.add<bool>("debug", isDebug());
        readerOptions.add<boost::uint32_t>("verbose", getVerboseLevel());
    }

    Stage* reader = AppSupport::makeReader(readerOptions);

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
    if (m_Dimensions.size())
    {
        Option dimensions("dimensions", m_Dimensions, "Use explicit list of dimensions");
        m_options.add(dimensions);
        Option do_sample("do_sample", false, "Dont do sampling");
        m_options.add(do_sample);
    }
    
    pdal::Options options = m_options + readerOptions;
    
    pdal::filters::Stats* filter = new pdal::filters::Stats(*reader, options);
    

    filter->initialize();

    if (m_pointNumber != (std::numeric_limits<boost::uint64_t>::max)())
    {
        dumpOnePoint(*filter);
    }

    if (m_showStats)
    {
        dumpStats(*filter);
    }

    if (m_showSchema)
    {
        dumpSchema(*reader);
    }
    
    if (m_showMetadata)
    {
        dumpMetadata(*reader);
    }
    if (m_showStage)
    {
        dumpStage(*reader);
    }
    
    if (m_showSDOPCMetadata)
    {
        dumpSDO_PCMetadata(*reader);
    }
    
    if (m_QueryPoint.size())
    {
        IndexedPointBuffer buffer(reader->getSchema(), reader->getNumPoints());
        dumpQuery(*reader, buffer);
    }
    
    std::ostream& ostr = m_outputStream ? *m_outputStream : std::cout;
    ostr << std::endl;
    
    delete filter;
    delete reader;

    if (m_outputStream)
    {
        FileUtils::closeFile(m_outputStream);
    }
    
    return 0;
}


int main(int argc, char* argv[])
{
    PcInfo app(argc, argv);
    return app.run();
}

