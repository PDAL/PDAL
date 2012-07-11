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

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "AppSupport.hpp"
#include "Application.hpp"

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
    void dumpMetadata(const Stage&) const;

    std::string m_inputFile;
    bool m_showStats;
    bool m_showSchema;
    bool m_showStage;
    bool m_showMetadata;
    pdal::Options m_options;
    boost::uint64_t m_pointNumber;
    std::ostream* m_outputStream;
    boost::uint32_t m_seed;
    boost::uint32_t m_sample_size;
    bool m_useXML;
};


PcInfo::PcInfo(int argc, char* argv[])
    : Application(argc, argv, "pcinfo")
    , m_inputFile("")
    , m_showStats(false)
    , m_showSchema(false)
    , m_showStage(false)
    , m_showMetadata(false)
    , m_pointNumber((std::numeric_limits<boost::uint64_t>::max)())
    , m_outputStream(0)
    , m_useXML(false)
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
        m_showStage;
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
        ("stats,a", po::value<bool>(&m_showStats)->zero_tokens()->implicit_value(true), "dump stats on all points (reads entire dataset)")
        ("schema,s", po::value<bool>(&m_showSchema)->zero_tokens()->implicit_value(true), "dump the schema")
        ("metadata,m", po::value<bool>(&m_showMetadata)->zero_tokens()->implicit_value(true), "dump the metadata")
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

    PointBuffer data(schema);

    boost::scoped_ptr<StageSequentialIterator> iter(filter.createSequentialIterator(data));

    boost::uint64_t totRead = 0;
    while (!iter->atEnd())
    {

        const boost::uint32_t numRead = iter->read(data);
        totRead += numRead;
    }

    boost::property_tree::ptree stats_tree = static_cast<pdal::filters::iterators::sequential::Stats*>(iter.get())->toPTree();
    
    boost::property_tree::ptree tree;
    tree.add_child("stats", stats_tree);
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

    boost::property_tree::ptree tree = schema.toPTree();
    
    std::ostream& ostr = m_outputStream ? *m_outputStream : std::cout;

    if (m_useXML)
        write_xml(ostr, tree);
    else
        write_json(ostr, tree);
    
    return;
}


void PcInfo::dumpStage(const Stage& stage) const
{
    boost::property_tree::ptree tree = stage.toPTree();

    std::ostream& ostr = m_outputStream ? *m_outputStream : std::cout;

    boost::property_tree::ptree output;
    output.add_child("stage", tree);
    if (m_useXML)
        write_xml(ostr, output);
    else
        write_json(ostr, tree);

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
    
    Option exact_count("exact_count", "Classification", "use exact counts for classification stats");
    m_options.add(exact_count);
    
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

