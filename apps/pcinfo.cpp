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
#include <pdal/SchemaLayout.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/filters/StatsFilter.hpp>
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
    void dumpStats(pdal::filters::StatsFilter& filter) const;
    void dumpSchema(const Stage&) const;
    void dumpStage(const Stage&) const;

    std::string m_inputFile;
    std::string m_outputFile;
    bool m_useLiblas;
    bool m_showStats;
    bool m_showSchema;
    bool m_showStage;
    boost::uint64_t m_pointNumber;
    std::ostream* m_outputStream;
};


PcInfo::PcInfo(int argc, char* argv[])
    : Application(argc, argv, "pcinfo")
    , m_inputFile("")
    , m_outputFile("")
    , m_useLiblas(false)
    , m_showStats(false)
    , m_showSchema(false)
    , m_showStage(false)
    , m_pointNumber((std::numeric_limits<boost::uint64_t>::max)())
    , m_outputStream(0)
{
    return;
}


void PcInfo::validateSwitches()
{
    if (m_inputFile == "")
    {
        throw app_usage_error("input file name required");
    }

    return;
}


void PcInfo::addSwitches()
{
    namespace po = boost::program_options;

    po::options_description* file_options = new po::options_description("file options");

    file_options->add_options()
        ("input,i", po::value<std::string>(&m_inputFile)->default_value(""), "input file name")
        ("output,o", po::value<std::string>(&m_outputFile)->default_value(""), "output file name")
        ("liblas", po::value<bool>(&m_useLiblas)->zero_tokens()->implicit_value(true), "use libLAS driver (not PDAL native driver)")
        ;

    addSwitchSet(file_options);

    po::options_description* processing_options = new po::options_description("processing options");

    processing_options->add_options()
        ("point,p", po::value<boost::uint64_t>(&m_pointNumber)->implicit_value(0), "point to dump")
        ("stats,a", po::value<bool>(&m_showStats)->zero_tokens()->implicit_value(true), "dump stats on all points (reads entire dataset)")
        ("schema,s", po::value<bool>(&m_showSchema)->zero_tokens()->implicit_value(true), "dump the schema")
        ("stage,r", po::value<bool>(&m_showStage)->zero_tokens()->implicit_value(true), "dump the stage info")
        ;

    addSwitchSet(processing_options);

    addPositionalSwitch("input", 1);

    return;
}


void PcInfo::dumpOnePoint(const Stage& stage) const
{
    const Schema& schema = stage.getSchema();
    SchemaLayout layout(schema);

    PointBuffer data(schema, 1);
    
    boost::scoped_ptr<StageSequentialIterator> iter(stage.createSequentialIterator());
    iter->skip(m_pointNumber);

    const boost::uint32_t numRead = iter->read(data);
    if (numRead != 1)
    {
        throw app_runtime_error("problem reading point number " + m_pointNumber);
    }

    boost::property_tree::ptree tree = data.toPTree();
   
    std::ostream& ostr = m_outputStream ? *m_outputStream : std::cout;

    ostr << "Point " << m_pointNumber << ":\n";
    write_json(ostr, tree.get_child("0"));
    ostr << "\n";

    return;
}


void PcInfo::dumpStats(pdal::filters::StatsFilter& filter) const
{

    const Schema& schema = filter.getSchema();
    SchemaLayout layout(schema);

    boost::scoped_ptr<StageSequentialIterator> iter(filter.createSequentialIterator());

    boost::uint64_t totRead = 0;
    while (!iter->atEnd())
    {
        PointBuffer data(schema, iter->getChunkSize());

        const boost::uint32_t numRead = iter->read(data);
        totRead += numRead;
    }

    boost::property_tree::ptree tree = filter.toStatsPTree();

    std::ostream& ostr = m_outputStream ? *m_outputStream : std::cout;

    write_json(ostr, tree);
    
    return;
}


void PcInfo::dumpSchema(const Stage& stage) const
{
    const Schema& schema = stage.getSchema();

    boost::property_tree::ptree tree = schema.toPTree();
    
    std::ostream& ostr = m_outputStream ? *m_outputStream : std::cout;

    boost::property_tree::write_json(ostr, tree);
    
    return;
}


void PcInfo::dumpStage(const Stage& stage) const
{
    boost::property_tree::ptree tree = stage.toPTree();

    std::ostream& ostr = m_outputStream ? *m_outputStream : std::cout;

    boost::property_tree::write_json(ostr, tree);

    return;
}


int PcInfo::execute()
{
    if (m_outputFile != "")
    {
        m_outputStream = FileUtils::createFile(m_outputFile);
        if (!m_outputStream)
        {
            throw app_runtime_error("cannot open output file: " + m_outputFile);
        }
    }

    Options readerOptions;
    {
        readerOptions.add<std::string>("filename", m_inputFile);
        readerOptions.add<bool>("debug", isDebug());
        readerOptions.add<boost::uint32_t>("verbose", getVerboseLevel());
        readerOptions.add<bool>("liblas", m_useLiblas);
    }

    Stage* reader = AppSupport::makeReader(readerOptions);
        
    pdal::filters::StatsFilter* filter = new pdal::filters::StatsFilter(*reader);

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


#if 0



liblas::Summary check_points(   liblas::Reader& reader,
                                std::vector<liblas::FilterPtr>& filters,
                                std::vector<liblas::TransformPtr>& transforms,
                                bool verbose)
{

    liblas::Summary summary;
    
    reader.SetFilters(filters);
    reader.SetTransforms(transforms);



    if (verbose)
    std::cout << "Scanning points:" 
        << "\n - : "
        << std::endl;

    //
    // Translation of points cloud to features set
    //
    boost::uint32_t i = 0;
    boost::uint32_t const size = reader.GetHeader().GetPointRecordsCount();
    

    while (reader.ReadNextPoint())
    {
        liblas::Point const& p = reader.GetPoint();
        summary.AddPoint(p);
        if (verbose)
            term_progress(std::cout, (i + 1) / static_cast<double>(size));
        i++;

    }
    if (verbose)
        std::cout << std::endl;
    
    return summary;
    
}



void PrintVLRs(std::ostream& os, liblas::Header const& header)
{
    if (!header.GetRecordsCount())
        return ;
        
    os << "---------------------------------------------------------" << std::endl;
    os << "  VLR Summary" << std::endl;
    os << "---------------------------------------------------------" << std::endl;
    
    typedef std::vector<VariableRecord>::size_type size_type;
    for(size_type i = 0; i < header.GetRecordsCount(); i++) {
        liblas::VariableRecord const& v = header.GetVLR(i);
        os << v;
    }
    
}


int main(int argc, char* argv[])
{

    std::string input;

    bool verbose = false;
    bool check = true;
    bool show_vlrs = true;
    bool show_schema = true;
    bool output_xml = false;
    bool output_json = false;
    bool show_point = false;
    bool use_locale = false;
    boost::uint32_t point = 0;
    


        file_options.add_options()
            ("no-vlrs", po::value<bool>(&show_vlrs)->zero_tokens()->implicit_value(false), "Don't show VLRs")
            ("no-schema", po::value<bool>(&show_schema)->zero_tokens()->implicit_value(false), "Don't show schema")
            ("no-check", po::value<bool>(&check)->zero_tokens()->implicit_value(false), "Don't scan points")
            ("xml", po::value<bool>(&output_xml)->zero_tokens()->implicit_value(true), "Output as XML")
            ("point,p", po::value<boost::uint32_t>(&point), "Display a point with a given id.  --point 44")

            ("locale", po::value<bool>(&use_locale)->zero_tokens()->implicit_value(true), "Use the environment's locale for output")

// --xml
// --json
// --restructured text output



        if (vm.count("point")) 
        {
            show_point = true;
        }




        if (show_point)
        {
            try 
            {
                reader.ReadPointAt(point);
                liblas::Point const& p = reader.GetPoint();
                if (output_xml) {
                    liblas::property_tree::ptree tree;
                    tree = p.GetPTree();
                    liblas::property_tree::write_xml(std::cout, tree);
                    exit(0);
                } 
                else 
                {
                    if (use_locale)
                    {
                        std::locale l("");
                        std::cout.imbue(l);
                    }
                    std::cout <<  p << std::endl;
                    exit(0);    
                }
                
            } catch (std::out_of_range const& e)
            {
                std::cerr << "Unable to read point at index " << point << ": " << e.what() << std::endl;
                exit(1);
                
            }

        }

        liblas::Summary summary;
        if (check)
            summary = check_points(  reader, 
                            filters,
                            transforms,
                            verbose
                            );

        liblas::Header const& header = reader.GetHeader();

        // Add the header to the summary so we can get more detailed 
        // info
        summary.SetHeader(header);
        
        if (output_xml && output_json) {
            std::cerr << "both JSON and XML output cannot be chosen";
            return 1;
        }
        if (output_xml) {
            liblas::property_tree::ptree tree;
            if (check)
                tree = summary.GetPTree();
            else 
            {
                tree.add_child("summary.header", header.GetPTree());
            }
            
            liblas::property_tree::write_xml(std::cout, tree);
            return 0;
        }

        if (use_locale)
        {
            std::locale l("");
            std::cout.imbue(l);
        }

        std::cout << header << std::endl;        
        if (show_vlrs)
            PrintVLRs(std::cout, header);

        if (show_schema)
            std::cout << header.GetSchema();
                    
        if (check) {
            std::cout << summary << std::endl;
            
        }
    }


#endif
