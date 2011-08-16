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

#include "AppSupport.hpp"
#include "Application.hpp"


class PcInfo : public Application
{
public:
    PcInfo(int argc, char* argv[]);
    int execute();

private:
    void addOptions();
    bool validateOptions();
    void readOnePoint();
    void readAllPoints();

    pdal::Stage* m_reader;
    std::string m_inputFile;
    boost::uint64_t m_pointNumber;
    boost::scoped_ptr<pdal::filters::StatsFilter> m_filter;
};


PcInfo::PcInfo(int argc, char* argv[])
    : Application(argc, argv, "pcinfo")
    , m_reader(NULL)
    , m_inputFile("")
    , m_pointNumber(0)
{
}


bool PcInfo::validateOptions()
{
    if (!hasOption("input"))
    {
        usageError("input file name required");
        return false;
    }

    return true;
}


void PcInfo::addOptions()
{
    namespace po = boost::program_options;

    po::options_description* file_options = new po::options_description("file options");

    file_options->add_options()
        ("input,i", po::value<std::string>(&m_inputFile), "input file name")
        ("liblas", "use libLAS driver (not PDAL native driver)")
        ;

    addOptionSet(file_options);

    po::options_description* processing_options = new po::options_description("processing options");

    processing_options->add_options()
        ("point,p", po::value<boost::uint64_t>(&m_pointNumber), "point to dump")
        ("points,a", "dump stats on all points (read entire dataset)")
        ;

    addOptionSet(processing_options);

    addPositionalOption("input", 1);

    return;
}


void PcInfo::readOnePoint()
{
    const pdal::Schema& schema = m_reader->getSchema();
    pdal::SchemaLayout layout(schema);

    pdal::PointBuffer data(layout, 1);
    
    boost::scoped_ptr<pdal::StageSequentialIterator> iter(m_reader->createSequentialIterator());
    iter->skip(m_pointNumber);

    const boost::uint32_t numRead = iter->read(data);
    if (numRead != 1)
    {
        runtimeError("problem reading point number " + m_pointNumber);
        return;
    }

    std::cout << "Read point " << m_pointNumber << "\n";

    return;
}


void PcInfo::readAllPoints()
{
    const pdal::Schema& schema = m_reader->getSchema();
    pdal::SchemaLayout layout(schema);

    boost::scoped_ptr<pdal::StageSequentialIterator> iter(m_reader->createSequentialIterator());

    boost::uint64_t totRead = 0;
    while (!iter->atEnd())
    {
        pdal::PointBuffer data(layout, 1024);

        const boost::uint32_t numRead = iter->read(data);
        totRead += numRead;
    }

    std::cout << "Read " << totRead << " points\n";
    
    return;
}


int PcInfo::execute()
{
    if (!pdal::FileUtils::fileExists(m_inputFile))
    {
        runtimeError("file not found: " + m_inputFile);
        return 1;
    }

    std::string driver = AppSupport::inferReaderDriver(m_inputFile);

    if (driver == "")
    {
        runtimeError("Cannot determine file type of " + m_inputFile);
        return 1;
    }

    if (hasOption("liblas") && driver == "drivers.las.reader")
    {
        driver = "drivers.liblas.reader";
    }

    pdal::Options options;
    options.add<bool>("debug", isDebug());
    options.add<boost::uint8_t>("verbose", getVerboseLevel());

    m_reader = AppSupport::createReader(driver, m_inputFile, options);

    
    m_reader->initialize();
    
    if (hasOption("point"))
    {
        readOnePoint();
    }

    if (hasOption("points"))
    {
        readAllPoints();
    }

    {
        const boost::uint64_t numPoints = m_reader->getNumPoints();
        const pdal::SpatialReference& srs = m_reader->getSpatialReference();

        std::cout << "driver type: " << m_reader->getName() << "\n";
        std::cout << numPoints << " points\n";
        std::cout << "WKT: " << srs.getWKT() << "\n";
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
