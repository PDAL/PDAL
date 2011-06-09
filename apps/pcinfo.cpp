/***************************************************************************
 *
 * Project: libLAS -- C/C++ read/write library for LAS LIDAR data
 * Purpose: LAS information with optional configuration
 * Author:  Howard Butler, hobu.inc at gmail.com
 ***************************************************************************
 * Copyright (c) 2010, Howard Butler, hobu.inc at gmail.com 
 *
 * See LICENSE.txt in this source distribution for more information.
 **************************************************************************/


#include <pdal/drivers/las/Reader.hpp>
#include <pdal/drivers/liblas/Reader.hpp>
#include <pdal/Utils.hpp>
#ifdef PDAL_HAVE_MRSID
#include <pdal/drivers/mrsid/Reader.hpp>
#endif

#include <iostream>

#include "Application.hpp"

using namespace pdal;
namespace po = boost::program_options;

class Application_pcinfo : public Application
{
public:
    Application_pcinfo(int argc, char* argv[]);
    int execute();
private:
    void addOptions();
    bool validateOptions();

    std::string m_inputFile;
};


Application_pcinfo::Application_pcinfo(int argc, char* argv[])
    : Application(argc, argv, "pcinfo")
{
}


bool Application_pcinfo::validateOptions()
{
    if (!hasOption("input"))
    {
        usageError("input file name required");
        return false;
    }

    return true;
}


void Application_pcinfo::addOptions()
{
    po::options_description* file_options = new po::options_description("file options");

    file_options->add_options()
        ("input,i", po::value<std::string>(&m_inputFile), "input file name")
        ("native", "use native LAS classes (not liblas)")
        ;

    addOptionSet(file_options);

    addPositionalOption("input", 1);

    return;
}


int Application_pcinfo::execute()
{
    if (!Utils::fileExists(m_inputFile))
    {
        runtimeError("file not found: " + m_inputFile);
        return 1;
    }

    pdal::Stage* reader = NULL;
    size_t ext = m_inputFile.find_last_of('.');
    if (ext != std::string::npos)
    {
        ext++;
        if (!m_inputFile.substr(ext).compare("las") ||
            !m_inputFile.substr(ext).compare("laz"))
        {
            if (hasOption("native"))
            {
                reader = new pdal::drivers::las::LasReader(m_inputFile);
            }
            else
            {
                reader = new pdal::drivers::liblas::LiblasReader(m_inputFile);
            }
        }
#ifdef PDAL_HAVE_MRSID
        else if (!m_inputFile.substr(ext).compare("sid"))
        {
            reader = new pdal::drivers::mrsid::Reader(m_inputFile.c_str());
        }
#endif
    }
    else
    {
        std::cerr << "Cannot determine file type of " << m_inputFile
                  << "." << std::endl;
        return 1;
    }

    const boost::uint64_t numPoints = reader->getNumPoints();
    const SpatialReference& srs = reader->getSpatialReference();

    std::cout << numPoints << " points\n";
    std::cout << "WKT: " << srs.getWKT() << "\n";

    delete reader;

    return 0;
}

int main(int argc, char* argv[])
{
    Application_pcinfo app(argc, argv);
    return app.run();
}


#if 0

#include <liblas/liblas.hpp>
#include "laskernel.hpp"

#include <boost/cstdint.hpp>
#include <boost/foreach.hpp>

#include <locale>


using namespace liblas;
using namespace std;


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

void OutputHelp( std::ostream & oss, po::options_description const& options)
{
    oss << "--------------------------------------------------------------------\n";
    oss << "    lasinfo (" << GetFullVersion() << ")\n";
    oss << "--------------------------------------------------------------------\n";

    oss << options;

    oss <<"\nFor more information, see the full documentation for lasinfo at:\n";
    
    oss << " http://liblas.org/utilities/lasinfo.html\n";
    oss << "----------------------------------------------------------\n";

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
    
    std::vector<liblas::FilterPtr> filters;
    std::vector<liblas::TransformPtr> transforms;
    
    liblas::Header header;

    try {

        po::options_description file_options("lasinfo options");
        po::options_description filtering_options = GetFilteringOptions();
        po::options_description header_options = GetHeaderOptions();

        po::positional_options_description p;
        p.add("input", 1);
        p.add("output", 1);

        file_options.add_options()
            ("help,h", "produce help message")
            ("input,i", po::value< string >(), "input LAS file")

            ("verbose,v", po::value<bool>(&verbose)->zero_tokens(), "Verbose message output")
            ("no-vlrs", po::value<bool>(&show_vlrs)->zero_tokens()->implicit_value(false), "Don't show VLRs")
            ("no-schema", po::value<bool>(&show_schema)->zero_tokens()->implicit_value(false), "Don't show schema")
            ("no-check", po::value<bool>(&check)->zero_tokens()->implicit_value(false), "Don't scan points")
            ("xml", po::value<bool>(&output_xml)->zero_tokens()->implicit_value(true), "Output as XML")
            ("point,p", po::value<boost::uint32_t>(&point), "Display a point with a given id.  --point 44")

            ("locale", po::value<bool>(&use_locale)->zero_tokens()->implicit_value(true), "Use the environment's locale for output")

// --xml
// --json
// --restructured text output
        ;

        po::variables_map vm;
        po::options_description options;
        options.add(file_options).add(filtering_options);
        po::store(po::command_line_parser(argc, argv).
          options(options).positional(p).run(), vm);

        po::notify(vm);

        if (vm.count("help")) 
        {
            OutputHelp(std::cout, options);
            return 1;
        }

        if (vm.count("point")) 
        {
            show_point = true;
        }

        if (vm.count("input")) 
        {
            input = vm["input"].as< string >();
            std::ifstream ifs;
            if (verbose)
                std::cout << "Opening " << input << " to fetch Header" << std::endl;
            if (!liblas::Open(ifs, input.c_str()))
            {
                std::cerr << "Cannot open " << input << " for read.  Exiting..." << std::endl;
                return 1;
            }
            liblas::ReaderFactory f;
            liblas::Reader reader = f.CreateWithStream(ifs);
            header = reader.GetHeader();
        } else {
            std::cerr << "Input LAS file not specified!\n";
            OutputHelp(std::cout, options);
            return 1;
        }


        filters = GetFilters(vm, verbose);

        std::ifstream ifs;
        if (!liblas::Open(ifs, input.c_str()))
        {
            std::cerr << "Cannot open " << input << " for read.  Exiting..." << std::endl;
            return false;
        }
    

        liblas::ReaderFactory f;
        liblas::Reader reader = f.CreateWithStream(ifs);
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
    catch(std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    catch(...) {
        std::cerr << "Exception of unknown type!\n";
    }
    
    return 0;


}

//las2las2 -i lt_srs_rt.las  -o foo.las -c 1,2 -b 2483590,366208,2484000,366612
#endif
