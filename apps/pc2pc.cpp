/***************************************************************************
 *
 * Project: libLAS -- C/C++ read/write library for LAS LIDAR data
 * Purpose: LAS translation with optional configuration
 * Author:  Howard Butler, hobu.inc at gmail.com
 ***************************************************************************
 * Copyright (c) 2010, Howard Butler, hobu.inc at gmail.com 
 *
 * See LICENSE.txt in this source distribution for more information.
 **************************************************************************/


#include <iostream>

#include <libpc/exceptions.hpp>
//#include <libpc/libpc_config.hpp>
//#include <libpc/Bounds.hpp>
//#include <libpc/Color.hpp>
//#include <libpc/Dimension.hpp>
//#include <libpc/Schema.hpp>
#include <libpc/filters/Chipper.hpp>
//#include <libpc/ColorFilter.hpp>
//#include <libpc/MosaicFilter.hpp>
//#include <libpc/FauxReader.hpp>
//#include <libpc/FauxWriter.hpp>
#include <libpc/drivers/las/Reader.hpp>
//#include <libpc/LasHeader.hpp>
#include <libpc/drivers/las/Writer.hpp>
#include <libpc/filters/CacheFilter.hpp>
#include <libpc/filters/ByteSwapFilter.hpp>

#include <libpc/drivers/liblas/Writer.hpp>
#include <libpc/drivers/liblas/Reader.hpp>

#ifdef LIBPC_HAVE_ORACLE
#include <libpc/drivers/oci/Writer.hpp>
#include <libpc/drivers/oci/Reader.hpp>
#endif

#include <boost/property_tree/xml_parser.hpp>

#include "Application.hpp"

using namespace libpc;
namespace po = boost::program_options;


class Application_pc2pc : public Application
{
public:
    Application_pc2pc(int argc, char* argv[]);
    int execute();

private:
    void addOptions();
    bool validateOptions();

    std::string m_inputFile;
    std::string m_outputFile;
    std::string m_xml;
    std::string m_srs;
    bool m_bCompress;
    
};


Application_pc2pc::Application_pc2pc(int argc, char* argv[])
    : Application(argc, argv, "pc2pc")
{
}


bool Application_pc2pc::validateOptions()
{
    if (!hasOption("input"))
    {
        usageError("--input/-i required");
        return false;
    }

    if (!hasOption("output"))
    {
        usageError("--output/-o required");
        return false;
    }

    return true;
}


void Application_pc2pc::addOptions()
{
    po::options_description* file_options = new po::options_description("file options");

    file_options->add_options()
        ("input,i", po::value<std::string>(&m_inputFile), "input file name")
        ("output,o", po::value<std::string>(&m_outputFile), "output file name")
        ("native", "use native LAS classes (not liblas)")
        ("oracle-writer", "Read data from LAS file and write to Oracle")
        ("oracle-reader", "Read data from Oracle and write to LAS")
        ("a_srs", po::value<std::string>(&m_srs)->default_value(""), "Assign output coordinate system")
        ("compress", po::value<bool>(&m_bCompress)->zero_tokens()->implicit_value(true),"Compress output data if available")
        ("xml", po::value<std::string>(&m_xml)->default_value("log.xml"), "XML file to load process (OCI only right now)")
        ;

    addOptionSet(file_options);
}

int Application_pc2pc::execute()
{
    if (!Utils::fileExists(m_inputFile))
    {
        runtimeError("file not found: " + m_inputFile);
        return 1;
    }

    std::ostream* ofs = Utils::createFile(m_outputFile);

    if (hasOption("native"))
    {
        libpc::drivers::las::LasReader reader(m_inputFile);
    
        const boost::uint64_t numPoints = reader.getNumPoints();

        libpc::drivers::las::LasWriter writer(reader, *ofs);

        //BUG: handle laz writer.setCompressed(false);

        //writer.setPointFormat( reader.getPointFormatNumber() );

        writer.write(numPoints);
    }

    else if (hasOption("oracle-writer"))
    {
#ifdef LIBPC_HAVE_ORACLE
        libpc::drivers::liblas::LiblasReader reader(m_inputFile);
    
        const boost::uint64_t numPoints = reader.getNumPoints();

        boost::property_tree::ptree load_tree;
        
        boost::property_tree::read_xml(m_xml, load_tree);
        
        boost::property_tree::ptree oracle_options = load_tree.get_child("drivers.oci.writer");
    
        libpc::Options options(oracle_options);
    
        boost::property_tree::ptree& tree = options.GetPTree();
        
        boost::uint32_t capacity = tree.get<boost::uint32_t>("capacity");
        
        
        libpc::filters::CacheFilter cache(reader, 1, capacity);
        libpc::filters::Chipper chipper(cache, capacity);
        libpc::filters::ByteSwapFilter swapper(chipper);
        libpc::drivers::oci::Writer writer(swapper, options);

        // libpc::filters::CacheFilter cache(reader, 1, capacity);
        // libpc::filters::Chipper chipper(cache, capacity);
        // libpc::drivers::oci::Writer writer(chipper, options);

        writer.write(numPoints);
        boost::property_tree::ptree output_tree;
        // output_tree.put_child(writer.getName(), options.GetPTree());
        // boost::property_tree::write_xml(m_xml, output_tree);
                    
#else
        throw configuration_error("libPC not compiled with Oracle support");
#endif
    }
        else if (hasOption("oracle-reader"))
        {
    #ifdef LIBPC_HAVE_ORACLE

        boost::property_tree::ptree load_tree;
        
        boost::property_tree::read_xml(m_xml, load_tree);
        
        boost::property_tree::ptree oracle_options = load_tree.get_child("drivers.oci.reader");
    
        libpc::Options options(oracle_options);

        libpc::drivers::oci::Reader reader(options);
        libpc::filters::ByteSwapFilter swapper(reader);
        const boost::uint64_t numPoints = reader.getNumPoints();
        libpc::drivers::las::LasWriter writer(swapper, *ofs);
        
        writer.setChunkSize(options.GetPTree().get<boost::uint32_t>("capacity"));

        if (hasOption("a_srs"))
        {
            libpc::SpatialReference ref;
            if (m_srs.size() > 0)
            {
                ref.setFromUserInput(m_srs);
                writer.setSpatialReference(ref);
            }
        }
        if (hasOption("compress"))
        {
            if (m_bCompress)
                writer.setCompressed(true);            
        }
        writer.write(numPoints);

        boost::property_tree::ptree output_tree;
        
        output_tree.put_child(reader.getName(), options.GetPTree());
        // output_tree.put_child(writer.getName(), )
        // boost::property_tree::write_xml(m_xml, output_tree);
            
    #else
            throw configuration_error("libPC not compiled with Oracle support");
    #endif
        }



    else
    {
        libpc::drivers::liblas::LiblasReader reader(m_inputFile);
    
        const boost::uint64_t numPoints = reader.getNumPoints();

        libpc::drivers::liblas::LiblasWriter writer(reader, *ofs);

        //BUG: handle laz writer.setCompressed(false);

        writer.setPointFormat( reader.getPointFormat() );

        writer.write(numPoints);
    }

    Utils::closeFile(ofs);

    return 0;
}

int main(int argc, char* argv[])
{
    Application_pc2pc app(argc, argv);
    return app.run();
}



