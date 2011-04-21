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

#include <libpc/drivers/liblas/Writer.hpp>
#include <libpc/drivers/liblas/Reader.hpp>

#ifdef LIBPC_HAVE_ORACLE
#include <libpc/drivers/oci/Writer.hpp>
#include <libpc/drivers/oci/Reader.hpp>
#endif

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
    std::string m_OracleReadSQL;
    std::string m_OracleWriteSQL;
    
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
        ("oracle-writer", "write data into oracle (must edit source to make this work right now)")
        ("oracle-reader", po::value<std::string>(&m_OracleReadSQL), "SQL to select a block table")
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
        
        libpc::drivers::oci::Options options;
        boost::property_tree::ptree& tree = options.GetPTree();
        
        boost::uint32_t capacity = 10000;
        tree.put("capacity", capacity);
        tree.put("overwrite", false);
        tree.put("connection", "lidar/lidar@oracle.hobu.biz/orcl");
        // tree.put("connection", "lidar/lidar@oracle.hobu.biz/crrel");
        tree.put("debug", true);
        tree.put("verbose", true);
        tree.put("scale.x", 0.0000001);
        tree.put("scale.y", 0.0000001);
        tree.put("scale.z", 0.001);
        
        libpc::filters::CacheFilter cache(reader, 1, 1024);
        libpc::filters::Chipper chipper(cache, capacity);
        libpc::drivers::oci::Writer writer(chipper, options);

        writer.write(numPoints);
#else
        throw configuration_error("libPC not compiled with Oracle support");
#endif
    }
        else if (hasOption("oracle-reader"))
        {
    #ifdef LIBPC_HAVE_ORACLE
            libpc::drivers::oci::Options options;
            boost::property_tree::ptree& tree = options.GetPTree();
            tree.put("capacity", 12);
            tree.put("connection", "lidar/lidar@oracle.hobu.biz/orcl");
            // tree.put("connection", "lidar/lidar@oracle.hobu.biz/crrel");
            tree.put("debug", true);
            tree.put("verbose", true);
        tree.put("scale.x", 0.0000001);
        tree.put("scale.y", 0.0000001);
        tree.put("scale.z", 0.001);
            // tree.put("select_sql", "select * from output");
            // tree.put("select_sql", "select cloud from hobu where id = 5");
            // tree.put("select_sql", "select cloud from hobu where id = 1");
            
            if (m_OracleReadSQL.size() == 0) {
                throw libpc_error("Select statement to read OCI data is empty!");
            }
            tree.put("select_sql", m_OracleReadSQL);

            libpc::drivers::oci::Reader reader(options);

            const boost::uint64_t numPoints = reader.getNumPoints();



            libpc::drivers::las::LasWriter writer(reader, *ofs);
            // writer.setPointFormat( 3);
            writer.write(numPoints);
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



