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

#include <pdal/exceptions.hpp>
#include <pdal/drivers/las/Reader.hpp>

#include <pdal/drivers/las/Writer.hpp>
#include <pdal/drivers/qfit/Reader.hpp>

#include "Application.hpp"

using namespace pdal;
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

    pdal::Options options;

    pdal::Option<std::string> filename("input", m_inputFile, "Input filename for reader to use" );
    options.add(filename);
    DataStagePtr reader(new pdal::drivers::qfit::Reader(options));
    
    const boost::uint64_t numPoints = reader->getNumPoints();

    Options optsW("filename", m_outputFile, "file to write to");
    WriterPtr writer(new pdal::drivers::las::LasWriter(reader, optsW));

    // writer.setPointFormat( reader.getPointFormat() );

    writer->write(numPoints);

    return 0;
}

int main(int argc, char* argv[])
{
    Application_pc2pc app(argc, argv);
    return app.run();
}



