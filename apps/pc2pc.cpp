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

#include <pdal/exceptions.hpp>
#include <pdal/FileUtils.hpp>

#include <pdal/drivers/las/Reader.hpp>
#include <pdal/drivers/las/Writer.hpp>
#ifdef PDAL_HAVE_LIBLAS
#include <pdal/drivers/liblas/Writer.hpp>
#include <pdal/drivers/liblas/Reader.hpp>
#endif
#ifdef PDAL_HAVE_ORACLE
#include <pdal/drivers/oci/Writer.hpp>
#include <pdal/drivers/oci/Reader.hpp>
#endif

#include <pdal/filters/ByteSwapFilter.hpp>
#include <pdal/filters/CacheFilter.hpp>
#include <pdal/filters/Chipper.hpp>
#include <pdal/filters/ReprojectionFilter.hpp>
#include <pdal/filters/ScalingFilter.hpp>

#include <boost/property_tree/xml_parser.hpp>

#include "AppSupport.hpp"

#include "Application.hpp"

using namespace pdal;


class Pc2Pc : public Application
{
public:
    Pc2Pc(int argc, char* argv[]);
    int execute();

private:
    void addSwitches();
    void validateSwitches();

    std::string m_inputFile;
    std::string m_outputFile;
    bool m_useLiblas;
    std::string m_srs;
    bool m_bCompress;
    
};


Pc2Pc::Pc2Pc(int argc, char* argv[])
    : Application(argc, argv, "pc2pc")
    , m_inputFile("")
    , m_outputFile("")
    , m_useLiblas(false)
    , m_srs("")
    , m_bCompress(false)
{
    return;
}


void Pc2Pc::validateSwitches()
{
    if (m_inputFile == "")
    {
        throw app_usage_error("--input/-i required");
    }

    if (m_outputFile == "")
    {
        throw app_usage_error("--output/-o required");
    }

    return;
}


void Pc2Pc::addSwitches()
{
    namespace po = boost::program_options;
    
    po::options_description* file_options = new po::options_description("file options");

    file_options->add_options()
        ("input,i", po::value<std::string>(&m_inputFile)->default_value(""), "input file name")
        ("output,o", po::value<std::string>(&m_outputFile)->default_value(""), "output file name")
        ("liblas", po::value<bool>(&m_useLiblas)->zero_tokens()->implicit_value(true), "use libLAS driver (not PDAL native driver)")
        ("a_srs", po::value<std::string>(&m_srs)->default_value(""), "Assign output coordinate system (if supported by output format)")
        ("compress,z", po::value<bool>(&m_bCompress)->zero_tokens()->implicit_value(true), "Compress output data (if supported by output format)")
        ;

    addSwitchSet(file_options);
}


int Pc2Pc::execute()
{
    Options readerOptions;
    {
        readerOptions.add<std::string>("filename", m_inputFile);
        readerOptions.add<bool>("debug", isDebug());
        readerOptions.add<boost::uint32_t>("verbose", getVerboseLevel());
        readerOptions.add<bool>("liblas", m_useLiblas);
    }

    Options writerOptions;
    {
        writerOptions.add<std::string>("filename", m_outputFile);
        writerOptions.add<bool>("debug", isDebug());
        writerOptions.add<boost::uint32_t>("verbose", getVerboseLevel());

        if (m_srs != "")
        {
            writerOptions.add<std::string>("a_srs", m_srs);
        }

        writerOptions.add<bool>("compression_hack1", m_bCompress);
        writerOptions.add<bool>("liblas", m_useLiblas);
    }

    Stage& stage = AppSupport::makeReader(readerOptions);

    // BUG: I don't know how we could do this...
    // writer.setPointFormat( reader.getPointFormat() );

    Writer& writer = AppSupport::makeWriter(writerOptions, stage);

    writer.initialize();

    const boost::uint64_t numPoints = writer.write(0);

    std::cout << "Wrote " << numPoints << " points\n";

    return 0;
}


int main(int argc, char* argv[])
{
    Pc2Pc app(argc, argv);
    return app.run();
}
