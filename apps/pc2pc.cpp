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

#include <pdal/FileUtils.hpp>

#include <pdal/drivers/las/Reader.hpp>
#include <pdal/drivers/las/Writer.hpp>
#ifdef PDAL_HAVE_ORACLE
#include <pdal/drivers/oci/Writer.hpp>
#include <pdal/drivers/oci/Reader.hpp>
#endif

#include <pdal/filters/Cache.hpp>
#include <pdal/filters/Chipper.hpp>
#include <pdal/Filters/Crop.hpp>
#include <pdal/filters/InPlaceReprojection.hpp>
#include <pdal/filters/Scaling.hpp>
#include <pdal/SpatialReference.hpp>
#include <pdal/Bounds.hpp>

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
    bool m_bCompress;
    boost::uint64_t m_numPointsToWrite; 
    boost::uint64_t m_numSkipPoints;
    pdal::SpatialReference m_input_srs;
    pdal::SpatialReference m_output_srs;
    pdal::Bounds<double> m_bounds;
    std::string m_wkt;
};


Pc2Pc::Pc2Pc(int argc, char* argv[])
    : Application(argc, argv, "pc2pc")
    , m_inputFile("")
    , m_outputFile("")
    , m_bCompress(false)
    , m_numPointsToWrite(0)
    , m_numSkipPoints(0)
    , m_input_srs(pdal::SpatialReference())
    , m_output_srs(pdal::SpatialReference())
    , m_wkt("")
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
        ("a_srs", po::value<pdal::SpatialReference>(&m_input_srs), "Assign input coordinate system (if supported by output format)")
        ("t_srs", po::value<pdal::SpatialReference>(&m_output_srs), "Transform to output coordinate system (if supported by output format)")
        ("compress,z", po::value<bool>(&m_bCompress)->zero_tokens()->implicit_value(true), "Compress output data (if supported by output format)")
        ("count", po::value<boost::uint64_t>(&m_numPointsToWrite)->default_value(0), "How many points should we write?")
        ("skip", po::value<boost::uint64_t>(&m_numSkipPoints)->default_value(0), "How many points should we skip?")
        ("bounds", po::value<pdal::Bounds<double> >(&m_bounds), "Extent (in XYZ to clip output to)")
        ("polygon", po::value<std::string >(&m_wkt), "POLYGON WKT to use for precise crop of data (2d or 3d)")
        ;

    addSwitchSet(file_options);
    
    addPositionalSwitch("input", 1);
    addPositionalSwitch("output", 1);    
}


int Pc2Pc::execute()
{
    Options readerOptions;
    {
        readerOptions.add<std::string>("filename", m_inputFile);
        readerOptions.add<bool>("debug", isDebug());
        readerOptions.add<boost::uint32_t>("verbose", getVerboseLevel());

        if (!m_input_srs.empty())
        {
            readerOptions.add<std::string>("spatialreference", m_input_srs.getWKT());
        }
    }

    Options writerOptions;
    {
        writerOptions.add<std::string>("filename", m_outputFile);
        writerOptions.add<bool>("debug", isDebug());
        writerOptions.add<boost::uint32_t>("verbose", getVerboseLevel());

        if (!m_input_srs.empty())
        {
            writerOptions.add<std::string>("spatialreference", m_input_srs.getWKT());
        }

        if (m_bCompress)
        {
            writerOptions.add<bool>("compression", true);
        }

        if (m_chunkSize != 0)
        {
            writerOptions.add<boost::uint32_t>("chunk_size", m_chunkSize);
        }
    }

    Stage* reader_stage = AppSupport::makeReader(readerOptions);
    
    Stage* final_stage(0);
    if (!m_bounds.empty() || !m_wkt.empty() || !m_output_srs.empty())
    {
        Stage* next_stage = reader_stage;
        
        Stage* crop_stage(0);
        Stage* reprojection_stage(0);

        if (!m_output_srs.empty())
        {
            readerOptions.add<std::string >("out_srs", m_output_srs.getWKT());
            if (m_output_srs.isGeographic())
            {
                readerOptions.add<double >("scale_x", 0.0000001);
                readerOptions.add<double >("scale_y", 0.0000001);                
            } else
            {
                readerOptions.add<double >("scale_x", 0.01);
                readerOptions.add<double >("scale_y", 0.01);
            }
            reprojection_stage = new pdal::filters::InPlaceReprojection(*next_stage, readerOptions);
            next_stage = reprojection_stage;
        }
        
        if (!m_bounds.empty() && m_wkt.empty())
        {
            readerOptions.add<pdal::Bounds<double> >("bounds", m_bounds);
            crop_stage = new pdal::filters::Crop(*next_stage, readerOptions);
            next_stage = crop_stage;
        } 
        else if (m_bounds.empty() && !m_wkt.empty())
        {
            std::istream* wkt_stream;
            try
            {
                wkt_stream = FileUtils::openFile(m_wkt);
                std::stringstream buffer;
                buffer << wkt_stream->rdbuf();

                m_wkt = buffer.str();
                
            } catch (pdal::pdal_error const&)
            {
                // If we couldn't open the file given in m_wkt because it 
                // was likely actually wkt, leave it alone
            }
            readerOptions.add<std::string >("polygon", m_wkt);
            crop_stage = new pdal::filters::Crop(*next_stage, readerOptions);
            next_stage = crop_stage;
        }


        
        final_stage = next_stage;
    }
    
    if (final_stage == 0) 
        final_stage = reader_stage;
    
    Writer* writer = AppSupport::makeWriter(writerOptions, *final_stage);

    if (!m_output_srs.empty())
    {
        writer->setSpatialReference(m_output_srs);
    }

    writer->initialize();

    const boost::uint64_t numPointsToRead = final_stage->getNumPoints();
    boost::scoped_ptr<pdal::UserCallback> callback((numPointsToRead == 0) ? 
        (pdal::UserCallback*)(new HeartbeatCallback) :
        (pdal::UserCallback*)(new PercentageCallback));
    writer->setUserCallback(callback.get());

    const boost::uint64_t numPointsRead = writer->write(m_numPointsToWrite, m_numSkipPoints);

    std::cout << "Wrote " << numPointsRead << " points\n";

    delete writer;
    delete final_stage;

    return 0;
}


int main(int argc, char* argv[])
{
    Pc2Pc app(argc, argv);
    return app.run();
}
