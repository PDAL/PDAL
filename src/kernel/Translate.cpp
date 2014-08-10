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

#include <pdal/kernel/Translate.hpp>

#include <boost/tokenizer.hpp>
typedef boost::tokenizer<boost::char_separator<char>> tokenizer;

#include <pdal/filters/Crop.hpp>
#include <pdal/filters/Decimation.hpp>
#include <pdal/filters/Reprojection.hpp>
#include <pdal/kernel/Support.hpp>

namespace pdal
{
namespace kernel
{

Translate::Translate(int argc, const char* argv[]) :
    Application(argc, argv, "translate"), m_bCompress(false),
    m_numPointsToWrite(0), m_numSkipPoints(0),
    m_input_srs(pdal::SpatialReference()),
    m_output_srs(pdal::SpatialReference()), m_bForwardMetadata(false),
    m_decimation_step(1), m_decimation_offset(0)
{}


void Translate::validateSwitches()
{
    if (m_inputFile == "")
        throw app_usage_error("--input/-i required");
    if (m_outputFile == "")
        throw app_usage_error("--output/-o required");
}


void Translate::addSwitches()
{
    po::options_description* file_options =
        new po::options_description("file options");

    file_options->add_options()
        ("input,i", po::value<std::string>(&m_inputFile)->default_value(""),
         "input file name")
        ("output,o", po::value<std::string>(&m_outputFile)->default_value(""),
         "output file name")
        ("a_srs", po::value<pdal::SpatialReference>(&m_input_srs),
         "Assign input coordinate system (if supported by output format)")
        ("t_srs", po::value<pdal::SpatialReference>(&m_output_srs),
         "Transform to output coordinate system (if supported by "
         "output format)")
        ("compress,z",
         po::value<bool>(&m_bCompress)->zero_tokens()->implicit_value(true),
         "Compress output data (if supported by output format)")
        ("count",
         po::value<boost::uint64_t>(&m_numPointsToWrite)->default_value(0),
         "How many points should we write?")
        ("skip", po::value<boost::uint64_t>(&m_numSkipPoints)->default_value(0),
         "How many points should we skip?")
        ("bounds", po::value<pdal::Bounds<double> >(&m_bounds),
         "Extent (in XYZ to clip output to)")
        ("polygon", po::value<std::string >(&m_wkt),
         "POLYGON WKT to use for precise crop of data (2d or 3d)")
        ("scale", po::value< std::string >(&m_scales),
         "A comma-separated or quoted, space-separated list of scales to "
         "set on the output file: \n--scale 0.1,0.1,0.00001\n--scale \""
         "0.1 0.1 0.00001\"")
        ("offset", po::value< std::string >(&m_offsets),
         "A comma-separated or quoted, space-separated list of offsets to "
         "set on the output file: \n--offset 0,0,0\n--offset "
         "\"1234 5678 91011\"")
        ("metadata,m",
         po::value< bool >(&m_bForwardMetadata)->implicit_value(true),
         "Forward metadata (VLRs, header entries, etc) from previous stages")
        ("d_step",
         po::value<boost::uint32_t>(&m_decimation_step)->default_value(1),
         "Decimation filter step")
        ("d_offset",
         po::value<boost::uint32_t>(&m_decimation_offset)->default_value(0),
         "Decimation filter offset")
        ;

    addSwitchSet(file_options);
    addPositionalSwitch("input", 1);
    addPositionalSwitch("output", 1);    
}

Stage* Translate::makeReader(Options readerOptions)
{
    if (isDebug())
    {
        readerOptions.add<bool>("debug", true);
        boost::uint32_t verbosity(getVerboseLevel());
        if (!verbosity)
            verbosity = 1;
        
        readerOptions.add<boost::uint32_t>("verbose", verbosity);
        readerOptions.add<std::string>("log", "STDERR");
    }

    Stage* reader_stage = AppSupport::makeReader(readerOptions);
    Stage* final_stage = reader_stage;
    if (!m_bounds.empty() || !m_wkt.empty() || !m_output_srs.empty())
    {
        Stage* next_stage = reader_stage;
        Stage* crop_stage(0);
        Stage* reprojection_stage(0);

        if (!m_output_srs.empty())
        {
            readerOptions.add<std::string>("out_srs", m_output_srs.getWKT());
            reprojection_stage =
                new filters::Reprojection(readerOptions);
            reprojection_stage->setInput(next_stage);
            next_stage = reprojection_stage;
        }
        
        if (!m_bounds.empty() && m_wkt.empty())
        {
            readerOptions.add<pdal::Bounds<double>>("bounds", m_bounds);
            crop_stage = new pdal::filters::Crop(readerOptions);
            crop_stage->setInput(next_stage);
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
                FileUtils::closeFile(wkt_stream);
                
            }
            catch (pdal::pdal_error const&)
            {
                // If we couldn't open the file given in m_wkt because it 
                // was likely actually wkt, leave it alone
            }
            readerOptions.add<std::string >("polygon", m_wkt);
            crop_stage = new pdal::filters::Crop(readerOptions);
            crop_stage->setInput(next_stage);
            next_stage = crop_stage;
        }
        final_stage = next_stage;
    }
 
    if (m_decimation_step > 1)
    {
        Options decimationOptions;
        decimationOptions.add<bool>("debug", isDebug());
        decimationOptions.add<uint32_t>("verbose", getVerboseLevel());
        decimationOptions.add<uint32_t>("step", m_decimation_step);
        decimationOptions.add<uint32_t>("offset", m_decimation_offset);
        Stage *decimation_stage = new filters::Decimation(decimationOptions);
        decimation_stage->setInput(final_stage);
        final_stage = decimation_stage;
    }
    return final_stage;    
}

int Translate::execute()
{
    Options readerOptions;
    {
        readerOptions.add<std::string>("filename", m_inputFile);
        readerOptions.add<bool>("debug", isDebug());
        readerOptions.add<uint32_t>("verbose", getVerboseLevel());
        if (!m_input_srs.empty())
            readerOptions.add<std::string>("spatialreference",
                m_input_srs.getWKT());
    }

    Options writerOptions;
    {
        writerOptions.add<std::string>("filename", m_outputFile);
        writerOptions.add<bool>("debug", isDebug());
        writerOptions.add<uint32_t>("verbose", getVerboseLevel());
    
        if (!m_input_srs.empty())
            writerOptions.add<std::string>("spatialreference",
                m_input_srs.getWKT());

        if (m_bCompress)
            writerOptions.add<bool>("compression", true);
        if (m_bForwardMetadata)
            writerOptions.add<bool>("forward_metadata", true);
    }
    Stage* final_stage = makeReader(readerOptions);

    std::vector<std::string> cmd = getProgressShellCommand();
    UserCallback *callback =
        cmd.size() ? (UserCallback *)new ShellScriptCallback(cmd) :
        m_numPointsToWrite ? (UserCallback *)new PercentageCallback() :
        (UserCallback *)new HeartbeatCallback();

    Writer* writer = AppSupport::makeWriter(writerOptions, *final_stage);
    if (!m_output_srs.empty())
        writer->setSpatialReference(m_output_srs);

    PointContext ctx;
    writer->setUserCallback(callback);
    writer->prepare(ctx);
    writer->execute(ctx);

    //ABELL - This looks like there are probably stage leaks.
    delete writer;
    delete final_stage;

    return 0;
}

} // namespace kernel
} // namespace pdal
