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


namespace pdal { namespace kernel {

Translate::Translate(int argc, const char* argv[])
    : Application(argc, argv, "translate")
    , m_inputFile("")
    , m_outputFile("")
    , m_bCompress(false)
    , m_numPointsToWrite(0)
    , m_numSkipPoints(0)
    , m_input_srs(pdal::SpatialReference())
    , m_output_srs(pdal::SpatialReference())
    , m_wkt("")
    , m_scales("")
    , m_offsets("")
    , m_bForwardMetadata(false)
{
    return;
}


void Translate::validateSwitches()
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


void Translate::addSwitches()
{
    
    // Action translate("translate");
    
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
        ("scale", po::value< std::string >(&m_scales), "A comma-separated or quoted, space-separated list of scales to set on the output file: \n--scale 0.1,0.1,0.00001\n--scale \"0.1 0.1 0.00001\"")
        ("offset", po::value< std::string >(&m_offsets), "A comma-separated or quoted, space-separated list of offsets to set on the output file: \n--offset 0,0,0\n--offset \"1234 5678 91011\"")
        ("metadata,m", po::value< bool >(&m_bForwardMetadata)->implicit_value(true), "Forward metadata (VLRs, header entries, etc) from previous stages")
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
    
    Stage* final_stage(0);
    if (!m_bounds.empty() || !m_wkt.empty() || !m_output_srs.empty())
    {
        Stage* next_stage = reader_stage;
        
        Stage* crop_stage(0);
        Stage* reprojection_stage(0);

        if (!m_output_srs.empty())
        {
            readerOptions.add<std::string >("out_srs", m_output_srs.getWKT());

            boost::char_separator<char> sep(SEPARATORS);
            std::vector<double> offsets;
            tokenizer off_tokens(m_offsets, sep);
            for (tokenizer::iterator t = off_tokens.begin(); t != off_tokens.end(); ++t) {
                offsets.push_back(boost::lexical_cast<double>(*t));
            }

            std::vector<double> scales;
            tokenizer scale_tokens(m_scales, sep);
            for (tokenizer::iterator t = scale_tokens.begin(); t != scale_tokens.end(); ++t) {
                scales.push_back(boost::lexical_cast<double>(*t));
            }
            
            if (scales.size())
            {
                if (scales.size() <= 1)
                {
                    readerOptions.add<double >("scale_x", scales[0]);
                    
                }
                else if (scales.size() <= 2)
                {
                    readerOptions.add<double >("scale_x", scales[0]);
                    readerOptions.add<double >("scale_y", scales[1]);
                }
                else if (scales.size() <= 3)
                {
                    readerOptions.add<double >("scale_x", scales[0]);
                    readerOptions.add<double >("scale_y", scales[1]);
                    readerOptions.add<double >("scale_z", scales[2]);
                }
            }

            if (offsets.size())
            {
                if (offsets.size() <= 1)
                {
                    readerOptions.add<double >("offset_x", offsets[0]);
                    
                }
                else if (offsets.size() <= 2)
                {
                    readerOptions.add<double >("offset_x", offsets[0]);
                    readerOptions.add<double >("offset_y", offsets[1]);
                }
                else if (offsets.size() <= 3)
                {
                    readerOptions.add<double >("offset_x", offsets[0]);
                    readerOptions.add<double >("offset_y", offsets[1]);
                    readerOptions.add<double >("offset_z", offsets[2]);
                }
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
                FileUtils::closeFile(wkt_stream);
                
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
    
    return final_stage;    

}

void Translate::forwardMetadata(Options& options, Metadata metadata)
{
    // boost::property_tree::ptree::const_iterator m;
    // Metadata mdata = metadata.getMetadata("drivers.las.reader");
    // boost::property_tree::ptree const& entries = mdata.toPTree();
    // Option metadata_opt;
    // boost::property_tree::xml_parser::write_xml(std::cout, entries);
    // for (m = entries.begin(); m != entries.end(); ++m)
    // {
    //     std::string const& name = m->first;
    //     std::string value = m->second.get_value<std::string>();
    // }            

}


int Translate::execute()
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

        if (m_bForwardMetadata)
        {
            writerOptions.add<bool>("forward_metadata", true);
        }

    }

    
    Stage* final_stage = makeReader(readerOptions);    
    
    Writer* writer = AppSupport::makeWriter(writerOptions, *final_stage);

    if (!m_output_srs.empty())
    {
        writer->setSpatialReference(m_output_srs);
    }

    writer->initialize();

    const boost::uint64_t numPointsToRead = final_stage->getNumPoints();
    
    if (m_numPointsToWrite == 0)
        m_numPointsToWrite = numPointsToRead;

    std::cerr << "Requested to read " << numPointsToRead << " points" << std::endl;
    std::cerr << "Requested to write " << m_numPointsToWrite << " points" << std::endl;
    // std::cerr << "Buffer capacity is " << writer->getChunkSize() << std::endl;
        
    pdal::UserCallback* callback;
    if (!getProgressShellCommand().size())
        if (m_numPointsToWrite == 0)
            callback = static_cast<pdal::UserCallback*>(new HeartbeatCallback);
        else
            callback = static_cast<pdal::UserCallback*>(new PercentageCallback);
    else
        callback = static_cast<pdal::UserCallback*>(new ShellScriptCallback(getProgressShellCommand()));
    writer->setUserCallback(callback);

    const boost::uint64_t numPointsRead = writer->write(m_numPointsToWrite, m_numSkipPoints);

    std::cerr << "Wrote " << numPointsRead << " points\n";

    delete writer;
    delete final_stage;

    return 0;
}

}} // pdal::kernel
