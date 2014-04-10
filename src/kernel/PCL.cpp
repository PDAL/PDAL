/******************************************************************************
 * Copyright (c) 2014, Brad Chambers (brad.chambers@gmail.com)
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

#include <pdal/kernel/PCL.hpp>
#include <pdal/filters/Cache.hpp>
#include <pdal/filters/PCLBlock.hpp>
#include <pdal/filters/Selector.hpp>

namespace pdal
{
namespace kernel
{

PCL::PCL(int argc, const char* argv[])
    : Application(argc, argv, "pcl")
    , m_bCompress(false)
{
    return;
}

void PCL::validateSwitches()
{
    if (m_inputFile == "")
    {
        throw app_usage_error("--input/-i required");
    }

    if (m_outputFile == "")
    {
        throw app_usage_error("--output/-o required");
    }

    if (m_pclFile == "")
    {
        throw app_usage_error("--pcl/-p required");
    }

    return;
}

void PCL::addSwitches()
{
    po::options_description* file_options = new po::options_description("file options");

    file_options->add_options()
    ("input,i", po::value<std::string>(&m_inputFile)->default_value(""), "input file name")
    ("output,o", po::value<std::string>(&m_outputFile)->default_value(""), "output file name")
    ("pcl,p", po::value<std::string>(&m_pclFile)->default_value(""), "pcl file name")
    ("compress,z", po::value<bool>(&m_bCompress)->zero_tokens()->implicit_value(true), "Compress output data (if supported by output format)")
    ;

    addSwitchSet(file_options);

    addPositionalSwitch("input", 1);
    addPositionalSwitch("output", 1);
    addPositionalSwitch("pcl", 1);
}

Stage* PCL::makeReader(Options readerOptions)
{
    if (isDebug())
    {
        readerOptions.add<bool>("debug", true);
        boost::uint32_t verbosity(getVerboseLevel());
        if (!verbosity)
        {
            verbosity = 1;
        }

        readerOptions.add<boost::uint32_t>("verbose", verbosity);
        readerOptions.add<std::string>("log", "STDERR");
    }

    Stage* reader_stage = AppSupport::makeReader(readerOptions);

    Options keeps;
    Option x("dimension", "X", "x dim");
    Option y("dimension", "Y", "y dim");
    Option z("dimension", "Z", "z dim");
    keeps.add(x);
    keeps.add(y);
    keeps.add(z);

    Options selectorOptions;
    selectorOptions.add<bool>("debug", isDebug());
    selectorOptions.add<boost::uint32_t>("verbose", getVerboseLevel());
    selectorOptions.add<bool>("ignore_default", true);
    selectorOptions.add<Options>("keep", keeps);

    Stage* selector_stage = new pdal::filters::Selector(*reader_stage, selectorOptions);

    Options cacheOptions;
    cacheOptions.add<bool>("debug", isDebug());
    cacheOptions.add<boost::uint32_t>("verbose", getVerboseLevel());
    cacheOptions.add<boost::uint32_t>("max_cache_blocks", 1);

    Stage* cache_stage = new pdal::filters::Cache(*selector_stage, cacheOptions);

    Options pclOptions;
    pclOptions.add<std::string>("filename", m_pclFile);
    pclOptions.add<bool>("debug", isDebug());
    pclOptions.add<boost::uint32_t>("verbose", getVerboseLevel());

    Stage* pcl_stage = new pdal::filters::PCLBlock(*cache_stage, pclOptions);

    return pcl_stage;
}

int PCL::execute()
{
    Options readerOptions;
    readerOptions.add<std::string>("filename", m_inputFile);
    readerOptions.add<bool>("debug", isDebug());
    readerOptions.add<boost::uint32_t>("verbose", getVerboseLevel());

    Options writerOptions;
    writerOptions.add<std::string>("filename", m_outputFile);
    writerOptions.add<bool>("debug", isDebug());
    writerOptions.add<boost::uint32_t>("verbose", getVerboseLevel());

    if (m_bCompress)
    {
        writerOptions.add<bool>("compression", true);
    }

    Stage* final_stage = makeReader(readerOptions);

    Writer* writer = AppSupport::makeWriter(writerOptions, *final_stage);

    writer->initialize();

    const boost::uint64_t numPointsToRead = final_stage->getNumPoints();

    std::cerr << "Requested to read " << numPointsToRead << " points" << std::endl;

    pdal::UserCallback* callback;
    if (!getProgressShellCommand().size())
    {
        if (numPointsToRead == 0)
        {
            callback = static_cast<pdal::UserCallback*>(new HeartbeatCallback);
        }
        else
        {
            callback = static_cast<pdal::UserCallback*>(new PercentageCallback);
        }
    }
    else
    {
        callback = static_cast<pdal::UserCallback*>(new ShellScriptCallback(getProgressShellCommand()));
    }
    writer->setUserCallback(callback);

    const boost::uint64_t numPointsRead = writer->write(numPointsToRead, 0, m_chunkSize);

    std::cerr << "Wrote " << numPointsRead << " points\n";

    delete writer;
    delete final_stage;

    return 0;
}

} // kernel
} // pdal

