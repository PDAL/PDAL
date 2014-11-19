/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
* Copyright (c) 2014, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "Ground.hpp"
#include "../filters/PCLBlock.hpp"
#include <pdal/kernel/KernelFactory.hpp>

#include "../drivers/buffer/BufferReader.hpp"

CREATE_KERNEL_PLUGIN(ground, pdal::kernel::Ground)

namespace pdal
{
namespace kernel
{

Ground::Ground()
    : Kernel()
    , m_inputFile("")
    , m_outputFile("")
    , m_maxWindowSize(33)
    , m_slope(1)
    , m_maxDistance(2.5)
    , m_initialDistance(0.15)
    , m_cellSize(1)
    , m_base(2)
    , m_exponential(true)
{
    return;
}


void Ground::validateSwitches()
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


void Ground::addSwitches()
{
    po::options_description* file_options = new po::options_description("file options");

    file_options->add_options()
    ("input,i", po::value<std::string>(&m_inputFile)->default_value(""), "input file name")
    ("output,o", po::value<std::string>(&m_outputFile)->default_value(""), "output file name")
//        ("compress,z", po::value<bool>(&m_bCompress)->zero_tokens()->implicit_value(true), "Compress output data (if supported by output format)")
    ("maxWindowSize", po::value<double>(&m_maxWindowSize)->default_value(33), "max window size")
    ("slope", po::value<double>(&m_slope)->default_value(1), "slope")
    ("maxDistance", po::value<double>(&m_maxDistance)->default_value(2.5), "max distance")
    ("initialDistance", po::value<double>(&m_initialDistance)->default_value(0.15, "0.15"), "initial distance")
    ("cellSize", po::value<double>(&m_cellSize)->default_value(1), "cell size")
    ("base", po::value<double>(&m_base)->default_value(2), "base")
    ("exponential", po::value<bool>(&m_exponential)->default_value(true), "exponential?")
    ;

    addSwitchSet(file_options);

    addPositionalSwitch("input", 1);
    addPositionalSwitch("output", 1);
}

std::unique_ptr<Stage> Ground::makeReader(Options readerOptions)
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

    Stage* stage = AppSupport::makeReader(m_inputFile);
    stage->setOptions(readerOptions);
    std::unique_ptr<Stage> reader_stage(stage);

    return reader_stage;
}


int Ground::execute()
{
    PointContext ctx;

    Options readerOptions;
    readerOptions.add<std::string>("filename", m_inputFile);
    readerOptions.add<bool>("debug", isDebug());
    readerOptions.add<boost::uint32_t>("verbose", getVerboseLevel());

    std::unique_ptr<Stage> readerStage = makeReader(readerOptions);

    // go ahead and prepare/execute on reader stage only to grab input
    // PointBufferSet, this makes the input PointBuffer available to both the
    // processing pipeline and the visualizer
    readerStage->prepare(ctx);
    PointBufferSet pbSetIn = readerStage->execute(ctx);

    // the input PointBufferSet will be used to populate a BufferReader that is
    // consumed by the processing pipeline
    PointBufferPtr input_buffer = *pbSetIn.begin();
    BufferReader bufferReader;
    bufferReader.setOptions(readerOptions);
    bufferReader.addBuffer(input_buffer);

    Options groundOptions;
    std::ostringstream ss;
    ss << "{";
    ss << "  \"pipeline\": {";
    ss << "    \"filters\": [{";
    ss << "      \"name\": \"ProgressiveMorphologicalFilter\",";
    ss << "      \"setMaxWindowSize\": " << m_maxWindowSize << ",";
    ss << "      \"setSlope\": " << m_slope << ",";
    ss << "      \"setMaxDistance\": " << m_maxDistance << ",";
    ss << "      \"setInitialDistance\": " << m_initialDistance << ",";
    ss << "      \"setCellSize\": " << m_cellSize << ",";
    ss << "      \"setBase\": " << m_base << ",";
    ss << "      \"setExponential\": " << m_exponential;
    ss << "      }]";
    ss << "    }";
    ss << "}";
    std::string json = ss.str();
    groundOptions.add<std::string>("json", json);
    groundOptions.add<bool>("debug", isDebug());
    groundOptions.add<boost::uint32_t>("verbose", getVerboseLevel());

    std::unique_ptr<Stage> groundStage(new filters::PCLBlock());
    groundStage->setInput(&bufferReader);
    groundStage->setOptions(groundOptions);

    // the PCLBlock groundStage consumes the BufferReader rather than the
    // readerStage
    groundStage->setInput(&bufferReader);

    Options writerOptions;
    writerOptions.add<std::string>("filename", m_outputFile);
    setCommonOptions(writerOptions);

    std::unique_ptr<Writer> writer(AppSupport::makeWriter(m_outputFile, groundStage.get()));
    writer->setOptions(writerOptions);

    std::vector<std::string> cmd = getProgressShellCommand();
    UserCallback *callback =
        cmd.size() ? (UserCallback *)new ShellScriptCallback(cmd) :
        (UserCallback *)new HeartbeatCallback();

    writer->setUserCallback(callback);

    for (auto pi: getExtraStageOptions())
    {
        std::string name = pi.first;
        Options options = pi.second;
        std::vector<Stage*> stages = writer->findStage(name);
        for (auto s: stages)
        {
            Options opts = s->getOptions();
            for (auto o: options.getOptions())
                opts.add(o);
            s->setOptions(opts);
        }
    }

    writer->prepare(ctx);

    // process the data, grabbing the PointBufferSet for visualization of the
    // resulting PointBuffer
    PointBufferSet pbSetOut = writer->execute(ctx);

    if (isVisualize())
        visualize(*pbSetOut.begin());
    //visualize(*pbSetIn.begin(), *pbSetOut.begin());

    return 0;
}

} // kernel
} // pdal
