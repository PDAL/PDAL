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

#include <pdal/drivers/buffer/BufferReader.hpp>
#include <pdal/filters/PCLBlock.hpp>

namespace pdal
{
namespace kernel
{

PCL::PCL(int argc, const char* argv[])
    : Application(argc, argv, "pcl")
    , m_bCompress(false)
{
}

void PCL::validateSwitches()
{
    if (m_inputFile == "")
        throw app_usage_error("--input/-i required");
    if (m_outputFile == "")
        throw app_usage_error("--output/-o required");
    if (m_pclFile == "")
        throw app_usage_error("--pcl/-p required");
}


void PCL::addSwitches()
{
    po::options_description* file_options =
        new po::options_description("file options");

    file_options->add_options()
    ("input,i", po::value<std::string>(&m_inputFile)->default_value(""),
        "input file name")
    ("output,o", po::value<std::string>(&m_outputFile)->default_value(""),
        "output file name")
    ("pcl,p", po::value<std::string>(&m_pclFile)->default_value(""),
        "pcl file name")
    ("compress,z",
        po::value<bool>(&m_bCompress)->zero_tokens()->implicit_value(true),
        "Compress output data (if supported by output format)")
    ;

    addSwitchSet(file_options);

    addPositionalSwitch("input", 1);
    addPositionalSwitch("output", 1);
    addPositionalSwitch("pcl", 1);
}

std::unique_ptr<Stage> PCL::makeReader(Options readerOptions)
{
    std::unique_ptr<Stage> reader_stage(AppSupport::makeReader(m_inputFile));
    if (isDebug())
    {
        readerOptions.add<bool>("debug", true);
        boost::uint32_t verbosity(getVerboseLevel());
        if (!verbosity)
            verbosity = 1;

        readerOptions.add<boost::uint32_t>("verbose", verbosity);
        readerOptions.add<std::string>("log", "STDERR");
        reader_stage->setOptions(readerOptions);
    }


    return reader_stage;
}


int PCL::execute()
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
    drivers::buffer::BufferReader bufferReader;
    bufferReader.addBuffer(input_buffer);

    Options pclOptions;
    pclOptions.add<std::string>("filename", m_pclFile);
    pclOptions.add<bool>("debug", isDebug());
    pclOptions.add<boost::uint32_t>("verbose", getVerboseLevel());

    std::unique_ptr<Stage> pclStage(new filters::PCLBlock());
    pclStage->setInput(&bufferReader);
    pclStage->setOptions(pclOptions);

    // the PCLBlock stage consumes the BufferReader rather than the
    // readerStage

    Options writerOptions;
    writerOptions.add<std::string>("filename", m_outputFile);
    setCommonOptions(writerOptions);

    if (m_bCompress)
        writerOptions.add<bool>("compression", true);

    std::vector<std::string> cmd = getProgressShellCommand();
    UserCallback *callback =
        cmd.size() ? (UserCallback *)new ShellScriptCallback(cmd) :
        (UserCallback *)new HeartbeatCallback();

    std::unique_ptr<Writer>
        writer(AppSupport::makeWriter(m_outputFile, pclStage.get()));
    writer->setOptions(writerOptions);

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

    visualize(*pbSetIn.begin(), *pbSetOut.begin());

    return 0;
}

} // namespace kernel
} // namespace pdal

