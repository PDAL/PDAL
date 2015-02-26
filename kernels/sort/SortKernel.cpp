/******************************************************************************
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

#include "SortKernel.hpp"

#include <pdal/BufferReader.hpp>
#include <pdal/KernelSupport.hpp>
#include <pdal/StageFactory.hpp>

#include <boost/program_options.hpp>

namespace pdal
{

static PluginInfo const s_info {
    "kernels.sort",
    "Sort Kernel",
    "http://pdal.io/kernels/kernels.sort.html" };

CREATE_STATIC_PLUGIN(SortKernel, Kernel, s_info)

SortKernel::SortKernel() :
    Kernel(), m_bCompress(false), m_bForwardMetadata(false)
{}


void SortKernel::validateSwitches()
{
    if (m_inputFile == "")
        throw app_usage_error("--input/-i required");
    if (m_outputFile == "")
        throw app_usage_error("--output/-o required");
}


void SortKernel::addSwitches()
{
    po::options_description* file_options =
        new po::options_description("file options");

    file_options->add_options()
    ("input,i", po::value<std::string>(&m_inputFile)->default_value(""),
     "input file name")
    ("output,o", po::value<std::string>(&m_outputFile)->default_value(""),
     "output file name")
    ("compress,z",
     po::value<bool>(&m_bCompress)->zero_tokens()->implicit_value(true),
     "Compress output data (if supported by output format)")
    ("metadata,m",
     po::value< bool >(&m_bForwardMetadata)->implicit_value(true),
     "Forward metadata (VLRs, header entries, etc) from previous stages")
    ;

    addSwitchSet(file_options);
    addPositionalSwitch("input", 1);
    addPositionalSwitch("output", 1);
}

std::unique_ptr<Stage> SortKernel::makeReader(Options readerOptions)
{
    if (isDebug())
    {
        readerOptions.add<bool>("debug", true);
        uint32_t verbosity(getVerboseLevel());
        if (!verbosity)
            verbosity = 1;

        readerOptions.add<uint32_t>("verbose", verbosity);
        readerOptions.add<std::string>("log", "STDERR");
    }

    Stage* stage = KernelSupport::makeReader(m_inputFile);
    stage->setOptions(readerOptions);
    std::unique_ptr<Stage> reader_stage(stage);

    return reader_stage;
}


int SortKernel::execute()
{
    PointContext ctx;

    Options readerOptions;
    readerOptions.add("filename", m_inputFile);
    readerOptions.add("debug", isDebug());
    readerOptions.add("verbose", getVerboseLevel());

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

    Options sortOptions;
    sortOptions.add<bool>("debug", isDebug());
    sortOptions.add<uint32_t>("verbose", getVerboseLevel());

    StageFactory f;
    Stage* sortStage = f.createStage2("filters.mortonorder");
    sortStage->setInput(&bufferReader);
    sortStage->setOptions(sortOptions);

    Options writerOptions;
    writerOptions.add("filename", m_outputFile);
    setCommonOptions(writerOptions);

    if (m_bCompress)
        writerOptions.add("compression", true);
    if (m_bForwardMetadata)
        writerOptions.add("forward_metadata", true);

    std::vector<std::string> cmd = getProgressShellCommand();
    UserCallback *callback =
        cmd.size() ? (UserCallback *)new ShellScriptCallback(cmd) :
        (UserCallback *)new HeartbeatCallback();

    std::unique_ptr<Stage> writer(KernelSupport::makeWriter(m_outputFile, sortStage));

    // Some options are inferred by makeWriter based on filename
    // (compression, driver type, etc).
    writer->setOptions(writerOptions+writer->getOptions());

    writer->setUserCallback(callback);

    for (const auto& pi : getExtraStageOptions())
    {
        std::string name = pi.first;
        Options options = pi.second;
        std::vector<Stage*> stages = writer->findStage(name);
        for (const auto& s : stages)
        {
            Options opts = s->getOptions();
            for (const auto& o : options.getOptions())
                opts.add(o);
            s->setOptions(opts);
        }
    }
    writer->prepare(ctx);

    // process the data, grabbing the PointBufferSet for visualization of the
    PointBufferSet pbSetOut = writer->execute(ctx);

    if (isVisualize())
        visualize(*pbSetOut.begin());
    //visualize(*pbSetIn.begin(), *pbSetOut.begin());

    return 0;
}

} // namespace pdal
