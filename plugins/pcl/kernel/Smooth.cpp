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

#include "Smooth.hpp"
#include "../filters/PCLBlock.hpp"
#include <pdal/kernel/KernelFactory.hpp>

#include "../drivers/buffer/BufferReader.hpp"

CREATE_KERNEL_PLUGIN(smooth, pdal::kernel::Smooth)

namespace pdal
{
namespace kernel
{


void Smooth::validateSwitches()
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


void Smooth::addSwitches()
{
    po::options_description* file_options = new po::options_description("file options");

    file_options->add_options()
    ("input,i", po::value<std::string>(&m_inputFile)->default_value(""), "input file name")
    ("output,o", po::value<std::string>(&m_outputFile)->default_value(""), "output file name")
//        ("compress,z", po::value<bool>(&m_bCompress)->zero_tokens()->implicit_value(true), "Compress output data (if supported by output format)")
    ;

    addSwitchSet(file_options);

    addPositionalSwitch("input", 1);
    addPositionalSwitch("output", 1);
}

std::unique_ptr<Stage> Smooth::makeReader(Options readerOptions)
{
    if (isDebug())
    {
        readerOptions.add("debug", true);
        uint32_t verbosity(getVerboseLevel());
        if (!verbosity)
            verbosity = 1;

        readerOptions.add("verbose", verbosity);
        readerOptions.add("log", "STDERR");
    }

    Stage* stage = AppSupport::makeReader(m_inputFile);
    stage->setOptions(readerOptions);
    std::unique_ptr<Stage> reader_stage(stage);

    return reader_stage;
}


int Smooth::execute()
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

    Options smoothOptions;
    std::ostringstream ss;
    ss << "{";
    ss << "  \"pipeline\": {";
    ss << "    \"filters\": [{";
    ss << "      \"name\": \"MovingLeastSquares\"";
    ss << "      }]";
    ss << "    }";
    ss << "}";
    std::string json = ss.str();
    smoothOptions.add("json", json);
    smoothOptions.add("debug", isDebug());
    smoothOptions.add("verbose", getVerboseLevel());

    std::unique_ptr<Stage> smoothStage(new filters::PCLBlock());
    smoothStage->setOptions(smoothOptions);
    smoothStage->setInput(&bufferReader);

    Options writerOptions;
    writerOptions.add("filename", m_outputFile);
    setCommonOptions(writerOptions);

    std::unique_ptr<Writer> writer(AppSupport::makeWriter(m_outputFile, smoothStage.get()));
    writer->setOptions(writerOptions);

    std::vector<std::string> cmd = getProgressShellCommand();
    UserCallback *callback =
        cmd.size() ? (UserCallback *)new ShellScriptCallback(cmd) :
        (UserCallback *)new HeartbeatCallback();

    writer->setUserCallback(callback);

    std::map<std::string, Options> extra_opts = getExtraStageOptions();
    std::map<std::string, Options>::iterator pi;
    for (pi = extra_opts.begin(); pi != extra_opts.end(); ++pi)
    {
        std::string name = pi->first;
        Options options = pi->second;
        std::vector<Stage*> stages = writer->findStage(name);
        std::vector<Stage*>::iterator s;
        for (s = stages.begin(); s != stages.end(); ++s)
        {
            Options opts = (*s)->getOptions();
            std::vector<Option>::iterator o;
            for (o = options.getOptions().begin(); o != options.getOptions().end(); ++o)
                opts.add(*o);
            (*s)->setOptions(opts);
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
