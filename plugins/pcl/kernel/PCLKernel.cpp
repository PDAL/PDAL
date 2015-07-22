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

#include "PCLKernel.hpp"

#include "PCLBlock.hpp"

#include <pdal/BufferReader.hpp>
#include <pdal/KernelFactory.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "kernels.pcl",
    "PCL Kernel",
    "http://pdal.io/kernels/kernels.pcl.html" );

CREATE_SHARED_PLUGIN(1, 0, PCLKernel, Kernel, s_info)

std::string PCLKernel::getName() const { return s_info.name; }

PCLKernel::PCLKernel()
    : Kernel()
    , m_bCompress(false)
    , m_bForwardMetadata(false)
{
}

void PCLKernel::validateSwitches()
{
    if (m_inputFile == "")
        throw app_usage_error("--input/-i required");
    if (m_outputFile == "")
        throw app_usage_error("--output/-o required");
    if (m_pclFile == "")
        throw app_usage_error("--pcl/-p required");
}


void PCLKernel::addSwitches()
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
    ("metadata,m",
        po::value< bool >(&m_bForwardMetadata)->implicit_value(true),
        "Forward metadata (VLRs, header entries, etc) from previous stages")
    ;

    addSwitchSet(file_options);

    addPositionalSwitch("input", 1);
    addPositionalSwitch("output", 1);
    addPositionalSwitch("pcl", 1);
}


int PCLKernel::execute()
{
    PointTable table;

    Options readerOptions;
    readerOptions.add<std::string>("filename", m_inputFile);
    setCommonOptions(readerOptions);

    Stage& readerStage(Kernel::makeReader(m_inputFile));
    readerStage.setOptions(readerOptions);

    // go ahead and prepare/execute on reader stage only to grab input
    // PointViewSet, this makes the input PointView available to both the
    // processing pipeline and the visualizer
    readerStage.prepare(table);
    PointViewSet viewSetIn = readerStage.execute(table);

    // the input PointViewSet will be used to populate a BufferReader that is
    // consumed by the processing pipeline
    PointViewPtr input_view = *viewSetIn.begin();
    std::shared_ptr<BufferReader> bufferReader(new BufferReader);
    bufferReader->addView(input_view);

    Options pclOptions;
    pclOptions.add<std::string>("filename", m_pclFile);
    pclOptions.add<bool>("debug", isDebug());
    pclOptions.add<uint32_t>("verbose", getVerboseLevel());

    std::shared_ptr<Stage> pclStage(new PCLBlock());
    pclStage->setInput(*bufferReader);
    pclStage->setOptions(pclOptions);

    // the PCLBlock stage consumes the BufferReader rather than the
    // readerStage

    Options writerOptions;
    writerOptions.add<std::string>("filename", m_outputFile);
    setCommonOptions(writerOptions);

    if (m_bCompress)
        writerOptions.add<bool>("compression", true);
    if (m_bForwardMetadata)
        writerOptions.add("forward_metadata", true);

    std::vector<std::string> cmd = getProgressShellCommand();
    UserCallback *callback =
        cmd.size() ? (UserCallback *)new ShellScriptCallback(cmd) :
        (UserCallback *)new HeartbeatCallback();

    Stage& writer(Kernel::makeWriter(m_outputFile, *pclStage));

    // Some options are inferred by makeWriter based on filename
    // (compression, driver type, etc).
    writer.setOptions(writerOptions+writer.getOptions());

    writer.setUserCallback(callback);
    applyExtraStageOptionsRecursive(&writer);
    writer.prepare(table);

    // process the data, grabbing the PointViewSet for visualization of the
    // resulting PointView
    PointViewSet viewSetOut = writer.execute(table);

    if (isVisualize())
        visualize(*viewSetOut.begin());
    //visualize(*viewSetIn.begin(), *viewSetOut.begin());

    return 0;
}

} // namespace pdal
