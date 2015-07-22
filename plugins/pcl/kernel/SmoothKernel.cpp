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

#include "SmoothKernel.hpp"

#include "PCLBlock.hpp"

#include <pdal/BufferReader.hpp>
#include <pdal/KernelFactory.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "kernels.smooth",
    "Smooth Kernel",
    "http://pdal.io/kernels/kernels.smooth.html" );

CREATE_SHARED_PLUGIN(1, 0, SmoothKernel, Kernel, s_info)

std::string SmoothKernel::getName() const { return s_info.name; }

void SmoothKernel::validateSwitches()
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


void SmoothKernel::addSwitches()
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


int SmoothKernel::execute()
{
    PointTable table;

    Options readerOptions;
    readerOptions.add("filename", m_inputFile);
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
    bufferReader->setOptions(readerOptions);
    bufferReader->addView(input_view);

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

    std::shared_ptr<Stage> smoothStage(new PCLBlock());
    smoothStage->setOptions(smoothOptions);
    smoothStage->setInput(*bufferReader);

    Options writerOptions;
    writerOptions.add("filename", m_outputFile);
    setCommonOptions(writerOptions);

    Stage& writer(Kernel::makeWriter(m_outputFile, *smoothStage));
    writer.setOptions(writerOptions);

    std::vector<std::string> cmd = getProgressShellCommand();
    UserCallback *callback =
        cmd.size() ? (UserCallback *)new ShellScriptCallback(cmd) :
        (UserCallback *)new HeartbeatCallback();

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

} // pdal
