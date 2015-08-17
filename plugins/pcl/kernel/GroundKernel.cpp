/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
* Copyright (c) 2014-2015, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "GroundKernel.hpp"

#include <pdal/KernelFactory.hpp>
#include <pdal/KernelSupport.hpp>
#include <pdal/Options.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Stage.hpp>
#include <pdal/StageFactory.hpp>

#include <memory>
#include <string>
#include <vector>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "kernels.ground",
    "Ground Kernel",
    "http://pdal.io/kernels/kernels.ground.html" );

CREATE_SHARED_PLUGIN(1, 0, GroundKernel, Kernel, s_info)

std::string GroundKernel::getName() const { return s_info.name; }

GroundKernel::GroundKernel()
    : Kernel()
    , m_inputFile("")
    , m_outputFile("")
    , m_maxWindowSize(33)
    , m_slope(1)
    , m_maxDistance(2.5)
    , m_initialDistance(0.15)
    , m_cellSize(1)
    , m_classify(true)
    , m_extract(false)
    , m_approximate(false)
{}

void GroundKernel::validateSwitches()
{
    if (m_inputFile == "")
    {
        throw app_usage_error("--input/-i required");
    }

    if (m_outputFile == "")
    {
        throw app_usage_error("--output/-o required");
    }
}

void GroundKernel::addSwitches()
{
    po::options_description* file_options = new po::options_description("file options");

    file_options->add_options()
    ("input,i", po::value<std::string>(&m_inputFile)->default_value(""), "input file name")
    ("output,o", po::value<std::string>(&m_outputFile)->default_value(""), "output file name")
    ("maxWindowSize", po::value<double>(&m_maxWindowSize)->default_value(33), "max window size")
    ("slope", po::value<double>(&m_slope)->default_value(1), "slope")
    ("maxDistance", po::value<double>(&m_maxDistance)->default_value(2.5), "max distance")
    ("initialDistance", po::value<double>(&m_initialDistance)->default_value(0.15, "0.15"), "initial distance")
    ("cellSize", po::value<double>(&m_cellSize)->default_value(1), "cell size")
    ("classify", po::bool_switch(&m_classify), "apply classification labels?")
    ("extract", po::bool_switch(&m_extract), "extract ground returns?")
    ("approximate,a", po::bool_switch(&m_approximate), "use approximate algorithm? (much faster)")
    ;

    addSwitchSet(file_options);

    addPositionalSwitch("input", 1);
    addPositionalSwitch("output", 1);
}

int GroundKernel::execute()
{
    PointTable table;

    Options readerOptions;
    readerOptions.add<std::string>("filename", m_inputFile);
    setCommonOptions(readerOptions);

    Stage& readerStage(Kernel::makeReader(m_inputFile));
    readerStage.setOptions(readerOptions);

    Options groundOptions;
    groundOptions.add<double>("maxWindowSize", m_maxWindowSize);
    groundOptions.add<double>("slope", m_slope);
    groundOptions.add<double>("maxDistance", m_maxDistance);
    groundOptions.add<double>("initialDistance", m_initialDistance);
    groundOptions.add<double>("cellSize", m_cellSize);
    groundOptions.add<bool>("classify", m_classify);
    groundOptions.add<bool>("extract", m_extract);
    groundOptions.add<bool>("approximate", m_approximate);
    groundOptions.add<bool>("debug", isDebug());
    groundOptions.add<uint32_t>("verbose", getVerboseLevel());

    StageFactory f;
    std::unique_ptr<Stage> groundStage(f.createStage("filters.ground"));
    groundStage->setOptions(groundOptions);
    groundStage->setInput(readerStage);

    // setup the Writer and write the results
    Options writerOptions;
    writerOptions.add<std::string>("filename", m_outputFile);
    setCommonOptions(writerOptions);

    Stage& writer(Kernel::makeWriter(m_outputFile, *groundStage));
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

} // namespace pdal
