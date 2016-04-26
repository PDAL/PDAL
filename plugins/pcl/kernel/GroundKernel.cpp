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

void GroundKernel::addSwitches(ProgramArgs& args)
{
    args.add("input,i", "Input filename", m_inputFile).setPositional();
    args.add("output,o", "Output filename", m_outputFile).setPositional();
    args.add("max_window_size", "Max window size", m_maxWindowSize, 33.0);
    args.add("slope", "Slope", m_slope, 1.0);
    args.add("max_distance", "Max distance", m_maxDistance, 2.5);
    args.add("initial_distance", "Initial distance", m_initialDistance, .15);
    args.add("cell_size", "Cell size", m_cellSize, 1.0);
    args.add("classify", "Apply classification labels?", m_classify);
    args.add("extract", "extract ground returns?", m_extract);
    args.add("approximate,a", "use approximate algorithm? (much faster)",
        m_approximate);
}

int GroundKernel::execute()
{
    PointTable table;

    Stage& readerStage(makeReader(m_inputFile, ""));

    Options groundOptions;
    groundOptions.add("max_window_size", m_maxWindowSize);
    groundOptions.add("slope", m_slope);
    groundOptions.add("max_distance", m_maxDistance);
    groundOptions.add("initial_distance", m_initialDistance);
    groundOptions.add("cell_size", m_cellSize);
    groundOptions.add("classify", m_classify);
    groundOptions.add("extract", m_extract);
    groundOptions.add("approximate", m_approximate);

    Stage& groundStage = makeFilter("filters.ground", readerStage);
    groundStage.addOptions(groundOptions);

    // setup the Writer and write the results
    Stage& writer(makeWriter(m_outputFile, groundStage, ""));

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
