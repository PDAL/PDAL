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

#include <buffer/BufferReader.hpp>
#include <pdal/KernelFactory.hpp>
#include <pdal/pdal_macros.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "kernels.smooth",
    "Smooth Kernel",
    "http://pdal.io/kernels/kernels.smooth.html" );

CREATE_SHARED_PLUGIN(1, 0, SmoothKernel, Kernel, s_info)

std::string SmoothKernel::getName() const { return s_info.name; }

void SmoothKernel::addSwitches(ProgramArgs& args)
{
    args.add("input,i", "Input filename", m_inputFile).setPositional();
    args.add("output,o", "Output filename", m_outputFile).setPositional();
}


int SmoothKernel::execute()
{
    PointTable table;

    Stage& readerStage(makeReader(m_inputFile, ""));

    // go ahead and prepare/execute on reader stage only to grab input
    // PointViewSet, this makes the input PointView available to both the
    // processing pipeline and the visualizer
    readerStage.prepare(table);
    PointViewSet viewSetIn = readerStage.execute(table);

    // the input PointViewSet will be used to populate a BufferReader that is
    // consumed by the processing pipeline
    PointViewPtr input_view = *viewSetIn.begin();

    std::shared_ptr<BufferReader> bufferReader(new BufferReader);
    Options bufferOptions;
    setCommonOptions(bufferOptions);
    bufferReader->setOptions(bufferOptions);
    bufferReader->addView(input_view);

    std::ostringstream ss;
    ss << "{";
    ss << "  \"pipeline\": {";
    ss << "    \"filters\": [{";
    ss << "      \"name\": \"MovingLeastSquares\"";
    ss << "      }]";
    ss << "    }";
    ss << "}";

    Options smoothOptions;
    smoothOptions.add("json", ss.str());

    Stage& smoothStage = makeFilter("filters.pclblock", *bufferReader);
    smoothStage.addOptions(smoothOptions);

    Stage& writer(Kernel::makeWriter(m_outputFile, smoothStage, ""));

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
