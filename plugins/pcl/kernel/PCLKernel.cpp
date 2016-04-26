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

#include <buffer/BufferReader.hpp>
#include <pdal/KernelFactory.hpp>
#include <pdal/pdal_macros.hpp>

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
{}


void PCLKernel::addSwitches(ProgramArgs& args)
{
    args.add("input,i", "Input filename", m_inputFile).setPositional();
    args.add("output,o", "Output filename", m_outputFile).setPositional();
    args.add("pcl,p", "PCL filename", m_pclFile).setPositional();
    args.add("compress,z",
        "Compress output data (if supported by output format)",
        m_bCompress);
    args.add("metadata,m",
        "Forward metadata (VLRs, header entries, etc) from previous stages",
        m_bForwardMetadata);
}


int PCLKernel::execute()
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
    bufferReader->addView(input_view);

    Options pclOptions;
    pclOptions.add("filename", m_pclFile);

    Stage& pclStage = makeFilter("filters.pclblock", *bufferReader);
    pclStage.addOptions(pclOptions);

    // the PCLBlock stage consumes the BufferReader rather than the
    // readerStage

    Options writerOptions;
    if (m_bCompress)
        writerOptions.add<bool>("compression", true);
    if (m_bForwardMetadata)
        writerOptions.add("forward_metadata", true);

    Stage& writer(makeWriter(m_outputFile, pclStage, ""));
    writer.addOptions(writerOptions);

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
