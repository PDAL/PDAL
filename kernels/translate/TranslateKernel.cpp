/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
* Copyright (c) 2015, Bradley J Chambers (brad.chambers@gmail.com)
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

#include "TranslateKernel.hpp"

#include <pdal/KernelFactory.hpp>
#include <pdal/Options.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/PipelineWriter.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Stage.hpp>
#include <pdal/StageFactory.hpp>

#include <memory>
#include <string>
#include <vector>

namespace pdal
{

static PluginInfo const s_info =
    PluginInfo("kernels.translate",
               "The Translate kernel allows users to construct a pipeline " \
               "consisting of a reader, a writer, and N filter stages. " \
               "Any supported stage type can be specified from the command " \
               "line, reducing the need to create custom kernels for every " \
               "combination.",
               "http://pdal.io/kernels/kernels.translate.html");

CREATE_STATIC_PLUGIN(1, 0, TranslateKernel, Kernel, s_info)

std::string TranslateKernel::getName() const
{
    return s_info.name;
}

TranslateKernel::TranslateKernel()
    : Kernel()
    , m_inputFile("")
    , m_outputFile("")
    , m_pipelineOutput("")
    , m_readerType("")
    , m_writerType("")
{}

void TranslateKernel::addSwitches(ProgramArgs& args)
{
    args.add("input,i", "Input filename", m_inputFile).setPositional();
    args.add("output,o", "Output filename", m_outputFile).setPositional();
    args.add("filter,f", "Filter type", m_filterType).setOptionalPositional();
    args.add("pipeline,p", "Pipeline output", m_pipelineOutput);
    args.add("reader,r", "Reader type", m_readerType);
    args.add("writer,w", "Writer type", m_writerType);
}

int TranslateKernel::execute()
{
    Options readerOptions, filterOptions, writerOptions;

    Stage& reader = m_manager.makeReader(m_inputFile, m_readerType);
    Stage* stage = &reader;

    // add each filter provided on the command-line, updating the stage pointer
    for (auto const f : m_filterType)
    {
        std::string filter_name(f);

        if (!Utils::startsWith(f, "filters."))
            filter_name.insert(0, "filters.");

        Stage& filter = m_manager.makeFilter(filter_name, *stage);
        stage = &filter;
    }

    Stage& writer = m_manager.makeWriter(m_outputFile, m_writerType, *stage);
    m_manager.execute();
    if (m_pipelineOutput.size() > 0)
        PipelineWriter::writePipeline(&writer, m_pipelineOutput);

    return 0;
}

} // namespace pdal
