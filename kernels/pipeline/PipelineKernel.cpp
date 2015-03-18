/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include "PipelineKernel.hpp"

#include <boost/program_options.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "kernels.pipeline",
    "Pipeline Kernel",
    "http://pdal.io/kernels/kernels.pipeline.html" );

CREATE_STATIC_PLUGIN(1, 0, PipelineKernel, Kernel, s_info)

std::string PipelineKernel::getName() const { return s_info.name; }

PipelineKernel::PipelineKernel() : m_validate(false)
{}


void PipelineKernel::validateSwitches()
{
    if (m_usestdin)
        m_inputFile = "STDIN";

    if (m_inputFile.empty())
        throw app_usage_error("input file name required");
}


void PipelineKernel::addSwitches()
{
    po::options_description* file_options =
        new po::options_description("file options");

    file_options->add_options()
        ("input,i", po::value<std::string>(&m_inputFile)->default_value(""),
            "input file name")
        ("pipeline-serialization",
            po::value<std::string>(&m_pipelineFile)->default_value(""), "")
        ("validate",
            po::value<bool>(&m_validate)->zero_tokens()->implicit_value(true),
            "Validate the pipeline (including serialization), but do not "
            "execute writing of points")
        ;

    addSwitchSet(file_options);
    addPositionalSwitch("input", 1);
}

int PipelineKernel::execute()
{
    if (!FileUtils::fileExists(m_inputFile))
        throw app_runtime_error("file not found: " + m_inputFile);

    pdal::PipelineManager manager;

    pdal::PipelineReader reader(manager, isDebug(), getVerboseLevel());
    bool isWriter = reader.readPipeline(m_inputFile);
    if (!isWriter)
        throw app_runtime_error("Pipeline file does not contain a writer. "
            "Use 'pdal info' to read the data.");

    for (const auto& pi : getExtraStageOptions())
    {
        std::string name = pi.first;
        Options options = pi.second;
        std::vector<Stage *> stages = manager.getStage()->findStage(name);
        for (const auto& s : stages)
        {
            Options opts = s->getOptions();
            for (const auto& o : options.getOptions())
                opts.add(o);
            s->setOptions(opts);
        }
    }

    PointTable table;
    manager.getStage()->prepare(table);
    manager.getStage()->execute(table);
    if (m_pipelineFile.size() > 0)
    {
        pdal::PipelineWriter writer(manager);
        writer.writePipeline(m_pipelineFile);
    }
    return 0;
}

} // pdal
