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

#ifdef PDAL_HAVE_LIBXML2
#include <pdal/XMLSchema.hpp>
#endif

#include <pdal/PipelineReaderJSON.hpp>
#include <pdal/PipelineReaderXML.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/util/FileUtils.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "kernels.pipeline",
    "Pipeline Kernel",
    "http://pdal.io/kernels/kernels.pipeline.html" );

CREATE_STATIC_PLUGIN(1, 0, PipelineKernel, Kernel, s_info)

std::string PipelineKernel::getName() const { return s_info.name; }

PipelineKernel::PipelineKernel() : m_validate(false), m_progressFd(-1)
{}


void PipelineKernel::validateSwitches(ProgramArgs& args)
{
    if (m_usestdin)
        m_inputFile = "STDIN";

    if (m_inputFile.empty())
        throw pdal_error("Input filename required.");
}


void PipelineKernel::addSwitches(ProgramArgs& args)
{
    args.add("input,i", "input file name", m_inputFile).setOptionalPositional();
    args.add("pipeline-serialization", "Output file for pipeline serialization",
        m_pipelineFile);
    args.add("validate", "Validate the pipeline (including serialization), "
        "but do not write points", m_validate);
    args.add("progress",
        "Name of file or FIFO to which stages should write progress "
        "information.  The file/FIFO must exist.  PDAL will not create "
        "the progress file.",
        m_progressFile);
    args.add("pointcloudschema", "dump PointCloudSchema XML output",
        m_PointCloudSchemaOutput).setHidden();
}

int PipelineKernel::execute()
{
    if (!FileUtils::fileExists(m_inputFile))
        throw app_runtime_error("file not found: " + m_inputFile);
    if (m_progressFile.size())
        m_progressFd = Utils::openProgress(m_progressFile);

    pdal::PipelineManager manager(m_progressFd);

    // TODO(chambbj): we want to get back to something like this, as implemented
    // in https://github.com/PDAL/PDAL/commit/7668b140c17937fb3649e27ad181683429478cae
    // bool isWriter = manager.readPipeline(m_inputFile);
    bool isWriter = false;
    if (FileUtils::extension(m_inputFile) == ".xml")
    {
        PipelineReaderXML pipeReader(manager, isDebug(), getVerboseLevel());
        isWriter = pipeReader.readPipeline(m_inputFile);
    }
    else if (FileUtils::extension(m_inputFile) == ".json")
    {
        PipelineReaderJSON pipeReader(manager, isDebug(), getVerboseLevel());
        isWriter = pipeReader.readPipeline(m_inputFile);
    }

    if (!isWriter)
        throw app_runtime_error("Pipeline file does not contain a writer. "
            "Use 'pdal info' to read the data.");

    applyExtraStageOptionsRecursive(manager.getStage());
    manager.execute();

    if (m_pipelineFile.size() > 0)
        PipelineWriter::writePipeline(manager.getStage(), m_pipelineFile);

    if (m_PointCloudSchemaOutput.size() > 0)
    {
#ifdef PDAL_HAVE_LIBXML2
        XMLSchema schema(manager.pointTable().layout());

        std::ostream *out = FileUtils::createFile(m_PointCloudSchemaOutput);
        std::string xml(schema.xml());
        out->write(xml.c_str(), xml.size());
        FileUtils::closeFile(out);
#else
        std::cerr << "libxml2 support not available, no schema is produced" <<
            std::endl;
#endif

    }
    Utils::closeProgress(m_progressFd);
    return 0;
}

} // pdal
