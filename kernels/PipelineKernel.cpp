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

#include "pdal/pdal_features.hpp"

#include "PipelineKernel.hpp"

#ifdef PDAL_HAVE_LIBXML2
#include <pdal/XMLSchema.hpp>
#endif

#include <pdal/PDALUtils.hpp>
#include <nlohmann/json.hpp>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "kernels.pipeline",
    "Pipeline Kernel",
    "http://pdal.io/apps/pipeline.html"
};

CREATE_STATIC_KERNEL(PipelineKernel, s_info)

std::string PipelineKernel::getName() const { return s_info.name; }

PipelineKernel::PipelineKernel() : m_validate(false), m_progressFd(-1)
{}


void PipelineKernel::validateSwitches(ProgramArgs& args)
{
    if (m_usestdin)
        m_inputFile = "STDIN";

    if (m_inputFile.empty())
        throw pdal_error("Input filename required.");

    if (m_stream && m_noStream)
        throw pdal_error("Can't execute with 'stream' and 'nostream' options");
    if (m_stream)
        m_mode = ExecMode::Stream;
    else if (m_noStream)
        m_mode = ExecMode::Standard;
    else
        m_mode = ExecMode::PreferStream;
}


bool PipelineKernel::isStagePrefix(const std::string& stage)
{
    return Kernel::isStagePrefix(stage) || stage == "stage";
}


void PipelineKernel::addSwitches(ProgramArgs& args)
{
    args.add("input,i", "Input filename", m_inputFile).setOptionalPositional();

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
    args.add("stdin,s", "Read pipeline from standard input", m_usestdin);
    args.add("stream", "Run in stream mode.  Error if not streamable.",
        m_stream);
    args.add("nostream", "Run in standard mode.", m_noStream);
    args.add("metadata", "Metadata filename", m_metadataFile);
    args.add("dims", "Dimensions to be stored", m_dimNames);
}


int PipelineKernel::execute()
{
    if (!Utils::fileExists(m_inputFile))
        throw pdal_error("file not found: " + m_inputFile);
    if (m_progressFile.size())
    {
        m_progressFd = Utils::openProgress(m_progressFile);
        m_manager.setProgressFd(m_progressFd);
    }

    if (m_validate)
    {
        NL::json root;
        // Validate the options of the pipeline we were
        // given, and once we succeed, we're done
        try
        {
            m_manager.readPipeline(m_inputFile);
            if (!m_manager.hasReader())
                throw pdal_error("Pipeline does not start with a reader.");
            m_manager.prepare();
            root["valid"] = true;
            root["error_detail"] = "";
            root["streamable"] = m_manager.pipelineStreamable();
        }
        catch (pdal::pdal_error const& e)
        {
            root["valid"] = false;
            root["error_detail"] = e.what();
            root["streamable"] = false;
        }
        Utils::closeProgress(m_progressFd);
        std::cout << root.dump(4) << "\n";
        return 0;
    }

    m_manager.readPipeline(m_inputFile);
    if (!m_manager.hasReader())
        throw pdal_error("Pipeline does not start with a reader.");
    m_manager.pointTable().layout()->setAllowedDims(m_dimNames);
    if (m_manager.execute(m_mode).m_mode == ExecMode::None)
        throw pdal_error("Couldn't run pipeline in requested execution mode.");

    if (m_metadataFile.size())
    {
        std::ostream *out = Utils::createFile(m_metadataFile, false);
        if (!out)
            throw pdal_error("Can't open file '" + m_metadataFile +
                "' for metadata output.");
        Utils::toJSON(m_manager.getMetadata(), *out);
        Utils::closeFile(out);
    }
    if (m_pipelineFile.size())
        PipelineWriter::writePipeline(m_manager.getStage(), m_pipelineFile);

    if (m_PointCloudSchemaOutput.size() > 0)
    {
#ifdef PDAL_HAVE_LIBXML2
        XMLSchema schema(m_manager.pointTable().layout());

        std::ostream *out = Utils::createFile(m_PointCloudSchemaOutput);
        std::string xml(schema.xml());
        out->write(xml.c_str(), xml.size());
        Utils::closeFile(out);
#else
        std::cerr << "libxml2 support not available, no schema is produced" <<
            std::endl;
#endif

    }
    Utils::closeProgress(m_progressFd);
    return 0;
}

} // pdal
