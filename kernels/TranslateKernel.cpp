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
#include <pdal/pdal_macros.hpp>
#include <pdal/PipelineWriter.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Stage.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/PipelineReaderJSON.hpp>
#include <pdal/util/FileUtils.hpp>
#include <json/json.h>

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
               "http://pdal.io/apps/translate.html");

CREATE_STATIC_PLUGIN(1, 0, TranslateKernel, Kernel, s_info)

std::string TranslateKernel::getName() const
{
    return s_info.name;
}

TranslateKernel::TranslateKernel()
{}

void TranslateKernel::addSwitches(ProgramArgs& args)
{
    args.add("input,i", "Input filename", m_inputFile).
        setPositional();
    args.add("output,o", "Output filename", m_outputFile).
        setPositional();
    args.add("filter,f", "Filter type", m_filterType).
        setOptionalPositional();
    args.add("json", "JSON array of filters", m_filterJSON);
    args.add("pipeline,p", "Pipeline output", m_pipelineOutput);
    args.add("metadata,m", "Dump metadata output to the specified file",
        m_metadataFile);
    args.add("reader,r", "Reader type", m_readerType);
    args.add("writer,w", "Writer type", m_writerType);
}

/*
  Build a pipeline from a JSON filter specification.
*/
void TranslateKernel::makeJSONPipeline()
{
    std::string json;

    if (pdal::FileUtils::fileExists(m_filterJSON))
        json = pdal::FileUtils::readFileIntoString(m_filterJSON);

    if (json.empty())
        json = m_filterJSON;

    Json::Reader jsonReader;
    Json::Value filters;
    jsonReader.parse(json, filters);
    if (filters.type() != Json::arrayValue || filters.empty())
        throw pdal_error("JSON must be an array of filter specifications");

    Json::Value pipeline(Json::arrayValue);

    // Add the input file, the filters (as provided) and the output file.
    if (m_readerType.size())
    {
        Json::Value node(Json::objectValue);
        node["filename"] = m_inputFile;
        node["type"] = m_readerType;
        pipeline.append(node);
    }
    else
        pipeline.append(Json::Value(m_inputFile));
    for (Json::ArrayIndex i = 0; i < filters.size(); ++i)
        pipeline.append(filters[i]);
    if (m_writerType.size())
    {
        Json::Value node(Json::objectValue);
        node["filename"] = m_outputFile;
        node["type"] = m_writerType;
        pipeline.append(node);
    }
    else
        pipeline.append(Json::Value(m_outputFile));

    Json::Value root;
    root["pipeline"] = pipeline;

    std::stringstream pipeline_str;
    pipeline_str << root;
    m_manager.readPipeline(pipeline_str);
}


/*
  Build a pipeline from filters specified as command-line arguments.
*/
void TranslateKernel::makeArgPipeline()
{
    Stage& reader = m_manager.makeReader(m_inputFile, m_readerType);
    Stage* stage = &reader;

    // add each filter provided on the command-line,
    // updating the stage pointer
    for (auto const f : m_filterType)
    {
        std::string filter_name(f);

        if (!Utils::startsWith(f, "filters."))
            filter_name.insert(0, "filters.");

        Stage& filter = m_manager.makeFilter(filter_name, *stage);
        stage = &filter;
    }
    m_manager.makeWriter(m_outputFile, m_writerType, *stage);
}


int TranslateKernel::execute()
{
    std::ostream *metaOut(nullptr);

    if (m_metadataFile.size())
    {
        metaOut = FileUtils::createFile(m_metadataFile);
        if (! metaOut)
            throw pdal_error("Couldn't output metadata output file '" +
                m_metadataFile + "'.");
    }

    if (m_filterJSON.size() && m_filterType.size())
        throw pdal_error("Cannot set both --filter options and --json options");

    if (!m_filterJSON.empty())
        makeJSONPipeline();
    else
        makeArgPipeline();

    if (m_pipelineOutput.size() > 0)
        PipelineWriter::writePipeline(m_manager.getStage(), m_pipelineOutput);
    m_manager.execute();
    if (metaOut)
    {
        MetadataNode m = m_manager.getMetadata();
        *metaOut << Utils::toJSON(m);
        FileUtils::closeFile(metaOut);
    }

    return 0;
}

} // namespace pdal
