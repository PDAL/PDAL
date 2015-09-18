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
#include <pdal/KernelSupport.hpp>
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

void TranslateKernel::validateSwitches()
{
    if (m_inputFile == "")
        throw app_usage_error("--input/-i required");

    if (m_outputFile == "")
        throw app_usage_error("--output/-o required");
}

void TranslateKernel::addSwitches()
{
    po::options_description* file_options =
        new po::options_description("file options");

    file_options->add_options()
    ("input,i", po::value<std::string>(&m_inputFile)->default_value(""),
     "input file name")
    ("output,o", po::value<std::string>(&m_outputFile)->default_value(""),
     "output file name")
    ("pipeline,p", po::value<std::string>(&m_pipelineOutput)->default_value(""),
     "pipeline output")
    ("reader,r", po::value<std::string>(&m_readerType)->default_value(""),
     "reader type")
    ("filter,f",
     po::value<std::vector<std::string> >(&m_filterType)->multitoken(),
     "filter type")
    ("writer,w", po::value<std::string>(&m_writerType)->default_value(""),
     "writer type")
    ;

    addSwitchSet(file_options);

    addPositionalSwitch("input", 1);
    addPositionalSwitch("output", 1);
    addPositionalSwitch("filter", -1);
}

int TranslateKernel::execute()
{
    // setting common options for each stage propagates the debug flag and
    // verbosity level
    Options readerOptions, filterOptions, writerOptions;
    setCommonOptions(readerOptions);
    setCommonOptions(filterOptions);
    setCommonOptions(writerOptions);

    m_manager = std::unique_ptr<PipelineManager>(new PipelineManager);

    if (!m_readerType.empty())
    {
        m_manager->addReader(m_readerType);
    }
    else
    {
        StageFactory factory;
        std::string driver = factory.inferReaderDriver(m_inputFile);

        if (driver.empty())
            throw app_runtime_error("Cannot determine input file type of " +
                                    m_inputFile);
        m_manager->addReader(driver);
    }

    if (m_manager == NULL)
        throw pdal_error("Error making pipeline\n");

    Stage* reader = m_manager->getStage();

    if (reader == NULL)
        throw pdal_error("Error getting reader\n");

    readerOptions.add("filename", m_inputFile);
    reader->setOptions(readerOptions);

    Stage* stage = reader;

    // add each filter provided on the command-line, updating the stage pointer
    for (auto const f : m_filterType)
    {
        std::string filter_name(f);

        if (!Utils::startsWith(f, "filters."))
            filter_name.insert(0, "filters.");

        Stage* filter = &(m_manager->addFilter(filter_name));

        if (filter == NULL)
            throw pdal_error("Error getting filter\n");

        filter->setOptions(filterOptions);
        filter->setInput(*stage);
        stage = filter;
    }

    if (!m_writerType.empty())
    {
        m_manager->addWriter(m_writerType);
    }
    else
    {
        StageFactory factory;
        std::string driver = factory.inferWriterDriver(m_outputFile);

        if (driver.empty())
            throw app_runtime_error("Cannot determine output file type of " +
                                    m_outputFile);
        Options options = factory.inferWriterOptionsChanges(m_outputFile);
        writerOptions += options;
        m_manager->addWriter(driver);
    }

    Stage* writer = m_manager->getStage();

    if (writer == NULL)
        throw pdal_error("Error getting writer\n");

    writerOptions.add("filename", m_outputFile);
    writer->setOptions(writerOptions);
    writer->setInput(*stage);

    // be sure to recurse through any extra stage options provided by the user
    applyExtraStageOptionsRecursive(writer);

    m_manager->execute();

    if (m_pipelineOutput.size() > 0)
    {
        PipelineWriter writer(*m_manager);
        writer.writePipeline(m_pipelineOutput);
    }

    return 0;
}

} // namespace pdal
