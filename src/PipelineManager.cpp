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

#include <pdal/PipelineManager.hpp>

#include <pdal/util/FileUtils.hpp>

#include "PipelineReaderXML.hpp"
#include "PipelineReaderJSON.hpp"

namespace pdal
{

PipelineManager::~PipelineManager()
{
    FileUtils::closeFile(m_input);
}


void PipelineManager::readPipeline(std::istream& input)
{
    // Read stream into string.
    std::string s(std::istreambuf_iterator<char>(input), {});

    std::istringstream ss(s);
    if (s.find("?xml") != std::string::npos)
        PipelineReaderXML(*this).readPipeline(ss);
    else if (s.find("\"pipeline\"") != std::string::npos)
        PipelineReaderJSON(*this).readPipeline(ss);
    else
    {
        try
        {
            PipelineReaderXML(*this).readPipeline(ss);
        }
        catch (pdal_error)
        {
            // Rewind to make sure the stream is properly positioned after
            // attempting an XML pipeline.
            ss.seekg(0);
            PipelineReaderJSON(*this).readPipeline(ss);
        }
    }
}


void PipelineManager::readPipeline(const std::string& filename)
{
    if (FileUtils::extension(filename) == ".xml")
    {
        PipelineReaderXML pipeReader(*this);
        return pipeReader.readPipeline(filename);
    }
    else if (FileUtils::extension(filename) == ".json")
    {
        PipelineReaderJSON pipeReader(*this);
        return pipeReader.readPipeline(filename);
    }
    else
    {
        FileUtils::closeFile(m_input);
        m_input = FileUtils::openFile(filename);
        readPipeline(*m_input);
    }
}


Stage& PipelineManager::addReader(const std::string& type)
{
    Stage *reader = m_factory.createStage(type);
    if (!reader)
    {
        std::ostringstream ss;
        ss << "Couldn't create reader stage of type '" << type << "'.";
        throw pdal_error(ss.str());
    }
    reader->setProgressFd(m_progressFd);
    m_stages.push_back(reader);
    return *reader;
}


Stage& PipelineManager::addFilter(const std::string& type)
{
    Stage *filter = m_factory.createStage(type);
    if (!filter)
    {
        std::ostringstream ss;
        ss << "Couldn't create filter stage of type '" << type << "'.";
        throw pdal_error(ss.str());
    }
    filter->setProgressFd(m_progressFd);
    m_stages.push_back(filter);
    return *filter;
}


Stage& PipelineManager::addWriter(const std::string& type)
{
    Stage *writer = m_factory.createStage(type);
    if (!writer)
    {
        std::ostringstream ss;
        ss << "Couldn't create writer stage of type '" << type << "'.";
        throw pdal_error(ss.str());
    }
    writer->setProgressFd(m_progressFd);
    m_stages.push_back(writer);
    return *writer;
}


void PipelineManager::validateStageOptions() const
{
    // Make sure that the options specified are for relevant stages.
    for (auto& si : m_stageOptions)
    {
        const std::string& stageName = si.first;
        auto it = std::find_if(m_stages.begin(), m_stages.end(),
            [stageName](Stage *s)
            { return (s->getName() == stageName); });

        // If the option stage name matches no created stage, then error.
        if (it == m_stages.end())
        {
            std::ostringstream oss;
            oss << "Argument references invalid/unused stage: '" <<
                stageName << "'.";
            throw pdal_error(oss.str());
        }
    }
}


QuickInfo PipelineManager::preview() const
{
    QuickInfo qi;

    validateStageOptions();
    Stage *s = getStage();
    if (s)
       qi = s->preview();
    return qi;
}


void PipelineManager::prepare() const
{
    validateStageOptions();
    Stage *s = getStage();
    if (s)
       s->prepare(m_table);
}


point_count_t PipelineManager::execute()
{
    prepare();

    Stage *s = getStage();
    if (!s)
        return 0;
    m_viewSet = s->execute(m_table);
    point_count_t cnt = 0;
    for (auto pi = m_viewSet.begin(); pi != m_viewSet.end(); ++pi)
    {
        PointViewPtr view = *pi;
        cnt += view->size();
    }
    return cnt;
}


MetadataNode PipelineManager::getMetadata() const
{
    MetadataNode output("stages");

    for (auto s : m_stages)
    {
        output.add(s->getMetadata());
    }
    return output;
}


Stage& PipelineManager::makeReader(const std::string& inputFile,
    std::string driver)
{
    if (!inputFile.empty() && !FileUtils::fileExists(inputFile))
        throw pdal_error("file not found: " + inputFile);

    if (driver.empty())
    {
        driver = StageFactory::inferReaderDriver(inputFile);
        if (driver.empty())
            throw pdal_error("Cannot determine input file type of " +
                inputFile);
    }
    Options options;
    if (!inputFile.empty())
        options.add("filename", inputFile);

    Stage& reader = addReader(driver);
    setOptions(reader, options);
    return reader;
}


Stage& PipelineManager::makeFilter(const std::string& driver)
{
    Stage& filter = addFilter(driver);
    setOptions(filter, Options());
    return filter;
}


Stage& PipelineManager::makeFilter(const std::string& driver, Stage& parent)
{
    Stage& filter = makeFilter(driver);
    filter.setInput(parent);
    return filter;
}


Stage& PipelineManager::makeWriter(const std::string& outputFile,
    std::string driver)
{
    if (driver.empty())
    {
        driver = StageFactory::inferWriterDriver(outputFile);
        if (driver.empty())
            throw pdal_error("Cannot determine output file type of " +
                outputFile);
    }

    Options options;
    if (!outputFile.empty())
        options.add("filename", outputFile);

    auto& writer = addWriter(driver);
    setOptions(writer, options);
    return writer;
}


Stage& PipelineManager::makeWriter(const std::string& outputFile,
    std::string driver, Stage& parent)
{
    Stage& writer = makeWriter(outputFile, driver);
    writer.setInput(parent);
    return writer;
}


void PipelineManager::setOptions(Stage& stage, const Options& addOps)
{
    // First apply common options.
    stage.setOptions(m_commonOptions);

    // Apply additional reader/writer options, making sure they replace any
    // common options.
    stage.removeOptions(addOps);
    stage.addOptions(addOps);

    // Apply options provided on the command line, overriding others.
    Options& ops = stageOptions(stage);
    stage.removeOptions(ops);
    stage.addOptions(ops);
}


Options& PipelineManager::stageOptions(Stage& stage)
{
    static Options nullOpts;

    auto oi = m_stageOptions.find(stage.getName());
    if (oi == m_stageOptions.end())
        return nullOpts;
    return oi->second;
}

} // namespace pdal
