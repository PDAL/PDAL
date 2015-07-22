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

#include <pdal/KernelSupport.hpp>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <pdal/util/FileUtils.hpp>
#include <pdal/PipelineReader.hpp>
#include <pdal/util/Utils.hpp>

namespace pdal
{

PipelineManager* KernelSupport::makePipeline(const std::string& inputFile)
{
    if (!pdal::FileUtils::fileExists(inputFile))
        throw app_runtime_error("file not found: " + inputFile);

    PipelineManager* output = new PipelineManager;

    if (inputFile == "STDIN")
    {
        PipelineReader pipeReader(*output);
        pipeReader.readPipeline(std::cin);
    }
    else if (boost::filesystem::extension(inputFile) == ".xml")
    {
        PipelineReader pipeReader(*output);
        pipeReader.readPipeline(inputFile);
    }
    else
    {
        StageFactory factory;
        std::string driver = factory.inferReaderDriver(inputFile);

        if (driver.empty())
            throw app_runtime_error("Cannot determine input file type of " +
                inputFile);
        output->addReader(driver);
    }
    return output;
}


PercentageCallback::PercentageCallback(double major, double minor)
    : m_lastMajorPerc(-1 * major)
    , m_lastMinorPerc(-1 * minor)
    , m_done(false)
{}


void PercentageCallback::callback()
{
    if (m_done)
        return;

    double currPerc = getPercentComplete();

    if (pdal::Utils::compare_distance<double>(currPerc, 100.0))
    {
        std::cerr << ".100" << std::endl;
        m_done = true;
    }
    else if (currPerc >= m_lastMajorPerc + 10.0)
    {
        std::cerr << (int)currPerc << std::flush;
        m_lastMajorPerc = currPerc;
        m_lastMinorPerc = currPerc;
    }
    else if (currPerc >= m_lastMinorPerc + 2.0)
    {
        std::cerr << '.' << std::flush;
        m_lastMinorPerc = currPerc;
    }
}

ShellScriptCallback::ShellScriptCallback(
    const std::vector<std::string>& command)
{
    double major_tick(10.0);
    double minor_tick(2.0);

    if (command.size())
    {
        m_command = command[0];
        if (command.size() == 3)
        {
            major_tick = boost::lexical_cast<double>(command[1]);
            minor_tick = boost::lexical_cast<double>(command[2]);
        }
        else if (command.size() == 2)
            major_tick = boost::lexical_cast<double>(command[1]);
    }
    PercentageCallback(major_tick, minor_tick);
}


void ShellScriptCallback::callback()
{
    if (m_done)
        return;

    double currPerc = getPercentComplete();
    if (Utils::compare_distance<double>(currPerc, 100.0))
        m_done = true;
    else if (currPerc >= m_lastMajorPerc + 10.0)
    {
        std::string output;
        Utils::run_shell_command(m_command + " " +
            boost::lexical_cast<std::string>(static_cast<int>(currPerc)),
            output);
        m_lastMajorPerc = currPerc;
        m_lastMinorPerc = currPerc;
    }
    else if (currPerc >= m_lastMinorPerc + 2.0)
        m_lastMinorPerc = currPerc;
}

} // namespace pdal
