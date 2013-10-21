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

#include <pdal/kernel/Support.hpp>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <pdal/FileUtils.hpp>
#include <pdal/Utils.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/PipelineReader.hpp>

namespace pdal { namespace kernel {
    
pdal::Stage* AppSupport::makeReader(pdal::Options& options)
{
    const std::string inputFile = options.getValueOrThrow<std::string>("filename");

    if (!pdal::FileUtils::fileExists(inputFile))
    {
        throw app_runtime_error("file not found: " + inputFile);
    }

    pdal::StageFactory factory;
    std::string driver = factory.inferReaderDriver(inputFile, options);
    if (driver == "")
    {
        throw app_runtime_error("Cannot determine input file type of " + inputFile);
    }

    pdal::Stage* stage = factory.createReader(driver, options);
    if (!stage)
    {
        throw app_runtime_error("reader creation failed");
    }

    return stage;
}


pdal::Writer* AppSupport::makeWriter(pdal::Options& options, pdal::Stage& stage)
{
    const std::string outputFile = options.getValueOrThrow<std::string>("filename");

    pdal::StageFactory factory;
    std::string driver = factory.inferWriterDriver(outputFile, options);
    if (driver == "")
    {
        throw app_runtime_error("Cannot determine output file type of " + outputFile);
    }
        
    pdal::Writer* writer = factory.createWriter(driver, stage, options);
    if (!writer)
    {
        throw app_runtime_error("writer creation failed");
    }

    return writer;
}


PercentageCallback::PercentageCallback(double major, double minor)
    : m_lastMajorPerc(-1 * major)
    , m_lastMinorPerc(-1 * minor)
    , m_done(false)
{
    return;
}


void PercentageCallback::callback()
{
    if (m_done) return;

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

    return;
}

ShellScriptCallback::ShellScriptCallback(std::vector<std::string> const& command)
{
    double major_tick(10.0);
    double minor_tick(2.0);
    
    if (!command.size())
    {
        m_command = "";        
    }
    else
    {
        m_command = command[0];

        if (command.size() == 3)
        {
            major_tick = boost::lexical_cast<double>(command[1]);
            minor_tick = boost::lexical_cast<double>(command[2]);
        } 
        else if (command.size() == 2)
        {
            major_tick = boost::lexical_cast<double>(command[1]);
        }
    }

    PercentageCallback(major_tick, minor_tick);

    return;
}


void ShellScriptCallback::callback()
{
    if (m_done) return;

    double currPerc = getPercentComplete();
    
    if (pdal::Utils::compare_distance<double>(currPerc, 100.0))
    {
        m_done = true;
    }
    else if (currPerc >= m_lastMajorPerc + 10.0)
    {
        std::string output;
        int stat = pdal::Utils::run_shell_command(m_command + " " + boost::lexical_cast<std::string>(static_cast<int>(currPerc)), output);
        m_lastMajorPerc = currPerc;
        m_lastMinorPerc = currPerc;
    }
    else if (currPerc >= m_lastMinorPerc + 2.0)
    {
        m_lastMinorPerc = currPerc;
    }

    return;
}

HeartbeatCallback::HeartbeatCallback()
{
    return;
}


void HeartbeatCallback::callback()
{
    std::cerr << '.';

    return;
}

}} // pdal::kernel