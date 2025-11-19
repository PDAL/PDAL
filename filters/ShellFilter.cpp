/******************************************************************************
* Copyright (c) 2019, Howard Butler, howard@hobu.co
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

#include "ShellFilter.hpp"

#include <pdal/util/Algorithm.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>



namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.shell",
    "Execute a shell operation inline with PDAL pipeline steps",
    "http://pdal.org/stages/filters.shell.html"
};

CREATE_STATIC_STAGE(ShellFilter, s_info)

std::string ShellFilter::getName() const { return s_info.name; }

void ShellFilter::addArgs(ProgramArgs& args)
{
    args.add("command", "Command to run", m_command).setPositional();
}

void ShellFilter::initialize()
{
	std::string allowed;
    int set = Utils::getenv("PDAL_ALLOW_SHELL", allowed);
	if (set == -1)
		throw pdal::pdal_error("PDAL_ALLOW_SHELL environment variable not set, shell access is not allowed");
}





PointViewSet ShellFilter::run(PointViewPtr view)
{
    log()->get(LogLevel::Debug) << "running command : '" << m_command << "'"<< std::endl;

    int status = Utils::run_shell_command(m_command.c_str(), m_command_output);
    if (status)
    {
        std::stringstream msg;
        msg << "Command '" << m_command << "' failed to execute";
        msg << " with output '" << m_command_output <<"'";
        throw pdal::pdal_error(msg.str());

    }

    log()->get(LogLevel::Debug) << "command output: '" << m_command_output << "'"<< std::endl;
    log()->get(LogLevel::Debug) << "status: '" << status<< "'"<< std::endl;

    PointViewSet views;
    views.insert(view);
    return views;
}



void ShellFilter::done(PointTableRef table)
{
    bool isJson = (m_command_output.find("{") != m_command_output.npos) ||
                  (m_command_output.find("}") != m_command_output.npos);

    if (isJson)
        m_metadata.addWithType("output",
                                m_command_output,
                                "json",
                                "Command output");
    else
        m_metadata.add("output", m_command_output, "Command output");

}





} // namespace pdal

