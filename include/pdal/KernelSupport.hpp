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

#pragma once

#include <string>

#include <pdal/Options.hpp>
#include <pdal/Stage.hpp>
#include <pdal/UserCallback.hpp>
#include <pdal/PipelineManager.hpp>

namespace pdal
{

class app_usage_error : public pdal::pdal_error
{
public:
    inline app_usage_error(std::string const& msg)
        : pdal_error(msg)
    {}
};


class app_runtime_error : public pdal::pdal_error
{
public:
    inline app_runtime_error(std::string const& msg)
        : pdal_error(msg)
    {}
};


// this is a static class with some helper functions the cmd line apps need
class PDAL_DLL KernelSupport
{
public:
    // makes a writer, from just the filename and some other
    // options (and the input stage)
    static PipelineManager* makePipeline(const std::string& filename);

private:
    KernelSupport& operator=(const KernelSupport&); // not implemented
    KernelSupport(const KernelSupport&); // not implemented
};


class PDAL_DLL PercentageCallback : public pdal::UserCallback
{
public:
    PercentageCallback(double major = 10.0, double minor = 2.0);
    virtual void callback();

protected:
    double m_lastMajorPerc;
    double m_lastMinorPerc;
    bool m_done;
};


class PDAL_DLL HeartbeatCallback : public pdal::UserCallback
{
public:
    virtual void callback()
        { std::cerr << "."; }
};


class PDAL_DLL ShellScriptCallback : public PercentageCallback
{
public:
    ShellScriptCallback(const std::vector<std::string>& command);
    virtual void callback();

private:
    std::string m_command;
};

} // namespace pdal
