/******************************************************************************
* Copyright (c) 2014, Bradley J Chambers (brad.chambers@gmail.com)
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

#include <cstdint>
#include <iosfwd>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <pdal/PipelineManager.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

class Options;
class PointView;

typedef std::shared_ptr<PointView> PointViewPtr;

//
// The application base class gives us these common options:
//    --help / -h
//    --verbose / -v
//    --version
//
class PDAL_DLL Kernel
{
    FRIEND_TEST(KernelTest, parseOption);

public:
    virtual ~Kernel()
    {}

    // call this, to start the machine
    int run(int argc, const char* argv[], const std::string& appName);

    bool isDebug() const;
    uint32_t getVerboseLevel() const;
    virtual std::string getName() const = 0;
    bool isVisualize() const;
    void visualize(PointViewPtr view);

protected:
    // this is protected; your derived class ctor will be the public entry point
    Kernel();
    Stage& makeReader(const std::string& inputFile, std::string driver);
    Stage& makeFilter(const std::string& driver, Stage& parent);
    Stage& makeFilter(const std::string& driver);
    Stage& makeWriter(const std::string& outputFile, Stage& parent,
        std::string driver);

public:
    virtual void addSwitches(ProgramArgs& args)
    {}

    // implement this, to do sanity checking of cmd line
    // will throw if the user gave us bad options
    virtual void validateSwitches(ProgramArgs& args)
    {}

    // implement this, to do your actual work
    // it will be wrapped in a global catch try/block for you
    virtual int execute() = 0;

protected:
    bool m_usestdin;
    Log m_log;
    PipelineManager m_manager;
    std::string m_driverOverride;

private:
    int innerRun(ProgramArgs& args);
    void outputHelp(ProgramArgs& args);
    void outputVersion();
    void addBasicSwitches(ProgramArgs& args);
    void parseCommonOptions();

    void doSwitches(int argc, const char *argv[], ProgramArgs& args);
    int doStartup();
    int doExecution(ProgramArgs& args);

    static bool test_parseOption(std::string o, std::string& stage,
        std::string& option, std::string& value);

    bool m_isDebug;
    uint32_t m_verboseLevel;
    bool m_showHelp;
    bool m_showOptions;
    bool m_showVersion;
    bool m_showTime;
    std::string m_appName;
    bool m_hardCoreDebug;
    bool m_reportDebug;
    std::string m_scales;
    std::string m_offsets;
    bool m_visualize;
    std::string m_label;

    Kernel& operator=(const Kernel&); // not implemented
    Kernel(const Kernel&); // not implemented
};

PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const Kernel&);

} // namespace pdal
