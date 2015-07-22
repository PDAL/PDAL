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

#include <pdal/KernelSupport.hpp>
#include <pdal/pdal_export.hpp>

#ifdef PDAL_COMPILER_MSVC
#  pragma warning(push)
#  pragma warning(disable: 4512)  // assignment operator could not be generated
#endif
#include <boost/program_options.hpp>
#ifdef PDAL_COMPILER_MSVC
#  pragma warning(pop)
#endif

#include <cstdint>
#include <iosfwd>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace po = boost::program_options;

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
    Stage& makeReader(const std::string& inputFile);
    Stage& makeWriter(const std::string& outputFile, Stage& parent);

public:
    // implement this, with calls to addOptionSet()
    virtual void addSwitches() {}

    // implement this, to do sanity checking of cmd line
    // will throw if the user gave us bad options
    virtual void validateSwitches() {}

    // implement this, to do your actual work
    // it will be wrapped in a global catch try/block for you
    virtual int execute() = 0;

    void addSwitchSet(po::options_description* options);
    void addHiddenSwitchSet(po::options_description* options);
    void addPositionalSwitch(const char* name, int max_count);
    void setCommonOptions(Options &options);

    void setProgressShellCommand(std::vector<std::string> const& command)
    {
        m_heartbeat_shell_command = command;
    }
    std::vector<std::string> getProgressShellCommand()
    {
        return m_heartbeat_shell_command;
    }

    const Options& extraStageOptions(const std::string& stage)
    {
        static Options nullOpts;

        auto oi = m_extraStageOptions.find(stage);
        if (oi == m_extraStageOptions.end())
            return nullOpts;
        return oi->second;
    }

    void applyExtraStageOptionsRecursive(Stage *s)
    {
        s->addOptions(extraStageOptions(s->getName()));
        auto stages = s->getInputs();
        for (Stage *s : stages)
            applyExtraStageOptionsRecursive(s);
    }

protected:
    Stage& ownStage(Stage *s)
    {
        m_stages.push_back(std::unique_ptr<Stage>(s));
        return *s;
    }
    bool argumentExists(const std::string& name)
        { return (bool)m_variablesMap.count(name); }
    bool argumentSpecified(const std::string& name);

    bool m_usestdin;
    int m_argc;
    const char** m_argv;
    Log m_log;

private:
    int innerRun();
    void parseSwitches();
    void outputHelp();
    void outputVersion();
    void addBasicSwitchSet();
    void collectExtraOptions();

    int do_switches();
    int do_startup();
    int do_execution();
    int do_shutdown();

    bool m_isDebug;
    uint32_t m_verboseLevel;
    bool m_showHelp;
    std::string m_showOptions;
    bool m_showVersion;
    bool m_showTime;
    std::string m_appName;
    bool m_hardCoreDebug;
    std::vector<std::string> m_heartbeat_shell_command;
    bool m_reportDebug;
    std::string m_scales;
    std::string m_offsets;
    bool m_visualize;
    std::string m_label;

    std::vector<po::options_description*> m_public_options;
    std::vector<po::options_description*> m_hidden_options;
    po::positional_options_description m_positionalOptions;
    po::variables_map m_variablesMap;
    std::vector<std::string> m_extra_options;
    std::map<std::string, Options> m_extraStageOptions;
    std::vector<std::unique_ptr<Stage>> m_stages;

    Kernel& operator=(const Kernel&); // not implemented
    Kernel(const Kernel&); // not implemented
};

PDAL_DLL std::ostream& operator<<(std::ostream& ostr, const Kernel&);

} // namespace pdal

