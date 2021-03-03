/******************************************************************************
* Copyright (c) 2013, Howard Butler (hobu.inc@gmail.com)
* Copyright (c) 2014-2015, Bradley J Chambers (brad.chambers@gmail.com)
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

#include <pdal/Kernel.hpp>
#include <pdal/PluginManager.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/pdal_config.hpp>
#include <pdal/util/Backtrace.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>

#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#ifndef _WIN32
#include <csignal>
#include <unistd.h>
#endif

using namespace pdal;

std::string headline(Utils::screenWidth(), '-');

class App
{
public:
    App() : m_out(std::cout)
    {}

    int execute(StringList& cmdArgs, LogPtr& log);

private:
    void outputVersion();
    void outputHelp(const ProgramArgs& args);
    void outputDrivers();
    void outputCommands(const std::string& leader);
    void outputOptions();
    void outputOptions(const std::string& stageName,std::ostream& strm);
    void addArgs(ProgramArgs& args);

    std::ostream& m_out;

    std::string m_command;
    bool m_debug;
    LogLevel m_logLevel;
    bool m_showDrivers;
    bool m_help;
    bool m_showCommands;
    bool m_showVersion;
    std::string m_showOptions;
    bool m_showJSON;
    std::string m_log;
    bool m_logtiming;
};


void App::outputVersion()
{
    m_out << headline << std::endl;
    m_out << "pdal " << Config::fullVersionString() << std::endl;
    m_out << headline << std::endl;
    m_out << std::endl;
}


void App::outputHelp(const ProgramArgs& args)
{
    m_out << "Usage:" << std::endl;
    m_out << "  pdal <options>" << std::endl;
    m_out << "  pdal <command> <command options>" << std::endl;

    args.dump(m_out, 2, 80);
    m_out << std::endl;

    m_out << "The following commands are available:" << std::endl;

    // Load all kernels so that we can report the names.
    StageFactory f;
    PluginManager<Kernel>::loadAll();
    outputCommands("  - ");
    m_out << std::endl;
    m_out << "See http://pdal.io/apps/ for more detail" << std::endl;
}


void App::outputDrivers()
{
    // Force plugin loading.
    StageFactory f;
    PluginManager<Stage>::loadAll();
    StringList stages = PluginManager<Stage>::names();

    if (!m_showJSON)
    {
        int nameColLen(28);
        int descripColLen(Utils::screenWidth() - nameColLen - 1);

        std::string tablehead(std::string(nameColLen, '=') + ' ' +
            std::string(descripColLen, '='));

        m_out << std::endl;
        m_out << tablehead << std::endl;
        m_out << std::left << std::setw(nameColLen) << "Name" <<
            " Description" << std::endl;
        m_out << tablehead << std::endl;

        m_out << std::left;


        for (auto name : stages)
        {
            std::string descrip = PluginManager<Stage>::description(name);
            StringList lines = Utils::wordWrap(descrip, descripColLen - 1);
            for (size_t i = 0; i < lines.size(); ++i)
            {
                m_out << std::setw(nameColLen) << name << " " <<
                    lines[i] << std::endl;
                name.clear();
            }
        }

        m_out << tablehead << std::endl << std::endl;
    }
    else
    {
        NL::json j;
        StageExtensions& extensions = PluginManager<Stage>::extensions();
        for (auto name : stages)
        {
            Stage *s = f.createStage(name);
            std::string description = PluginManager<Stage>::description(name);
            std::string link = PluginManager<Stage>::link(name);
            j.push_back(
                { { "name", name },
                  { "description", description },
                  { "link", link },
                  { "extensions", extensions.extensions(name) },
                  { "streamable", s->pipelineStreamable() }
                }
            );
        }
        m_out << std::setw(4) << j;
    }
}


void App::outputCommands(const std::string& leader)
{
    StageFactory f;
    PluginManager<Kernel>::loadAll();
    std::string kernelbase("kernels.");
    for (auto name : PluginManager<Kernel>::names())
    {
        if (Utils::startsWith(name, kernelbase))
            name = name.substr(kernelbase.size());
        m_out << leader << name << std::endl;
    }
}


void App::outputOptions(std::string const& stageName, std::ostream& strm)
{
    // Force plugin loading.
    StageFactory f(false);

    Stage* s = f.createStage(stageName);
    if (!s)
    {
        std::cerr << "Unable to create stage " << stageName << "\n";
        return;
    }

    ProgramArgs args;
    s->addAllArgs(args);

    if (!m_showJSON)
    {
        strm  << stageName << " -- " << PluginManager<Stage>::link(stageName) <<
            std::endl;
        strm  << headline << std::endl;

        args.dump2(strm , 2, 6, headline.size());
    }
    else
    {
        std::ostringstream ostr;
        args.dump3(ostr);

        NL::json array;
        try
        {
            array = NL::json::parse(ostr.str());
        }
        catch (NL::json::parse_error&)
        {}

        NL::json object = { stageName, array };
        strm  << object;
    }
}


void App::outputOptions()
{
    // Force plugin loading.
    StageFactory f(false);

    StringList nv = PluginManager<Stage>::names();

    if (!m_showJSON)
    {
        for (auto const& n : nv)
        {
            outputOptions(n, m_out);
            m_out << std::endl;
        }
    }
    else
    {
        std::stringstream strm;
        NL::json options;
        for (auto const& n : nv)
        {
            outputOptions(n, strm);
            NL::json j;
            try
            {
                strm >> j;
            }
            catch (NL::json::parse_error&)
            {}
            options.push_back(j);
            strm.str("");
        }
        m_out << options;
    }
}


void App::addArgs(ProgramArgs& args)
{
    args.add("command", "The PDAL command", m_command).setPositional();
    args.add("debug", "Sets the output level to 3 (option deprecated)",
        m_debug);
    args.add("verbose,v", "Sets the output level (0-8)", m_logLevel,
        LogLevel::None);
    args.add("drivers", "List available drivers", m_showDrivers);
    args.add("help,h", "Display help text", m_help);
    args.add("list-commands", "List available commands", m_showCommands);
    args.add("version", "Show program version", m_showVersion);
    args.add("options", "Show options for specified driver (or 'all')",
        m_showOptions);
    args.add("log", "Log filename (accepts stderr, stdout, stdlog, devnull"
        " as special cases)", m_log, "stderr");
    args.add("logtiming", "Turn on timing for log messages", m_logtiming);
    Arg& json = args.add("showjson", "List options or drivers as JSON output",
        m_showJSON);
    json.setHidden();
}

namespace
{
    LogPtr logPtr(Log::makeLog("PDAL", "stderr"));
}

int main(int argc, char* argv[])
{
    App pdal;

    StringList cmdArgs;
    for (int i = 1; i < argc; ++i)
        cmdArgs.push_back(argv[i]);
    return pdal.execute(cmdArgs, logPtr);
}


int App::execute(StringList& cmdArgs, LogPtr& log)
{
    ProgramArgs args;

    addArgs(args);
    try
    {
        args.parseSimple(cmdArgs);
    }
    catch (arg_val_error const& e)
    {
        Utils::printError(e.what());
        return 1;
    }

    log = Log::makeLog("PDAL", m_log, m_logtiming);
    if (m_logLevel != LogLevel::None)
        log->setLevel(m_logLevel);
    else if (m_debug)
        log->setLevel(LogLevel::Debug);
    log->get(LogLevel::Debug) << "Debugging..." << std::endl;
    PluginManager<Stage>::setLog(log);
    PluginManager<Kernel>::setLog(log);
#ifndef _WIN32
    if (m_debug)
    {
        signal(SIGSEGV, [](int sig)
        {
            logPtr->get(LogLevel::Debug) << "Segmentation fault (signal 11)\n";
            StringList lines = Utils::backtrace();

            for (const auto& l : lines)
                logPtr->get(LogLevel::Debug) << l << std::endl;
            exit(1);
        });
    }
#endif

    m_command = Utils::tolower(m_command);
    if (!m_command.empty())
    {
        int ret = 0;
        std::string name("kernels." + m_command);

        Kernel *kernel = PluginManager<Kernel>::createObject(name);
        if (kernel)
        {
            if (m_help)
                cmdArgs.push_back("--help");
            // This shouldn't throw.  If it does, it's something awful, so
            // not cleaning up seems inconsequential.
            log->setLeader("pdal " + m_command);
            ret = kernel->run(cmdArgs, log);
            delete kernel;
            // IMPORTANT - The kernel must be destroyed before GDAL
            //  drivers are unregistered or GDAL will attempt to destroy
            //  resources more than once, resulting in a crash.
            gdal::unregisterDrivers();
        }
        else
        {
            log->get(LogLevel::Error) << "Command '" << m_command <<
                "' not recognized" << std::endl << std::endl;
            ret = 1;
        }
        return ret;
    }

    // If we get here, all arguments should be consumed, if not, it's
    // an error.
    if (cmdArgs.size())
    {
        Utils::printError("Unexpected argument '" + cmdArgs[0] + "'.");
        return 1;
    }

    if (m_showVersion)
        outputVersion();
    else if (m_showDrivers)
        outputDrivers();
    else if (m_showOptions.size())
    {
        if (m_showOptions == "all")
            outputOptions();
        else
            outputOptions(m_showOptions, m_out);
    }
    else
        outputHelp(args);
    return 0;
}
