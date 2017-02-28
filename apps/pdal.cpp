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

#include <pdal/GDALUtils.hpp>
#include <pdal/KernelFactory.hpp>
#include <pdal/PluginManager.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/pdal_config.hpp>

#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <json/json.h>

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
    void outputCommands();
    void outputOptions();
    void outputOptions(const std::string& stageName,std::ostream& strm);
    void addArgs(ProgramArgs& args);
    std::string findKernel();

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
};


void App::outputVersion()
{
    m_out << headline << std::endl;
    m_out << "pdal " << GetFullVersionString() << std::endl;
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

    KernelFactory f(false);
    StringList loaded_kernels = PluginManager::names(PF_PluginType_Kernel);

    for (auto name : loaded_kernels)
        m_out << "   - " << name << std::endl;
    m_out << "See http://pdal.io/apps/ for more detail" << std::endl;
}


void App::outputDrivers()
{
    // Force plugin loading.
    StageFactory f(false);

    StringList stages = PluginManager::names(PF_PluginType_Filter |
        PF_PluginType_Reader | PF_PluginType_Writer);

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
            std::string descrip = PluginManager::description(name);
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

        Json::Value array(Json::arrayValue);
        for (auto name : stages)
        {
            std::string description = PluginManager::description(name);
            std::string link = PluginManager::link(name);
            Json::Value node(Json::objectValue);
            node["name"] = name;
            node["description"] = description;
            node["link"] = link;
            array.append(node);
        }

        m_out << array;

    }
}


void App::outputCommands()
{
    KernelFactory f(false);
    std::vector<std::string> loaded_kernels;
    loaded_kernels = PluginManager::names(PF_PluginType_Kernel);
    for (auto name : loaded_kernels)
        m_out << name << std::endl;
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
        strm  << stageName << " -- " << PluginManager::link(stageName) <<
            std::endl;
        strm  << headline << std::endl;

        args.dump2(strm , 2, 6, headline.size());
    }
    else
    {
        std::ostringstream ostr;
        args.dump3(ostr);
        std::string json = ostr.str();

        Json::Reader jsonReader;
        Json::Value array;
        Json::Value object(Json::objectValue);
        jsonReader.parse(json, array);

        object[stageName] = array;

        strm  << object;

    }
}


void App::outputOptions()
{
    // Force plugin loading.
    StageFactory f(false);

    StringList nv = PluginManager::names(PF_PluginType_Filter |
        PF_PluginType_Reader | PF_PluginType_Writer);

    if (!m_showJSON)
    {
        for (auto const& n : nv)
        {
            outputOptions(n, m_out);
            m_out << std::endl;
        }
    } else
    {
        std::ostringstream strm;
        Json::Value options (Json::arrayValue);
        for (auto const& n : nv)
        {
            outputOptions(n, strm);
            std::string json(strm.str());
            Json::Reader jsonReader;
            Json::Value array;
            jsonReader.parse(json, array);
            options.append(array);

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
    Arg& json = args.add("showjson", "List options or drivers as JSON output",
        m_showJSON);
    json.setHidden();
}

namespace
{
    LogPtr logPtr(new Log("PDAL", "stderr"));
}

int main(int argc, char* argv[])
{
    App pdal;

    StringList cmdArgs;
    for (int i = 1; i < argc; ++i)
        cmdArgs.push_back(argv[i]);
    return pdal.execute(cmdArgs, logPtr);
}


std::string App::findKernel()
{
    StringList loadedKernels;

    auto kernelSurname = [](const std::string& name)
    {
        StringList names = Utils::split2(name, '.');
        return names.size() == 2 ? names[1] : std::string();
    };

    KernelFactory f(true);
    // Discover available kernels without plugins, and test to see if
    // the positional option 'command' is a valid kernel
    loadedKernels = PluginManager::names(PF_PluginType_Kernel);
    for (auto& name : loadedKernels)
        if (m_command == kernelSurname(name))
            return name;

    // Force loading of plugins.
    KernelFactory f2(false);
    loadedKernels = PluginManager::names(PF_PluginType_Kernel);
    for (auto& name : loadedKernels)
        if (m_command == kernelSurname(name))
            return name;

    return std::string();
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
        return -1;
    }

    log.reset(new Log("PDAL", m_log));
    if (m_logLevel != LogLevel::None)
        log->setLevel(m_logLevel);
    else if (m_debug)
        log->setLevel(LogLevel::Debug);
    PluginManager::setLog(log);
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
        std::string name(findKernel());
        if (name.size())
        {
            if (m_help)
                cmdArgs.push_back("--help");
            void *obj = PluginManager::createObject(name);
            Kernel *kernel(static_cast<Kernel *>(obj));
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
            log->get(LogLevel::Error) << "Command '" << m_command <<
                "' not recognized" << std::endl << std::endl;
        return ret;
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

