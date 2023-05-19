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

#include <cctype>
#include <iostream>

#include <pdal/Kernel.hpp>
#include <pdal/Options.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/pdal_config.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/Algorithm.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <pdal/pdal_config.hpp>

#include <memory>
#include <vector>

namespace pdal
{

Kernel::Kernel() : m_showTime(false), m_hardCoreDebug(false)
{}


// Overridden in PipelineKernel to accept "stage" as well.
bool Kernel::isStagePrefix(const std::string& stageType)
{
   return (stageType == "readers" || stageType == "writers" ||
        stageType == "filters");
}


Kernel::ParseStageResult
Kernel::parseStageOption(std::string o, std::string& stage, std::string& option,
    std::string& value)
{
    value.clear();
    if ((o.size() < 2) || (o[0] != '-') || (o[1] != '-'))
        return ParseStageResult::Unknown;

    o = o.substr(2);

    // Options are stage_type.stage_name.option_name
    // stage_type is always lowercase stage_names start with lowercase and
    // then are lowercase or digits.  Option names start with lowercase and
    // then contain lowercase, digits or underscore.

    // This awfulness is to work around the multiply-defined islower.  Seems
    // a bit better than the cast solution.
    auto islc = [](char c)
        { return std::islower(c); };

    std::string::size_type pos = 0;
    std::string::size_type count = 0;

    // Get stage_type.
    count = Utils::extract(o, pos, islc);
    pos += count;
    std::string stageType = o.substr(0, pos);
    if (!isStagePrefix(stageType) || pos >= o.length() || o[pos++] != '.')
        return ParseStageResult::Unknown;

    // Get stage_name.
    bool ok;
    if (stageType == "stage")
        ok = Stage::parseTagName(o, pos);
    else
        ok = Stage::parseName(o, pos);
    if (!ok)
        return ParseStageResult::Unknown;

    stage = o.substr(0, pos);
    if (pos >= o.length() || o[pos++] != '.')
        return ParseStageResult::Unknown;

    // Get option name.
    std::string::size_type optionStart = pos;
    count = Option::parse(o, pos);
    pos += count;
    option = o.substr(optionStart, count);

    // We've gotten a good option name, so return true, even if the value
    // is missing.  The caller can handle the missing value if desired.
    if (pos >= o.length())
        return ParseStageResult::Ok;

    if (o[pos++] == '=')
    {
        value = o.substr(pos);
        if (value.size())
            return ParseStageResult::Ok;
    }
    return ParseStageResult::Invalid;
}


std::ostream& operator<<(std::ostream& ostr, const Kernel& kernel)
{
    ostr << "  Name: " << kernel.getName() << std::endl;
    return ostr;
}


StringList Kernel::extractStageOptions(const StringList& cmdArgs)
{
    StringList stringArgs;
    OptionsMap& stageOptions = m_manager.stageOptions();

    // Scan the argument vector for extra stage options.  Pull them out and
    // stick them in the list.  Let the ProgramArgs handle everything else.
    // NOTE: This depends on the format being "option=value" rather than
    //   "option value".  This is what we've always expected, so no problem,
    //   but it would be better to be more flexible.
    for (auto it = cmdArgs.begin(); it != cmdArgs.end(); ++it)
    {
        const std::string& cmd = *it;

        std::string stageName, opName, value;
        auto res = parseStageOption(cmd, stageName, opName, value);
        if (res == ParseStageResult::Unknown)
            stringArgs.push_back(cmd);
        else if (res == ParseStageResult::Invalid)
            throw pdal_error("Stage option '" + cmd + "' not valid.");
        else  // ParseStageResult::Ok
        {
            if (value.empty())
            {
                if (++it == cmdArgs.end())
                    throw pdal_error("Stage option '" + stageName + "." +
                        opName + "' has no value.");
                value = *it;
            }
            Option op(opName, value);
            stageOptions[stageName].add(op);
        }
    }
    return stringArgs;
}

bool Kernel::doSwitches(const StringList& cmdArgs, ProgramArgs& args)
{
    StringList stringArgs = extractStageOptions(cmdArgs);

    try
    {
        bool help;

        // parseSimple allows us to scan for the help option without
        // raising exception about missing arguments and so on.
        // It also removes consumed args from the arg list, so for now,
        // parse a copy that will be ignored by parse().
        ProgramArgs hargs;
        hargs.add("help,h", "Print help message", help);
        hargs.parseSimple(stringArgs);
        if (help)
            return false;

        addBasicSwitches(args);
        addSwitches(args);
        args.parse(stringArgs);
    }
    catch (arg_error& e)
    {
        throw pdal_error(getName() + ": " + e.what());
    }
    return true;
}


int Kernel::doStartup()
{
    return 0;
}


// this just wraps ALL the code in total catch block
int Kernel::run(const StringList& cmdArgs, LogPtr& log)
{
    m_log = log;
    m_manager.setLog(m_log);

    ProgramArgs args;

    try
    {
        if (!doSwitches(cmdArgs, args))
        {
            outputHelp();
            return 0;
        }
    }
    catch (const pdal_error& e)
    {
        Utils::printError(e.what());
        return 1;
    }

    int startup_status = doStartup();
    if (startup_status)
        return startup_status;

    return doExecution(args);
}


int Kernel::doExecution(ProgramArgs& args)
{
    if (m_hardCoreDebug)
        return innerRun(args);

    int status = 1;
    try
    {
        status = innerRun(args);
    }
    catch (pdal::pdal_error const& e)
    {
        Utils::printError(e.what());
        return 1;
    }
    catch (std::exception const& e)
    {
        Utils::printError(e.what());
        return 1;
    }
    catch (...)
    {
        Utils::printError("Caught unexpected exception.");
        return 1;
    }

    return status;
}


int Kernel::innerRun(ProgramArgs& args)
{
    try
    {
        // do any user-level sanity checking
        validateSwitches(args);
    }
    catch (pdal_error& e)
    {
        Utils::printError(e.what());
        outputHelp();
        return -1;
    }

    return execute();
}


void Kernel::outputHelp()
{
    ProgramArgs basicArgs;
    addBasicSwitches(basicArgs);

    ProgramArgs args;
    addSwitches(args);

    std::cout << "usage: " << "pdal " << getShortName() << " [options] " <<
        args.commandLine() << std::endl;

    std::cout << "standard options:" << std::endl;
    basicArgs.dump(std::cout, 2, Utils::screenWidth());
    std::cout << "options:" << std::endl;
    args.dump(std::cout, 2, Utils::screenWidth());

    std::cout <<"\nFor more information, see the full documentation for "
        "PDAL at http://pdal.io/\n" << std::endl;
}


void Kernel::addBasicSwitches(ProgramArgs& args)
{
    static bool s_help;

    args.add("developer-debug",
        "Enable developer debug (don't trap exceptions)", m_hardCoreDebug);
    args.add("label", "A string to label the process with", m_label);
    args.add("driver", "Override reader driver", m_driverOverride);
    args.add("help", "Print help and exit", s_help);
}

Stage& Kernel::makeReader(const std::string& inputFile, std::string driver)
{
    return m_manager.makeReader(inputFile, driver);
}


Stage& Kernel::makeReader(const std::string& inputFile, std::string driver,
    Options options)
{
    return m_manager.makeReader(inputFile, driver, options);
}


Stage& Kernel::makeFilter(const std::string& driver)
{
    return m_manager.makeFilter(driver);
}


Stage& Kernel::makeFilter(const std::string& driver, Stage& parent)
{
    return m_manager.makeFilter(driver, parent);
}


Stage& Kernel::makeFilter(const std::string& driver, Stage& parent,
    Options options)
{
    return m_manager.makeFilter(driver, parent, options);
}


Stage& Kernel::makeWriter(const std::string& outputFile, Stage& parent,
    std::string driver)
{
    return m_manager.makeWriter(outputFile, driver, parent);
}


Stage& Kernel::makeWriter(const std::string& outputFile, Stage& parent,
    std::string driver, Options options)
{
    return m_manager.makeWriter(outputFile, driver, parent, options);
}


Kernel::ParseStageResult Kernel::test_parseStageOption(
    std::string o, std::string& stage, std::string& option, std::string& value)
{
    class TestKernel : public Kernel
    {
    public:
        virtual std::string getName() const
            { return "TestKernel"; }
        int execute()
            { return 0; }
    };

    TestKernel k;
    return k.parseStageOption(o, stage, option, value);
}

} // namespace pdal
