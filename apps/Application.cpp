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

#include <iostream>

#include <boost/timer.hpp>

#include <pdal/pdal_config.hpp>
#include <pdal/GlobalEnvironment.hpp>
#include <pdal/StageFactory.hpp>

#include "Application.hpp"
#include <vector>
using namespace pdal;
namespace po = boost::program_options;


Application::Application(int argc, char* argv[], const std::string& appName)
    : m_isDebug(false)
    , m_verboseLevel(0)
    , m_showHelp(false)
    , m_showDrivers(false)
    , m_showOptions("")
    , m_showVersion(false)
    , m_showTime(false)
    , m_argc(argc)
    , m_argv(argv)
    , m_appName(appName)
    , m_hardCoreDebug(false)
    , m_reportDebug(false)
    , m_usestdin(false)
    , m_chunkSize(0)
{
    return;
}

Application::~Application()
{
    if (m_options.size())
    {
        typedef std::vector<boost::program_options::options_description*>::const_iterator  Iter;
        for (Iter i = m_options.begin(); i != m_options.end(); ++i)
        {
            delete (*i);
        }        
    }
}

int Application::do_switches()
{
    try
    {
        // add -h, -v, etc
        addBasicSwitchSet();

        // add the options for the derived application
        addSwitches();

        // parse the command line
        parseSwitches();
    }
    catch (std::exception const& e)
    {
        const std::string s("Caught exception handling switches: ");
        printError(s + e.what());
        return 1;
    }
    catch (...)
    {
        printError("Caught unknown exception handling switches");
        return 1;
    }

    return 0;
}


int Application::do_startup()
{
    try
    {
        pdal::GlobalEnvironment::startup();
    }
    catch (std::exception const& e)
    {
        const std::string s("Caught exception initializing PDAL: ");
        printError(s + e.what());
        return 1;
    }
    catch (...)
    {
        printError("Caught unknown exception initializing PDAL");
        return 1;
    }

    return 0;
}


int Application::do_execution()
{
    
    if (m_reportDebug)
    {
        std::cout << "PDAL's build debug status is '" << PDAL_BUILD_TYPE << "'" << std::endl;
        return 1;
    }
    
    if (m_hardCoreDebug)
    {
        int status = innerRun();
        return status;
    }

    int status = 1;

    try
    {
        status = innerRun();
    }
    catch (pdal::pdal_error const& e)
    {
        const std::string s("Caught PDAL exception: ");
        printError(s + e.what());
        return 1;
    }
    catch (std::exception const& e)
    {
        const std::string s("Caught exception: ");
        printError(s + e.what());
        return 1;
    }
    catch (...)
    {
        printError("Caught unknown exception");
        return 1;
    }

    return status;
}


int Application::do_shutdown()
{
    try
    {
        pdal::GlobalEnvironment::shutdown();
    }
    catch (std::exception const& e)
    {
        const std::string s("Caught exception shutting down PDAL: ");
        printError(s + e.what());
        return 1;
    }
    catch (...)
    {
        printError("Caught unknown exception shutting down PDAL");
        return 1;
    }

    return 0;
}


// this just wraps ALL the code in total catch block
int Application::run()
{
    int switches_status = do_switches();
    if (switches_status)
        return switches_status;

    int startup_status = do_startup();
    if (startup_status)
        return startup_status;

    int execution_status = do_execution();

    // note we will try to shutdown cleanly even if we got an error condition
    // in the execution phase

    int shutdown_status = do_shutdown();

    if (execution_status)
        return execution_status;

    return shutdown_status;
}


int Application::innerRun()
{
    // handle the well-known options
    if (m_showVersion) 
    {
        outputVersion();
        return 0;
    }
    
    if (m_showHelp) 
    {
        outputHelp();
        return 0;
    }
    
    if (m_showDrivers)
    {
        outputDrivers();
        return 0;
    }
    if (!m_showOptions.empty())
    {
        outputOptions(m_showOptions);
        return 0;
    }
    try
    {
        // do any user-level sanity checking
        validateSwitches();
    }
    catch (app_usage_error e)
    {
        std::string s("Usage error: ");
        printError(s + e.what());
        outputHelp();
        return 1;
    }

    boost::timer timer;
    
    int status = execute();
    
    if (status == 0 && m_showTime)
    {
        const double t = timer.elapsed();
        std::cout << "Elapsed time: " << t << " seconds" << std::endl;
    }

    return status;
}


void Application::printError(const std::string& err) const
{
    std::cout << err << std::endl;
    std::cout << std::endl;
}


bool Application::isDebug() const
{
    return m_isDebug;
}


boost::uint32_t Application::getVerboseLevel() const
{
    return m_verboseLevel;
}


void Application::addSwitchSet(po::options_description* options)
{
    if (!options) return;
    m_options.push_back(options);
}


void Application::addPositionalSwitch(const char* name, int max_count)
{
    m_positionalOptions.add(name, max_count);
}

void Application::outputDrivers()
{
    pdal::StageFactory factory;
    std::map<std::string, pdal::StageInfo> const& drivers = factory.getStageInfos();
    typedef std::map<std::string, pdal::StageInfo>::const_iterator Iterator;
    
    for (Iterator i = drivers.begin(); i != drivers.end(); ++i)
    {
        std::cout << " '" << i->first << "' -- '"<< i->second.getDescription()<<"'"<<std::endl;
    }
}

void Application::outputOptions(std::string const& driver)
{
    pdal::StageFactory* factory = new pdal::StageFactory;
    std::map<std::string, pdal::StageInfo> const& drivers = factory->getStageInfos();
    typedef std::map<std::string, pdal::StageInfo>::const_iterator Iterator;
    
    Iterator i = drivers.find(driver);
    if ( i != drivers.end())
    {
        std::cout << "Options for driver '" << i->first << "'" << std::endl;

        std::vector<Option> options = i->second.getProvidedOptions();
        for (std::vector<Option>::const_iterator it = options.begin();
            it != options.end();
            ++it)
        {
            pdal::Option const& opt = *it;
            std::cout   << "   '" << opt.getName() << "' --- '" 
                        << opt.getDescription() << "' " 
                        << "default: '" << opt.getValue<std::string>() << "'"
                        << std::endl;
        }
    }
    else
    {
        for (Iterator i = drivers.begin(); i != drivers.end(); ++i)
        {
            std::cout << "Options for driver '" << " '" << i->first << "' -- '"<< i->second.getDescription()<<"'"<<std::endl;        
            std::vector<Option> options = i->second.getProvidedOptions();
            for (std::vector<Option>::const_iterator it = options.begin();
                it != options.end();
                ++it)
            {
                pdal::Option const& opt = *it;
                std::cout   << "   '" << opt.getName() << "' --- '" 
                            << opt.getDescription() << "' " 
                            << "default: '" << opt.getValue<std::string>() << "'"
                            << std::endl;
            }
        }
        
    }
}


void Application::outputHelp()
{
    outputVersion();

    std::vector<po::options_description*>::const_iterator iter;
    for (iter = m_options.begin(); iter != m_options.end(); ++iter)
    {
        const po::options_description* options = *iter;
        std::cout << *options;
        std::cout << std::endl;
    }

    std::cout <<"\nFor more information, see the full documentation for PDAL at:\n";
    
    std::cout << "  http://pointcloud.org/\n";
    std::cout << "--------------------------------------------------------------------\n";
    std::cout << std::endl;

    return;
}


void Application::outputVersion()
{
    std::cout << "--------------------------------------------------------------------\n";
    std::cout << m_appName << " (" << pdal::GetFullVersionString() << ")\n";
    std::cout << "--------------------------------------------------------------------\n";
    std::cout << std::endl;
}


void Application::addBasicSwitchSet()
{
    po::options_description* basic_options = new po::options_description("basic options");

    basic_options->add_options()
        ("help,h", po::value<bool>(&m_showHelp)->zero_tokens()->implicit_value(true), "Print help message")
        ("drivers", po::value<bool>(&m_showDrivers)->zero_tokens()->implicit_value(true), "Show currently registered drivers (including dynamic with PDAL_DRIVER_PATH)")
        ("options", po::value<std::string>(&m_showOptions)->implicit_value("all"), "Show available options for a driver")
        ("debug,d", po::value<bool>(&m_isDebug)->zero_tokens()->implicit_value(true), "Enable debug mode")
        ("report-debug", po::value<bool>(&m_reportDebug)->zero_tokens()->implicit_value(true), "Report PDAL compilation DEBUG status")
        ("developer-debug", po::value<bool>(&m_hardCoreDebug)->zero_tokens()->implicit_value(true), "Enable developer debug mode (don't trap exceptions so segfaults are thrown)")
        ("verbose,v", po::value<boost::uint32_t>(&m_verboseLevel)->default_value(0), "Set verbose message level")
        ("version", po::value<bool>(&m_showVersion)->zero_tokens()->implicit_value(true), "Show version info")
        ("timer", po::value<bool>(&m_showTime)->zero_tokens()->implicit_value(true), "Show execution time")
        ("stdin,s", po::value<bool>(&m_usestdin)->zero_tokens()->implicit_value(true), "Read pipeline XML from stdin")
        ("chunk_size", po::value<boost::uint32_t>(&m_chunkSize)->default_value(0), "Use a specified buffer capacity rather than attempting to read the entire pipeline in a single buffer")
        ("heartbeat", po::value< std::vector<std::string> >(&m_heartbeat_shell_command), "Shell command to run for every progress heartbeat")

        ;

    addSwitchSet(basic_options);
    
    return;
}


void Application::parseSwitches()
{
    po::options_description options;

    std::vector<po::options_description*>::iterator iter1;
    for (iter1 = m_options.begin(); iter1 != m_options.end(); ++iter1)
    {
        po::options_description* sub_options = *iter1;
        options.add(*sub_options);
    }

    try
    {
        po::store(po::command_line_parser(m_argc, m_argv).
            options(options).positional(m_positionalOptions).run(), 
            m_variablesMap);
    }
    catch (boost::program_options::unknown_option e)
    {
#if BOOST_VERSION >= 104200
        throw app_usage_error("unknown option: " + e.get_option_name());
#else
        throw app_usage_error("unknown option: " + std::string(e.what()));
#endif
    }

    po::notify(m_variablesMap);

    return;
}

