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

#include <libpc/libpc_config.hpp>

#include "Application.hpp"

using namespace libpc;
namespace po = boost::program_options;


Application::Application(int argc, char* argv[], const std::string& appName)
    : m_isVerbose(false)
    , m_argc(argc)
    , m_argv(argv)
    , m_appName(appName)
{
    return;
}

int Application::run()
{
    // add -h, -v, etc
    addBasicOptionSet();

    // add the options for the derived application
    addOptions();

    // parse the command line
    parseOptions();

    // handle the well-known options
    if (hasOption("version")) 
    {
        outputVersion();
        return 0;
    }
    
    if (hasOption("help")) 
    {
        outputHelp();
        return 0;
    }

    // do any user-level sanity checking
    bool happy = validateOptions();
    if (!happy)
    {
        outputHelp();
        return 1;
    }

    boost::timer timer;
    
    // call derived function
    int status = 0;
    
    // try
    // {
        status = execute();
    // }
    // catch (std::exception e)
    // {
    //     const std::string s(e.what());
    //     runtimeError("Caught exception: " + s);
    //     status = 1;
    // }
    // catch (...)
    // {
    //     runtimeError("Caught unknown exception");
    //     status = 1;
    // }

    if (status == 0 && hasOption("timer"))
    {
        const double t = timer.elapsed();
        std::cout << "Elapsed time: " << t << " seconds" << std::endl;
    }

    return status;
}


void Application::usageError(const std::string& err)
{
    std::cout << "Usage error: " << err << std::endl;
    std::cout << std::endl;
}


void Application::runtimeError(const std::string& err)
{
    std::cout << "Error: " << err << std::endl;
    std::cout << std::endl;
}


bool Application::hasOption(const std::string& name)
{
    return m_variablesMap.count(name) > 0;
}


bool Application::isVerbose() const
{
    return m_isVerbose;
}


void Application::addOptionSet(po::options_description* options)
{
    if (!options) return;
    m_options.push_back(options);
}


void Application::addPositionalOption(const char* name, int max_count)
{
    m_positionalOptions.add(name, max_count);
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

    std::cout <<"\nFor more information, see the full documentation for libPC at:\n";
    
    std::cout << "  http://libpc.org/\n";
    std::cout << "--------------------------------------------------------------------\n";
    std::cout << std::endl;

    return;
}


void Application::outputVersion()
{
    std::cout << "--------------------------------------------------------------------\n";
    std::cout << m_appName << " (" << libpc::GetFullVersionString() << ")\n";
    std::cout << "--------------------------------------------------------------------\n";
    std::cout << std::endl;
}


void Application::addBasicOptionSet()
{
    po::options_description* basic_options = new po::options_description("basic options");

    basic_options->add_options()
        ("help,h", "produce help message")
        ("verbose,v", po::value<bool>(&m_isVerbose)->zero_tokens(), "Verbose message output")
        ("version", "Show version info")
        ("timer", "Show execution time")
        ;

    addOptionSet(basic_options);

    return;
}


void Application::parseOptions()
{
    po::options_description options;

    std::vector<po::options_description*>::iterator iter1;
    for (iter1 = m_options.begin(); iter1 != m_options.end(); ++iter1)
    {
        po::options_description* sub_options = *iter1;
        options.add(*sub_options);
    }

    po::store(po::command_line_parser(m_argc, m_argv).
        options(options).positional(m_positionalOptions).run(), 
        m_variablesMap);

    po::notify(m_variablesMap);

    return;
}

