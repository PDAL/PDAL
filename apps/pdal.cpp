/******************************************************************************
* Copyright (c) 2013, Howard Butler (hobu.inc@gmail.com)
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

#include <pdal/kernel/Kernel.hpp>
#include <pdal/pdal_config.hpp>

#include <boost/algorithm/string.hpp>

namespace po = boost::program_options;

std::string headline("------------------------------------------------------------------------------------------");

void outputVersion()
{
    std::cout << headline << std::endl;
    std::cout << "pdal " << "(" << pdal::GetFullVersionString() << ")" << std::endl;
    std::cout << headline << std::endl;
    std::cout << "  available actions: " << std::endl;
    std::cout << "     - info" << std::endl;
    std::cout << "     - pipeline" << std::endl;
    std::cout << "     - query" << std::endl;
    std::cout << "     - translate" << std::endl;
    
    std::cout << std::endl;
}



int main(int argc, char* argv[])
{
    po::options_description options;
    po::positional_options_description positional;
    po::variables_map variables;
    positional.add("action", 1);

    options.add_options()
        ("action", po::value<std::string>(), "action name")
        ("version", po::value<bool>()->zero_tokens()->implicit_value(true), "Show version info")
        ("help,h", po::value<bool>()->zero_tokens()->implicit_value(true), "Print help message")
            ;
    std::vector<std::string> action_args;
    
    if (argc < 2)
    {
        std::cerr << "Action not specified!" << std::endl << std::endl;
        outputVersion(); return 1;
    }
    
    action_args.push_back(argv[0]);
    action_args.push_back(argv[1]);

    try
    {
        po::store(po::command_line_parser(2, (char**)&action_args[0]).
            options(options).positional(positional).run(), 
            variables);
    }
    catch (boost::program_options::unknown_option& e)
    {
#if BOOST_VERSION >= 104200
        throw pdal::kernel::app_usage_error("unknown option: " + e.get_option_name());
#else
        throw pdal::kernel::app_usage_error("unknown option: " + std::string(e.what()));
#endif
    }

    
    int count(argc - 1); // remove the 1st argument
    const char** args = const_cast<const char**>(&argv[1]);
    
    
    if (variables.count("version") || variables.count("help") || !variables.count("action"))
    {
        outputVersion();
        return 0;
    }

    std::string action = variables["action"].as<std::string>();

    if (boost::iequals(action, "translate"))
    {
        pdal::kernel::Translate app(count, args);
        return app.run();
    }

    if (boost::iequals(action, "info"))
    {
        pdal::kernel::Info app(count, args);
        return app.run();
    }

    if (boost::iequals(action, "pipeline"))
    {
        pdal::kernel::Pipeline app(count, args);
        return app.run();
    }

    if (boost::iequals(action, "query"))
    {
        pdal::kernel::Query app(count, args);
        return app.run();
    }
    
    std::cerr << "Action '" << action <<"' not recognized" << std::endl << std::endl;
    outputVersion();
    return 1;
}
