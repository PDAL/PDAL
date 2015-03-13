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

#include <pdal/KernelFactory.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/pdal_config.hpp>

#include <boost/algorithm/string.hpp>

#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace pdal;

std::string headline("------------------------------------------------------------------------------------------");

std::string splitDriverName(std::string const& name)
{
    std::vector<std::string> driverVec;
    boost::algorithm::split(driverVec, name,
        boost::algorithm::is_any_of("."), boost::algorithm::token_compress_on);

    return driverVec[1];
}

void outputVersion()
{
    std::cout << headline << std::endl;
    std::cout << "pdal " << "(" << GetFullVersionString() << ")" << std::endl;
    std::cout << headline << std::endl;
    std::cout << std::endl;
}

void outputHelp()
{
    std::cerr << "Usage: pdal <command> [--debug] [--drivers] [--help] [--options[=<driver name>]] [--version]\n";
    std::cerr << "  --debug                Show debug information\n";
    std::cerr << "  --drivers              Show drivers\n";
    std::cerr << "  -h [ --help ]          Print help message\n";
    std::cerr << "  --options [=arg(=all)] Show driver options\n";
    std::cerr << "  --version              Show version info\n";
    std::cerr << "\n";
        
    std::cerr << "The most commonly used pdal commands are:\n";

    KernelFactory f(false);
    std::vector<std::string> loaded_kernels = f.getKernelNames();

    for (auto name : loaded_kernels)
        std::cout << "   - " << splitDriverName(name) << std::endl;

    std::cout << "See http://pdal.io/apps.html for more detail";
    std::cout << std::endl;
    std::cout << "Run pdal --list-commands for a complete list";
    std::cout << std::endl;
}

void outputDrivers()
{
    StageFactory f(false);
    std::map<std::string, std::string> sm = f.getStageMap();

    std::ostringstream strm;

    std::string tablehead("================================ ==========================================================");
    std::string headings ("Name                             Description");

    strm << std::endl;
    strm << tablehead << std::endl;
    strm << headings << std::endl;
    strm << tablehead << std::endl;

    uint32_t name_column(32);
    uint32_t description_column(57);

    strm << std::left;

    for (auto s : sm)
    {
        std::vector<std::string> lines;
        std::string description(s.second);
        description = boost::algorithm::erase_all_copy(description, "\n");

        Utils::wordWrap(description, lines, description_column-1);
        if (lines.size() == 1)
        {
            strm << std::setw(name_column) << s.first << " "
                 << std::setw(description_column) << description << std::endl;
        }
        else
        {
            strm << std::setw(name_column) << s.first << " "
                 << lines[0] << std::endl;
        }

        std::stringstream blank;
        size_t blanks(33);
        for (size_t i = 0; i < blanks; ++i)
            blank << " ";
        for (size_t i = 1; i < lines.size(); ++i)
            strm << blank.str() << lines[i] << std::endl;
    }

    strm << tablehead << std::endl;
    std::cout << strm.str() << std::endl;
}

void outputOptions(std::string const& n)
{
    StageFactory f(false);

    std::unique_ptr<Stage> s(f.createStage(n));
    if (!s)
    {
        std::cerr << "Unable to create stage " << n << "\n";
        return;
    }

    std::vector<Option> options = s->getDefaultOptions().getOptions();
    if (options.size())
    {
        std::ostringstream strm;

        strm << n << std::endl;
        strm << headline << std::endl;

        std::string tablehead("================================ "
            "=============== =========================================");
        std::string headings ("Name                              "
            "Default          Description");
        
        strm << std::endl;
        strm << tablehead << std::endl;
        strm << headings << std::endl;
        strm << tablehead << std::endl;
        
        uint32_t default_column(15);
        uint32_t name_column(32);
        uint32_t description_column(40);

        strm << std::left;

        for (auto const& opt : options)
        {
            std::string default_value(opt.getValue<std::string>() );
            default_value = boost::algorithm::erase_all_copy(default_value,
                "\n");
            if (default_value.size() > default_column -1 )
            {
                default_value = default_value.substr(0, default_column-3);
                default_value = default_value + "...";
            }
            
            std::vector<std::string> lines;
            std::string description(opt.getDescription());
            description = boost::algorithm::erase_all_copy(description, "\n");
            
            Utils::wordWrap(description, lines, description_column-1);
            if (lines.size() == 1)
            {    
                strm   << std::setw(name_column) << opt.getName() << " " 
                       << std::setw(default_column) << default_value << " " 
                       << std::setw(description_column) << description <<
                       std::endl;
            }
            else
            {
                strm   << std::setw(name_column) << opt.getName() << " " 
                       << std::setw(default_column) << default_value << " " 
                       << lines[0] << std::endl;
            }
            
            std::stringstream blank;
            size_t blanks(49);
            for (size_t i = 0; i < blanks; ++i)
                blank << " ";
            for (size_t i = 1; i < lines.size(); ++i)
                strm << blank.str() <<lines[i] << std::endl;
        }
        strm << tablehead << std::endl;
        strm << std::endl;

        std::cout << strm.str() << std::endl;
    }
    else
        std::cerr << n << " has no options\n";
}

void outputCommands()
{
    KernelFactory f(false);
    std::vector<std::string> loaded_kernels;
    loaded_kernels = f.getKernelNames();
    for (auto name : loaded_kernels)
    {
        std::cout << splitDriverName(name) << std::endl;
    }
}

void outputOptions()
{
    StageFactory f(false);
    StringList nv = f.getStageNames();
    for (auto const& n : nv)
        outputOptions(n);
}

int main(int argc, char* argv[])
{
    // No arguments, print basic usage, plugins will be loaded
    if (argc < 2)
    {
        outputHelp();
        return 1;
    }

    // Discover available kernels without plugins, and test to see if
    // the positional option 'command' is a valid kernel
    KernelFactory f(true);
    std::vector<std::string> loaded_kernels;
    loaded_kernels = f.getKernelNames();

    bool isValidKernel = false;
    std::string command(argv[1]);
    std::string fullname;
    for (auto name : loaded_kernels)
    {
        if (boost::iequals(argv[1], splitDriverName(name)))
        {
            fullname = name;
            isValidKernel = true;
            break;
        }
    }

    // If the kernel was not available, then light up the plugins and retry
    if (!isValidKernel)
    {
        KernelFactory f(false);
        loaded_kernels.clear();
        loaded_kernels = f.getKernelNames();

        for (auto name : loaded_kernels)
        {
            if (boost::iequals(argv[1], splitDriverName(name)))
            {
                fullname = name;
                isValidKernel = true;
                break;
            }
        }
    }

    // Dispatch execution to the kernel, passing all remaining args
    if (isValidKernel)
    {
        int count(argc - 1); // remove the 1st argument
        const char** args = const_cast<const char**>(&argv[1]);
        std::unique_ptr<Kernel> app = f.createKernel(fullname);
        return app->run(count, args, command);
    }

    // Otherwise, process the remaining args to see if they are supported
    bool debug = false;
    bool drivers = false;
    bool help = false;
    bool options = false;
    bool version = false;
    std::string optString("all");  // --options will default to displaying information on all available stages
    for (int i = 1; i < argc; ++i)
    {
        if (boost::iequals(argv[i], "--debug"))
        {
            debug = true;
        }
        else if (boost::iequals(argv[i], "--drivers"))
        {
            drivers = true;
        }
        else if (boost::iequals(argv[i], "--help") || boost::iequals(argv[i], "-h"))
        {
            help = true;
        }
        else if (boost::algorithm::istarts_with(argv[i], "--options"))
        {
            std::vector<std::string> optionsVec;
            // we are rather unsophisticated for now, only splitting on '=', no spaces allowed
            boost::algorithm::split(optionsVec, argv[i],
                boost::algorithm::is_any_of("="), boost::algorithm::token_compress_on);
            options = true;
            if (optionsVec.size() == 2)
                optString = optionsVec[1];
        }
        else if (boost::iequals(argv[i], "--version"))
        {
            version = true;
        }
        else if (boost::iequals(argv[i], "--list-commands"))
        {
            outputCommands();
            return 0;
        }
        else
        {
            if (boost::algorithm::istarts_with(argv[i], "--"))
                std::cerr << "Unknown option '" << argv[i] <<"' not recognized" << std::endl << std::endl;
        }
    }

    if (version)
    {
        outputVersion();
        return 0;
    }

    if (drivers)
    {
        outputDrivers();
        return 0;
    }

    if (options)
    {
        if (boost::iequals(optString, "all"))
            outputOptions();
        else
            outputOptions(optString);
        return 0;
    }

    if (debug)
    {
        std::cerr << getPDALDebugInformation() << std::endl;
        return 0;
    }

    if (help)
    {
        outputHelp();
        return 0;
    }

    if (!isValidKernel)
        std::cerr << "Command '" << command <<"' not recognized" << std::endl << std::endl;
    outputHelp();
    return 1;
}

