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

#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

using namespace pdal;

std::string headline(Utils::screenWidth(), '-');

std::string splitDriverName(std::string const& name)
{
    std::string out;

    StringList names = Utils::split2(name, '.');
    if (names.size() == 2)
        out = names[1];
    return out;
}

void outputVersion()
{
    std::cout << headline << std::endl;
    std::cout << "pdal " << GetFullVersionString() << std::endl;
    std::cout << headline << std::endl;
    std::cout << std::endl;
}

void outputHelp()
{
    std::cout << "Usage: pdal <command> [--debug] [--drivers] [--help] "
        "[--options[=<driver name>]] [--version]" << std::endl << std::endl;
    std::cout << "  --debug      Show debug information" << std::endl;
    std::cout << "  --drivers    Show drivers" << std::endl;
    std::cout << "  -h [--help]  Print help message" << std::endl;
    std::cout << "  --options [=arg(=all)]" << std::endl;
    std::cout << "               Show driver options" << std::endl;
    std::cout << "  --version    Show version info" << std::endl;
    std::cout << std::endl;

    std::cout << "The following commands are available:" << std::endl;

    KernelFactory f(false);
    StringList loaded_kernels = f.getKernelNames();

    for (auto name : loaded_kernels)
        std::cout << "   - " << splitDriverName(name) << std::endl;
    std::cout << "See http://pdal.io/apps.html for more detail" << std::endl;
}

void outputDrivers()
{
    StageFactory f(false);

    std::ostringstream strm;

    int nameColLen(25);
    int descripColLen(Utils::screenWidth() - nameColLen - 1);

    std::string tablehead(std::string(nameColLen, '=') + ' ' +
        std::string(descripColLen, '='));

    strm << std::endl;
    strm << tablehead << std::endl;
    strm << std::left << std::setw(nameColLen) << "Name" <<
        " Description" << std::endl;
    strm << tablehead << std::endl;

    strm << std::left;

    std::map<std::string, std::string> sm = f.getStageMap();
    for (auto s : sm)
    {
        std::string name = s.first;
        StringList lines = Utils::wordWrap(s.second, descripColLen - 1);
        for (size_t i = 0; i < lines.size(); ++i)
        {
            strm << std::setw(nameColLen) << name << " " <<
                lines[i] << std::endl;
            name.clear();
        }
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

    std::cout << n << std::endl;
    std::cout << headline << std::endl;

    std::vector<Option> options = s->getDefaultOptions().getOptions();
    if (options.empty())
    {
        std::cout << "No options" << std::endl << std::endl;
        return;
    }

    for (auto const& opt : options)
    {
        std::string name = opt.getName();
        std::string defVal = Utils::escapeNonprinting(
            opt.getValue<std::string>());
        std::string description = opt.getDescription();

        std::cout << name;
        if (!defVal.empty())
            std::cout << " [" << defVal << "]";
        std::cout << std::endl;

        if (!description.empty())
        {
            StringList lines =
                Utils::wordWrap(description, headline.size() - 6);
            for (std::string& line : lines)
                std::cout << "    " << line << std::endl;
        }
        std::cout << std::endl;
    }
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

