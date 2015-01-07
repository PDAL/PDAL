/******************************************************************************
* Copyright (c) 2013, Howard Butler (hobu.inc@gmail.com)
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

//#include <pdal/Kernels.hpp>
#include "KernelFactory.hpp"
#include <pdal/pdal_config.hpp>

#include <boost/algorithm/string.hpp>

using namespace pdal;
namespace po = boost::program_options;

std::string headline("------------------------------------------------------------------------------------------");

void outputVersion()
{
    std::cout << headline << std::endl;
    std::cout << "pdal " << "(" << GetFullVersionString() << ")" << std::endl;
    std::cout << headline << std::endl;
    std::cout << std::endl;
}

void outputHelp(po::options_description const& options)
{
    std::cerr << "Usage: pdal <command> [--debug] [--drivers] [--help] [--options[=<driver name>]] [--version]" << std::endl;
    std::cerr << options << std::endl;
        
    std::cerr << "The most commonly used pdal commands are:" << std::endl;

    KernelFactory f;
    std::map<std::string, KernelInfo> const& kernels = f.getKernelInfos();
    for (auto i = kernels.begin(); i != kernels.end(); ++i)
        std::cout << "    - " << i->second.getName() << std::endl;

    std::cout << "See http://pdal.io/apps.html for more detail";
    std::cout << std::endl;
}

void outputDrivers()
{
    StageFactory f;
    std::map<std::string, StageInfo> const& drivers = f.getStageInfos();

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

    for (auto i = drivers.begin(); i != drivers.end(); ++i)
    {
        std::vector<std::string> lines;
        std::string description(i->second.getDescription());
        description = boost::algorithm::erase_all_copy(description, "\n");

        Utils::wordWrap(description, lines, description_column-1);
        if (lines.size() == 1)
        {
            strm << std::setw(name_column) << i->second.getName() << " "
                 << std::setw(description_column) << description << std::endl;
        }
        else
        {
            strm << std::setw(name_column) << i->second.getName() << " "
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

void outputOptions(std::string const& opt)
{
    StageFactory f;
    std::cout << opt << std::endl;
    std::cout << f.toRST(opt) << std::endl;
}

int main(int argc, char* argv[])
{
    KernelFactory f;

    po::options_description options;
    po::positional_options_description positional;
    po::variables_map variables;
    positional.add("command", 1);

    options.add_options()
        ("command", po::value<std::string>(), "command name")
        ("debug", po::value<bool>()->zero_tokens()->implicit_value(true), "Show debug information")
        ("drivers", po::value<bool>()->zero_tokens()->implicit_value(true), "Show drivers")
        ("help,h", po::value<bool>()->zero_tokens()->implicit_value(true), "Print help message")
        ("options", po::value<std::string>()->implicit_value("all"), "Show driver options")
        ("version", po::value<bool>()->zero_tokens()->implicit_value(true), "Show version info")
            ;

    if (argc < 2)
    {
        outputHelp(options);
        return 1;
    }

    try
    {
        po::store(po::command_line_parser(2, argv).
            options(options).positional(positional).run(),
            variables);
    }
    catch (boost::program_options::unknown_option& e)
    {
#if BOOST_VERSION >= 104200

        std::cerr << "Unknown option '" << e.get_option_name() <<"' not recognized" << std::endl << std::endl;
#else
        std::cerr << "Unknown option '" << std::string(e.what()) <<"' not recognized" << std::endl << std::endl;
#endif
        outputVersion();
        return 1;
    }

    int count(argc - 1); // remove the 1st argument
    const char** args = const_cast<const char**>(&argv[1]);

    if (variables.count("version"))
    {
        outputVersion();
        return 0;
    }

    if (variables.count("drivers"))
    {
        outputDrivers();
        return 0;
    }

    if (variables.count("options"))
    {
        std::string opt = variables["options"].as<std::string>();
        outputOptions(opt);
        return 0;
    }

    if (variables.count("debug"))
    {
        std::cerr << getPDALDebugInformation() << std::endl;
        return 0;
    }

    if (variables.count("help") || !variables.count("command"))
    {
        outputHelp(options);
        return 0;
    }

    std::string command = variables["command"].as<std::string>();

    bool isValidKernel = false;
    std::map<std::string, KernelInfo> const& kernels = f.getKernelInfos();
    for (auto i = kernels.begin(); i != kernels.end(); ++i)
    {
        if (boost::iequals(command, i->second.getName()))
        {
            isValidKernel = true;
        }
    }

    if (isValidKernel)
    {
        std::unique_ptr<Kernel> app(f.createKernel(command));
        return app->run(count, args, command);
    }

    std::cerr << "Command '" << command <<"' not recognized" << std::endl << std::endl;
    outputHelp(options);
    return 1;
}
