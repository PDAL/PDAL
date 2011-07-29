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

#include <pdal/PipelineReader.hpp>
#include <pdal/PipelineManager.hpp>

#include "Application.hpp"

using namespace pdal;
namespace po = boost::program_options;


class Application_pcpipeline : public Application
{
public:
    Application_pcpipeline(int argc, char* argv[]);
    int execute();

private:
    void addOptions();
    bool validateOptions();

    std::string m_inputFile;
};


Application_pcpipeline::Application_pcpipeline(int argc, char* argv[])
    : Application(argc, argv, "pcpipeline")
    , m_inputFile("")
{
    return;
}


bool Application_pcpipeline::validateOptions()
{
    if (!hasOption("input"))
    {
        usageError("--input/-i required");
        return false;
    }

    return true;
}


void Application_pcpipeline::addOptions()
{
    po::options_description* file_options = new po::options_description("file options");

    file_options->add_options()
        ("input,i", po::value<std::string>(&m_inputFile), "input file name (required)")
        ;

    addOptionSet(file_options);
}


int Application_pcpipeline::execute()
{
    if (!Utils::fileExists(m_inputFile))
    {
        runtimeError("file not found: " + m_inputFile);
        return 1;
    }

    try
    {
        pdal::PipelineManager manager;

        pdal::PipelineReader reader(manager, isDebug(), getVerboseLevel());
        reader.readWriterPipeline(m_inputFile);

        const boost::uint64_t np = manager.execute();

        std::cout << "Wrote " << np << " points.\n";
    }
    catch (pdal::pdal_error ex)
    {
        std::cout << "Caught exception: " << ex.what() << "\n";
        return 1;
    }

    return 0;
}


int main(int argc, char* argv[])
{
    Application_pcpipeline app(argc, argv);
    return app.run();
}



