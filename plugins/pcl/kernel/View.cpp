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

#include "View.hpp"
#include <pdal/kernel/KernelFactory.hpp>

CREATE_KERNEL_PLUGIN(view, pdal::kernel::View)

namespace pdal
{
namespace kernel
{

View::View()
    : Kernel()
    , m_inputFile("")
{
    return;
}


void View::validateSwitches()
{
    if (m_inputFile == "")
    {
        throw app_usage_error("--input/-i required");
    }

    return;
}


void View::addSwitches()
{
    po::options_description* file_options = new po::options_description("file options");

    file_options->add_options()
    ("input,i", po::value<std::string>(&m_inputFile)->default_value(""), "input file name")
    ;

    addSwitchSet(file_options);

    addPositionalSwitch("input", 1);
}

std::unique_ptr<Stage> View::makeReader(Options readerOptions)
{
    if (isDebug())
    {
        readerOptions.add<bool>("debug", true);
        boost::uint32_t verbosity(getVerboseLevel());
        if (!verbosity)
            verbosity = 1;

        readerOptions.add<boost::uint32_t>("verbose", verbosity);
        readerOptions.add<std::string>("log", "STDERR");
    }

    Stage* stage = AppSupport::makeReader(m_inputFile);
    stage->setOptions(readerOptions);
    std::unique_ptr<Stage> reader_stage(stage);

    return reader_stage;
}


int View::execute()
{
    Options readerOptions;
    readerOptions.add<std::string>("filename", m_inputFile);
    readerOptions.add<bool>("debug", isDebug());
    readerOptions.add<boost::uint32_t>("verbose", getVerboseLevel());

    std::unique_ptr<Stage> readerStage = makeReader(readerOptions);

    Options writerOptions;
    setCommonOptions(writerOptions);

    std::unique_ptr<Writer> writer(AppSupport::makeWriter("foo.pclviz",readerStage.get()));

    std::vector<std::string> cmd = getProgressShellCommand();
    UserCallback *callback =
        cmd.size() ? (UserCallback *)new ShellScriptCallback(cmd) :
        (UserCallback *)new HeartbeatCallback();

    PointContext ctx;
    writer->setUserCallback(callback);

    for (auto pi: getExtraStageOptions())
    {
        std::string name = pi.first;
        Options options = pi.second;
        std::vector<Stage*> stages = writer->findStage(name);
        for (auto s: stages)
        {
            Options opts = s->getOptions();
            for (auto o: options.getOptions())
                opts.add(o);
            s->setOptions(opts);
        }
    }

    writer->setOptions(writerOptions);
    writer->prepare(ctx);
    writer->execute(ctx);

    return 0;
}

} // kernel
} // pdal
