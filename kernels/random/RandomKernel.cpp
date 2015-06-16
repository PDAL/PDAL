/******************************************************************************
* Copyright (c) 2014, Brad Chambers (brad.chambers@gmail.com)
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

#include "RandomKernel.hpp"

#include <boost/program_options.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "kernels.random",
    "Random Kernel",
    "http://pdal.io/kernels/kernels.random.html" );

CREATE_STATIC_PLUGIN(1, 0, RandomKernel, Kernel, s_info)

std::string RandomKernel::getName() const { return s_info.name; }

RandomKernel::RandomKernel()
    : m_bCompress(false)
    , m_numPointsToWrite(0)
    , m_distribution("uniform")
{
}


void RandomKernel::validateSwitches()
{
    if (m_outputFile == "")
        throw app_usage_error("--output/-o required");
}


void RandomKernel::addSwitches()
{
    po::options_description* file_options = new po::options_description("file options");

    file_options->add_options()
    ("output,o", po::value<std::string>(&m_outputFile)->default_value(""), "output file name")
    ("compress,z", po::value<bool>(&m_bCompress)->zero_tokens()->implicit_value(true), "Compress output data (if supported by output format)")
    ("count", po::value<uint64_t>(&m_numPointsToWrite)->default_value(0), "How many points should we write?")
    ("bounds", po::value<BOX3D>(&m_bounds), "Extent (in XYZ to clip output to)")
    ("mean", po::value< std::string >(&m_means), "A comma-separated or quoted, space-separated list of means (normal mode): \n--mean 0.0,0.0,0.0\n--mean \"0.0 0.0 0.0\"")
    ("stdev", po::value< std::string >(&m_stdevs), "A comma-separated or quoted, space-separated list of standard deviations (normal mode): \n--stdev 0.0,0.0,0.0\n--stdev \"0.0 0.0 0.0\"")
    ("distribution", po::value<std::string>(&m_distribution)->default_value("uniform"), "Distribution (uniform / normal)")
    ;

    addSwitchSet(file_options);

    addPositionalSwitch("output", 1);
}


Stage& RandomKernel::makeReader(Options readerOptions)
{
    if (isDebug())
    {
        readerOptions.add<bool>("debug", true);
        uint32_t verbosity(getVerboseLevel());
        if (!verbosity)
            verbosity = 1;

        readerOptions.add<uint32_t>("verbose", verbosity);
        readerOptions.add<std::string>("log", "STDERR");
    }

    StageFactory factory;
    Stage& readerStage = ownStage(factory.createStage("readers.faux"));
    readerStage.setOptions(readerOptions);

    return readerStage;
}


int RandomKernel::execute()
{
    Options readerOptions;
    {
        boost::char_separator<char> sep(SEPARATORS);
        std::vector<double> means;
        tokenizer mean_tokens(m_means, sep);
        for (tokenizer::iterator t = mean_tokens.begin();
            t != mean_tokens.end(); ++t)
        {
            means.push_back(boost::lexical_cast<double>(*t));
        }

        if (means.size())
        {
            readerOptions.add<double >("mean_x", means[0]);
            readerOptions.add<double >("mean_y", means[1]);
            readerOptions.add<double >("mean_z", means[2]);
        }

        std::vector<double> stdevs;
        tokenizer stdev_tokens(m_stdevs, sep);
        for (tokenizer::iterator t = stdev_tokens.begin();
            t != stdev_tokens.end(); ++t)
        {
            stdevs.push_back(boost::lexical_cast<double>(*t));
        }

        if (stdevs.size())
        {
            readerOptions.add<double >("stdev_x", stdevs[0]);
            readerOptions.add<double >("stdev_y", stdevs[1]);
            readerOptions.add<double >("stdev_z", stdevs[2]);
        }

        if (!m_bounds.empty())
            readerOptions.add<BOX3D >("bounds", m_bounds);

        if (boost::iequals(m_distribution, "uniform"))
            readerOptions.add<std::string>("mode", "uniform");
        else if (boost::iequals(m_distribution, "normal"))
            readerOptions.add<std::string>("mode", "normal");
        else if (boost::iequals(m_distribution, "random"))
            readerOptions.add<std::string>("mode", "random");
        else
            throw pdal_error("invalid distribution: " + m_distribution);
        readerOptions.add<int>("num_points", m_numPointsToWrite);
        readerOptions.add<bool>("debug", isDebug());
        readerOptions.add<uint32_t>("verbose", getVerboseLevel());
    }

    Options writerOptions;
    {
        writerOptions.add<std::string>("filename", m_outputFile);
        setCommonOptions(writerOptions);

        if (m_bCompress)
        {
            writerOptions.add<bool>("compression", true);
        }
    }

    Stage& writer = makeWriter(m_outputFile, makeReader(readerOptions));
    writer.setOptions(writerOptions);

    PointTable table;

    UserCallback* callback;
    if (!getProgressShellCommand().size())
        callback = static_cast<UserCallback*>(new PercentageCallback);
    else
        callback = static_cast<UserCallback*>(
            new ShellScriptCallback(getProgressShellCommand()));
    writer.setUserCallback(callback);
    writer.prepare(table);
    PointViewSet viewSet = writer.execute(table);

    if (isVisualize())
        visualize(*viewSet.begin());

    return 0;
}

} // pdal

