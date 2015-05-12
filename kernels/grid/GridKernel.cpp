/******************************************************************************
* Copyright (c) 2015, Oscar Martinez Rubi (o.rubi@esciencecenter.nl)
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

#include "GridKernel.hpp"

#include <pdal/BufferReader.hpp>
#include <pdal/KernelSupport.hpp>
#include <pdal/StageFactory.hpp>

#include <boost/program_options.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "kernels.grid",
    "Grid Kernel",
    "http://pdal.io/kernels/kernels.split.html" );

CREATE_STATIC_PLUGIN(1, 0, GridKernel, Kernel, s_info)

std::string GridKernel::getName() const
{
    return s_info.name;
}


void GridKernel::addSwitches()
{
    po::options_description* file_options =
        new po::options_description("file options");

    file_options->add_options()
        ("num_x", po::value<uint32_t>(&num_x)->default_value(0),
         "Number of grid cells in X")
        ("num_y", po::value<uint32_t>(&num_y)->default_value(0),
         "Number of grid cells in Y")
        ("min_x", po::value<double>(&min_x)->default_value(0),
         "Minimum X of grid")
        ("min_y", po::value<double>(&min_y)->default_value(0),
         "Minimum Y of grid")
        ("max_x", po::value<double>(&max_x)->default_value(0),
         "Maximum X of grid")
        ("max_y", po::value<double>(&max_y)->default_value(0),
         "Maximum Y of grid")
        ("input,i", po::value<std::string>(&m_inputFile)->default_value(""),
         "input file name")
        ("output,o", po::value<std::string>(&m_outputFile)->default_value(""),
         "output file name")
        ;

    addSwitchSet(file_options);
    addPositionalSwitch("input", 1);
    addPositionalSwitch("output", 1);
    addPositionalSwitch("num_x", 1);
    addPositionalSwitch("num_y", 1);
    addPositionalSwitch("min_x", 1);
    addPositionalSwitch("min_y", 1);
    addPositionalSwitch("max_x", 1);
    addPositionalSwitch("max_y", 1);
}


void GridKernel::validateSwitches()
{
#ifdef WIN32
    char pathSeparator = '\\';
#else
    char pathSeparator = '/';
#endif

    if (m_outputFile.back() == pathSeparator)
        m_outputFile += m_inputFile;
}


namespace
{
std::string makeFilename(const std::string& s, int i)
{
    std::string out = s;
    auto pos = out.find_last_of('.');
    if (pos == out.npos)
        pos = out.length();
    out.insert(pos, std::string("_") + std::to_string(i));
    return out;
}
}


int GridKernel::execute()
{
    PointTable table;

    Options readerOpts;
    readerOpts.add("filename", m_inputFile);
    readerOpts.add("debug", isDebug());
    readerOpts.add("verbose", getVerboseLevel());

    Stage& reader = makeReader(m_inputFile);
    reader.setOptions(readerOpts);

    std::unique_ptr<Stage> f;
    StageFactory factory;
    Options filterOpts;
    f.reset(factory.createStage("filters.gridder"));
    filterOpts.add("num_x", num_x);
    filterOpts.add("num_y", num_y);
    filterOpts.add("min_x", min_x);
    filterOpts.add("min_y", min_y);
    filterOpts.add("max_x", max_x);
    filterOpts.add("max_y", max_y);

    f->setInput(reader);
    f->setOptions(filterOpts);

    f->prepare(table);
    PointViewSet pvSet = f->execute(table);

    int filenum = 1;
    for (auto& pvp : pvSet)
    {
        BufferReader reader;
        reader.addView(pvp);

        std::string filename = makeFilename(m_outputFile, filenum++);
        Stage& writer = makeWriter(filename, reader);

        writer.prepare(table);
        writer.execute(table);
    }
    return 0;
}

} // namespace pdal

