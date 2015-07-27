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

#include "SplitKernel.hpp"

#include <pdal/BufferReader.hpp>
#include <pdal/KernelSupport.hpp>
#include <pdal/StageFactory.hpp>

#include <boost/program_options.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "kernels.split",
    "Split Kernel",
    "http://pdal.io/kernels/kernels.split.html" );

CREATE_STATIC_PLUGIN(1, 0, SplitKernel, Kernel, s_info)

std::string SplitKernel::getName() const
{
    return s_info.name;
}

    double m_length;
    double m_xOrigin;
    double m_yOrigin;

void SplitKernel::addSwitches()
{
    po::options_description* file_options =
        new po::options_description("file options");

    file_options->add_options()
        ("length", po::value<double>(&m_length)->default_value(0.0),
         "Edge length for splitter cells")
        ("capacity", po::value<uint32_t>(&m_capacity)->default_value(0),
         "Point capacity of chipper cells")
        ("origin_x", po::value<double>(&m_xOrigin)->default_value(std::numeric_limits<double>::quiet_NaN()),
         "Origin in X axis for splitter cells")
        ("origin_y", po::value<double>(&m_yOrigin)->default_value(std::numeric_limits<double>::quiet_NaN()),
         "Origin in Y axis for splitter cells")
        ("input,i", po::value<std::string>(&m_inputFile)->default_value(""),
         "input file name")
        ("output,o", po::value<std::string>(&m_outputFile)->default_value(""),
         "output file name")
        ;

    addSwitchSet(file_options);
    addPositionalSwitch("input", 1);
    addPositionalSwitch("output", 1);
}


void SplitKernel::validateSwitches()
{
#ifdef WIN32
    char pathSeparator = '\\';
#else
    char pathSeparator = '/';
#endif
    
    if (m_length && m_capacity)
        throw pdal_error("Can't specify for length and capacity.");
    if (!m_length && !m_capacity)
        m_capacity = 100000;
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


int SplitKernel::execute()
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
    if (m_length)
    {
        f.reset(factory.createStage("filters.splitter"));
        filterOpts.add("length", m_length);
        filterOpts.add("origin_x", m_xOrigin);
        filterOpts.add("origin_y", m_yOrigin);
    }
    else
    {
        f.reset(factory.createStage("filters.chipper"));
        filterOpts.add("capacity", m_capacity);
    }
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

