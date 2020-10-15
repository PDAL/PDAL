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

#include <io/BufferReader.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/Utils.hpp>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "kernels.split",
    "Split Kernel",
    "http://pdal.io/apps/split.html"
};

CREATE_STATIC_KERNEL(SplitKernel, s_info)

std::string SplitKernel::getName() const
{
    return s_info.name;
}


void SplitKernel::addSwitches(ProgramArgs& args)
{
    args.add("input,i", "Input filename", m_inputFile).setPositional();
    args.add("output,o", "Output filename", m_outputFile).setPositional();
    args.add("length", "Edge length for splitter cells", m_length, 0.0);
    args.add("capacity", "Point capacity of chipper cells", m_capacity);
    args.add("origin_x", "Origin in X axis for splitter cells", m_xOrigin,
        std::numeric_limits<double>::quiet_NaN());
    args.add("origin_y", "Origin in Y axis for splitter cells", m_yOrigin,
        std::numeric_limits<double>::quiet_NaN());
}


void SplitKernel::validateSwitches(ProgramArgs& args)
{
    if (m_length && m_capacity)
        throw pdal_error("Can't specify both length and capacity.");
    if (!m_length && !m_capacity)
        m_capacity = 100000;
    if (m_outputFile.back() == Utils::dirSeparator)
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
    Stage& reader = makeReader(m_inputFile, m_driverOverride);

    Options filterOpts;
    std::string driver = (m_length ? "filters.splitter" : "filters.chipper");
    if (m_length)
    {
        filterOpts.add("length", m_length);
        filterOpts.add("origin_x", m_xOrigin);
        filterOpts.add("origin_y", m_yOrigin);
    }
    else
    {
        filterOpts.add("capacity", m_capacity);
    }
    Stage& f = makeFilter(driver, reader, filterOpts);

    ColumnPointTable table;
    f.prepare(table);
    PointViewSet pvSet = f.execute(table);

    int filenum = 1;
    for (auto& pvp : pvSet)
    {
        BufferReader reader;
        reader.addView(pvp);

        std::string filename = makeFilename(m_outputFile, filenum++);
        Stage& writer = makeWriter(filename, reader, "");

        writer.prepare(table);
        writer.execute(table);
    }
    return 0;
}

} // namespace pdal
