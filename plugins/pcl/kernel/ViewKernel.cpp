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

#include "ViewKernel.hpp"

#include <pdal/KernelFactory.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "kernels.view",
    "View Kernel",
    "http://pdal.io/kernels/kernels.view.html" );

CREATE_SHARED_PLUGIN(1, 0, ViewKernel, Kernel, s_info)

std::string ViewKernel::getName() const { return s_info.name; }

// Support for parsing point numbers.  Points can be specified singly or as
// dash-separated ranges.  i.e. 6-7,8,19-20
namespace {

using namespace std;

vector<string> tokenize(const string s, char c)
{
    string::const_iterator begin;
    string::const_iterator end;
    vector<string> strings;
    begin = s.begin();
    while (true)
    {
        end = find(begin, s.end(), c);
        strings.push_back(string(begin, end));
        if (end == s.end())
            break;
        begin = end + 1;
    }
    return strings;
}

uint32_t parseInt(const string& s)
{
    try
    {
        return boost::lexical_cast<uint32_t>(s);
    }
    catch (boost::bad_lexical_cast)
    {
        throw app_runtime_error(string("Invalid integer: ") + s);
    }
}


void addSingle(const string& s, vector<uint32_t>& points)
{
    points.push_back(parseInt(s));
}


void addRange(const string& begin, const string& end,
    vector<uint32_t>& points)
{
    uint32_t low = parseInt(begin);
    uint32_t high = parseInt(end);
    if (low > high)
        throw app_runtime_error(string("Range invalid: ") + begin + "-" + end);
    while (low <= high)
        points.push_back(low++);
}


vector<uint32_t> getListOfPoints(std::string p)
{
    vector<uint32_t> output;

    //Remove whitespace from string with awful remove/erase idiom.
    p.erase(remove_if(p.begin(), p.end(), ::isspace), p.end());

    vector<string> ranges = tokenize(p, ',');
    for (string s : ranges)
    {
        vector<string> limits = tokenize(s, '-');
        if (limits.size() == 1)
            addSingle(limits[0], output);
        else if (limits.size() == 2)
            addRange(limits[0], limits[1], output);
        else
            throw app_runtime_error(string("Invalid point range: ") + s);
    }
    return output;
}

} //namespace

ViewKernel::ViewKernel()
    : Kernel()
    , m_inputFile("")
{}


void ViewKernel::validateSwitches()
{
    if (m_inputFile == "")
    {
        throw app_usage_error("--input/-i required");
    }
}


void ViewKernel::addSwitches()
{
    po::options_description* file_options = new po::options_description("file options");

    file_options->add_options()
    ("input,i", po::value<std::string>(&m_inputFile)->default_value(""), "input file name")
    ("point,p", po::value<std::string >(&m_pointIndexes), "point to dump")
    ;

    addSwitchSet(file_options);

    addPositionalSwitch("input", 1);
}

int ViewKernel::execute()
{
    Options readerOptions;
    readerOptions.add<std::string>("filename", m_inputFile);
    setCommonOptions(readerOptions);

    Stage& readerStage(Kernel::makeReader(m_inputFile));
    readerStage.setOptions(readerOptions);

    PointTable table;
    readerStage.prepare(table);
    PointViewSet viewSetIn = readerStage.execute(table);

    PointViewPtr buf = *viewSetIn.begin();
    if (m_pointIndexes.size())
    {
        PointViewPtr outbuf = buf->makeNew();

        std::vector<uint32_t> points = getListOfPoints(m_pointIndexes);
        for (size_t i = 0; i < points.size(); ++i)
        {
            PointId id = (PointId)points[i];
            if (id < buf->size())
                outbuf->appendPoint(*buf, id);
        }

        visualize(outbuf);
    }
    else
    {
        visualize(buf);
    }
    return 0;
}

} // pdal
