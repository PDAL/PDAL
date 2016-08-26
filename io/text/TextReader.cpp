/******************************************************************************
* Copyright (c) 2016, Hobu Inc., info@hobu.co
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
*     * Neither the name of Hobu, Inc. nor the
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

#include <pdal/PDALUtils.hpp>
#include <pdal/util/Algorithm.hpp>

#include "TextReader.hpp"

#include <pdal/pdal_macros.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "readers.text",
    "Text Reader",
    "http://pdal.io/stages/readers.text.html" );

CREATE_STATIC_PLUGIN(1, 0, TextReader, Reader, s_info)

std::string TextReader::getName() const { return s_info.name; }

void TextReader::initialize(PointTableRef table)
{
    m_istream = Utils::openFile(m_filename);
    if (!m_istream)
    {
        std::ostringstream oss;
        oss << getName() << ": Unable to open text file '" <<
            m_filename << "'.";
        throw pdal_error(oss.str());
    }

    std::string buf;
    std::getline(*m_istream, buf);

    auto isspecial = [](char c)
        { return (!std::isalnum(c) && c != ' '); };

    // Scan string for some character not a number, space or letter.
    for (size_t i = 0; i < buf.size(); ++i)
        if (isspecial(buf[i]))
        {
            m_separator = buf[i];
            break;
        }

    if (m_separator != ' ')
    {
        Utils::remove(buf, ' ');
        m_dimNames = Utils::split(buf, m_separator);
    }
    else
        m_dimNames = Utils::split2(buf, m_separator);
    for (auto f: m_dimNames)
    {
        log()->get(LogLevel::Error) << "field '" << f << "'" << std::endl;

    }
    Utils::closeFile(m_istream);
}


void TextReader::addDimensions(PointLayoutPtr layout)
{
    for (auto name : m_dimNames)
    {
        Dimension::Id id = layout->registerOrAssignDim(name,
            Dimension::Type::Double);
        m_dims.push_back(id);
    }
}


void TextReader::ready(PointTableRef table)
{
    m_istream = Utils::openFile(m_filename);
    if (!m_istream)
    {
        std::ostringstream oss;
        oss << getName() << ": Unable to open text file '" <<
            m_filename << "'.";
        throw pdal_error(oss.str());
    }

    // Skip header line.
    std::string buf;
    std::getline(*m_istream, buf);
}


point_count_t TextReader::read(PointViewPtr view, point_count_t numPts)
{
    PointId idx = view->size();

    point_count_t cnt = 0;
    size_t line = 1;
    while (m_istream->good() && cnt < numPts)
    {
        std::string buf;
        StringList fields;

        std::getline(*m_istream, buf);
        line++;
        if (buf.empty())
            continue;
        if (m_separator != ' ')
        {
            Utils::remove(buf, ' ');
            fields = Utils::split(buf, m_separator);
        }
        else
            fields = Utils::split2(buf, m_separator);
        if (fields.size() != m_dims.size())
        {
            log()->get(LogLevel::Error) << "Line " << line <<
               " in '" << m_filename << "' contains " << fields.size() <<
               " fields when " << m_dims.size() << " were expected.  "
               "Ignoring." << std::endl;
            continue;
        }

        double d;
        for (size_t i = 0; i < fields.size(); ++i)
        {
            if (!Utils::fromString(fields[i], d))
            {
                log()->get(LogLevel::Error) << "Can't convert "
                    "field '" << fields[i] << "' to numeric value on line " <<
                    line << " in '" << m_filename << "'.  Setting to 0." <<
                    std::endl;
                d = 0;
            }
            view->setField(m_dims[i], idx, d);
        }
        cnt++;
        idx++;
    }
    return cnt;
}


void TextReader::done(PointTableRef table)
{
    Utils::closeFile(m_istream);
}


} // namespace pdal

