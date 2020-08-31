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
#include "../filters/StatsFilter.hpp"

namespace pdal
{

static StaticPluginInfo const s_info
{
    "readers.text",
    "Text Reader",
    "http://pdal.io/stages/readers.text.html",
    { "txt", "csv" }
};

CREATE_STATIC_STAGE(TextReader, s_info)

std::string TextReader::getName() const { return s_info.name; }

// NOTE: - Forces reading of the entire file.
QuickInfo TextReader::inspect()
{
    QuickInfo qi;
    FixedPointTable t(100);

    StatsFilter f;
    f.setInput(*this);

    f.prepare(t);
    PointLayoutPtr layout = t.layout();
    for (Dimension::Id id : layout->dims())
        qi.m_dimNames.push_back(layout->dimName(id));
    f.execute(t);

    try
    {
        stats::Summary xSummary = f.getStats(Dimension::Id::X);
        qi.m_pointCount = xSummary.count();
        qi.m_bounds.minx = xSummary.minimum();
        qi.m_bounds.maxx = xSummary.maximum();
        stats::Summary ySummary = f.getStats(Dimension::Id::Y);
        qi.m_bounds.miny = ySummary.minimum();
        qi.m_bounds.maxy = ySummary.maximum();
        stats::Summary zSummary = f.getStats(Dimension::Id::Z);
        qi.m_bounds.minz = zSummary.minimum();
        qi.m_bounds.maxz = zSummary.maximum();
        qi.m_valid = true;
    }
    catch (pdal_error&)
    {}
    return qi;
}


// Make sure we have a header line.
void TextReader::checkHeader(const std::string& header)
{
    auto it = std::find_if(header.begin(), header.end(),
        [](char c){ return std::isalpha(c); });

    if (it == header.end())
        log()->get(LogLevel::Warning) << getName() <<
            ": file '" << m_filename <<
            "' doesn't appear to contain a header line." << std::endl;
}


void TextReader::parseHeader(const std::string& header)
{
    // If the first character is a double quote, assume that we have quoted
    // field names.
    if (header[0] == '"')
        parseQuotedHeader(header);
    else
        parseUnquotedHeader(header);
}


void TextReader::parseQuotedHeader(const std::string& header)
{
    // We know there's a double quote at position 0.
    std::string::size_type pos = 1;
    while (true)
    {
        size_t count = Dimension::extractName(header, pos);
        m_dimNames.push_back(header.substr(pos, count));
        pos += count;
        if (header[pos] != '"')
            throwError("Invalid character '" + std::string(1, header[pos]) +
                "' found while parsing quoted header line.");
        pos++; // Skip ending quote.

        // Skip everything other than a double quote.
        count = Utils::extract(header, pos, [](char c){ return c != '"'; });

        // Find a separator.
        if (!m_separatorArg->set())
        {
            std::string sep = header.substr(pos, count);
            Utils::trim(sep);
            if (sep.size() > 1)
                throwError("Found separator longer than a single character.");
            if (sep.size() == 0)
                sep = ' ';
            m_separatorArg->setValue(sep);
        }
        pos += count;
        if (header[pos++] != '"')
            break;
    }
}

void TextReader::parseUnquotedHeader(const std::string& header)
{
    auto isspecial = [](char c)
        { return (!std::isalnum(c)); };

    // If the separator wasn't provided on the command line extract it
    // from the header line.
    if (!m_separatorArg->set())
    {
        // Scan string for some character not a number or letter.
        for (size_t i = 0; i < header.size(); ++i)
            // Parenthesis around special to prevent macro expansion, see #3190
            if ((isspecial)(header[i]))
            {
                m_separator = header[i];
                break;
            }
    }

    if (m_separator != ' ')
        m_dimNames = Utils::split(header, m_separator);
    else
        m_dimNames = Utils::split2(header, m_separator);

    for (auto& s : m_dimNames)
    {
        Utils::trim(s);
        size_t cnt = Dimension::extractName(s, 0);
        if (cnt != s.size())
            throwError("Invalid character '" + std::string(1, s[cnt]) +
                "' in dimension name.");
    }
}


void TextReader::initialize(PointTableRef table)
{
    m_istream = Utils::openFile(m_filename, false);
    if (!m_istream)
        throwError("Unable to open text file '" + m_filename + "'.");

    m_line = 0;
    // Skip lines requested.
    std::string dummy;
    for (size_t i = 0; i < m_skip; ++i)
    {
        std::getline(*m_istream, dummy);
        m_line++;
    }

    std::string header;
    if (m_header.size())
        header = m_header;
    else
    {
        std::getline(*m_istream, header);
        m_line++;
        checkHeader(header);
    }

    try
    {
        parseHeader(header);
    }
    catch( const pdal_error& )
    {
        Utils::closeFile(m_istream);
        throw;
    }
    Utils::closeFile(m_istream);
}


void TextReader::addArgs(ProgramArgs& args)
{
    m_separatorArg = &(args.add("separator", "Separator character that "
        "overrides special character found in header line", m_separator, ' '));
    args.add("header", "Use this string as the header line.", m_header);
    args.add("skip", "Skip this number of lines before attempting to "
        "read the header.", m_skip);
}


void TextReader::addDimensions(PointLayoutPtr layout)
{
    m_dims.clear();
    for (auto name : m_dimNames)
    {
        Utils::trim(name);
        Dimension::Id id = layout->registerOrAssignDim(name,
            Dimension::Type::Double);
        if (Utils::contains(m_dims, id) && id != pdal::Dimension::Id::Unknown)
            throwError("Duplicate dimension '" + name +
                "' detected in input file '" + m_filename + "'.");
        m_dims.push_back(id);
    }
}


void TextReader::ready(PointTableRef table)
{
    m_istream = Utils::openFile(m_filename, false);
    if (!m_istream)
        throwError("Unable to open text file '" + m_filename + "'.");

    std::string dummy;
    for (size_t i = 0; i < m_line; ++i)
	std::getline(*m_istream, dummy);
}


point_count_t TextReader::read(PointViewPtr view, point_count_t numPts)
{
    PointId idx = view->size();
    point_count_t cnt = 0;
    PointRef point(*view, idx);
    while (cnt < numPts)
    {
        point.setPointId(idx);
        if (!processOne(point))
            break;
        cnt++;
        idx++;
    }
    return cnt;
}


bool TextReader::processOne(PointRef& point)
{
    if (!fillFields())
        return false;

    double d;
    for (size_t i = 0; i < m_fields.size(); ++i)
    {
        if (!Utils::fromString(m_fields[i], d))
        {
            log()->get(LogLevel::Error) << "Can't convert "
                "field '" << m_fields[i] << "' to numeric value on line " <<
                m_line << " in '" << m_filename << "'.  Setting to 0." <<
                std::endl;
            d = 0;
        }
        point.setField(m_dims[i], d);
    }
    return true;
}


bool TextReader::fillFields()
{
    while (true)
    {
        if (!m_istream->good())
            return false;

        std::string buf;

        std::getline(*m_istream, buf);
        m_line++;
        if (buf.empty())
            continue;
        if (m_separator != ' ')
        {
            Utils::remove(buf, ' ');
            m_fields = Utils::split(buf, m_separator);
        }
        else
            m_fields = Utils::split2(buf, m_separator);
        if (m_fields.size() != m_dims.size())
        {
            log()->get(LogLevel::Error) << "Line " << m_line <<
                " in '" << m_filename << "' contains " << m_fields.size() <<
                " fields when " << m_dims.size() << " were expected.  "
                "Ignoring." << std::endl;
            continue;
        }
        return true;
    }
}


void TextReader::done(PointTableRef table)
{
    Utils::closeFile(m_istream);
}


} // namespace pdal

