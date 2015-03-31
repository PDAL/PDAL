/******************************************************************************
* Copyright (c) 2011, Howard Butler, hobu.inc@gmail.com
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

#include "TerrasolidReader.hpp"

#include <pdal/PointView.hpp>
#include <pdal/util/Extractor.hpp>

#include <map>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "readers.terrasolid",
    "TerraSolid Reader",
    "http://pdal.io/stages/readers.terrasolid.html" );

CREATE_STATIC_PLUGIN(1, 0, TerrasolidReader, Reader, s_info)

std::string TerrasolidReader::getName() const { return s_info.name; }

void TerrasolidReader::initialize()
{
    ILeStream stream(m_filename);

    TerraSolidHeaderPtr h(new TerraSolidHeader);
    m_header.swap(h);

    stream >> m_header->HdrSize >> m_header->HdrVersion >> m_header->RecogVal;
    stream.get(m_header->RecogStr, 4);
    stream >> m_header->PntCnt >> m_header->Units >> m_header->OrgX >>
        m_header->OrgY >> m_header->OrgZ >> m_header->Time >> m_header->Color;

    if (m_header->RecogVal != 970401)
        throw terrasolid_error("Header identifier was not '970401', is this "
            "a TerraSolid .bin file?");

    m_haveColor = (m_header->Color != 0);
    m_haveTime = (m_header->Time != 0);
    m_format = static_cast<TERRASOLID_Format_Type>(m_header->HdrVersion);

    if ((m_format != TERRASOLID_Format_1) && (m_format != TERRASOLID_Format_2))
    {
        std::ostringstream oss;
        oss << "Version was '" << m_format << "', not '" <<
            TERRASOLID_Format_1 << "' or '" << TERRASOLID_Format_2 << "'";
        throw terrasolid_error(oss.str());
    }

    log()->get(LogLevel::Debug) << "TerraSolid Reader::initialize format: " <<
        m_format << std::endl;
    log()->get(LogLevel::Debug) << "OrgX: " << m_header->OrgX << std::endl;
    log()->get(LogLevel::Debug) << "OrgY: " << m_header->OrgY << std::endl;
    log()->get(LogLevel::Debug) << "OrgZ: " << m_header->OrgZ << std::endl;
    log()->get(LogLevel::Debug) << "Units: " << m_header->Units << std::endl;
    log()->get(LogLevel::Debug) << "Time: " << m_header->Time << std::endl;
    log()->get(LogLevel::Debug) << "Color: " << m_header->Color << std::endl;
    log()->get(LogLevel::Debug) << "Count: " << m_header->PntCnt << std::endl;
    log()->get(LogLevel::Debug) << "RecogVal: " << m_header->RecogVal <<
        std::endl;
}


void TerrasolidReader::addDimensions(PointLayoutPtr layout)
{
    m_size = 0;
    layout->registerDim(Dimension::Id::Classification);
    layout->registerDim(Dimension::Id::PointSourceId);
    layout->registerDim(Dimension::Id::Intensity);
    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);
    layout->registerDim(Dimension::Id::Z);
    layout->registerDim(Dimension::Id::ReturnNumber);
    layout->registerDim(Dimension::Id::NumberOfReturns);
    if (m_format == TERRASOLID_Format_2)
    {
        layout->registerDim(Dimension::Id::Flag);
        layout->registerDim(Dimension::Id::Mark);
    }
    if (m_format == TERRASOLID_Format_1)
        m_size = 16;
    else if (m_format == TERRASOLID_Format_2)
        m_size = 20;

    if (m_haveTime)
    {
        layout->registerDim(Dimension::Id::OffsetTime);
        m_size += 4;
    }

    if (m_haveColor)
    {
        layout->registerDim(Dimension::Id::Red);
        layout->registerDim(Dimension::Id::Green);
        layout->registerDim(Dimension::Id::Blue);
        layout->registerDim(Dimension::Id::Alpha);
        m_size += 4;
    }
}


Dimension::IdList TerrasolidReader::getDefaultDimensions()
{
    using namespace Dimension;

    IdList dims;

    dims.push_back(Id::Classification);
    dims.push_back(Id::PointSourceId);
    dims.push_back(Id::ReturnNumber);
    dims.push_back(Id::NumberOfReturns);
    dims.push_back(Id::Flag);
    dims.push_back(Id::Mark);
    dims.push_back(Id::Intensity);
    dims.push_back(Id::X);
    dims.push_back(Id::Y);
    dims.push_back(Id::Z);
    dims.push_back(Id::Red);
    dims.push_back(Id::Green);
    dims.push_back(Id::Blue);
    dims.push_back(Id::Alpha);
    dims.push_back(Id::OffsetTime);
    return dims;
}


void TerrasolidReader::ready(PointTableRef)
{
    m_istream.reset(new IStream(m_filename));
    // Skip to the beginning of points.
    m_istream->seek(56);
    m_index = 0;
}


point_count_t TerrasolidReader::read(PointViewPtr view, point_count_t count)
{
    count = std::min(count, getNumPoints() - m_index);

    std::vector<char> buf(m_size * count);
    m_istream->get(buf);
    LeExtractor extractor(buf.data(), buf.size());

    // See https://www.terrasolid.com/download/tscan.pdf
    // This spec is awful, but it's something.
    // The scaling adjustments are different than what we used to do and
    // seem wrong (scaling the offset is odd), but that's what the document
    // says.
    // Also modified the fetch of time/color based on header flag (rather
    // than just not write the data into the buffer).
    PointId nextId = view->size();
    while (!eof())
    {
        if (m_format == TERRASOLID_Format_1)
        {
            uint8_t classification, flight_line, echo_int, x, y, z;

            extractor >> classification >> flight_line >> echo_int >> x >> y >>
                z;

            view->setField(Dimension::Id::Classification, nextId,
                          classification);
            view->setField(Dimension::Id::PointSourceId, nextId, flight_line);
            switch (echo_int)
            {
            case 0: // only echo
                view->setField(Dimension::Id::ReturnNumber, nextId, 1);
                view->setField(Dimension::Id::NumberOfReturns, nextId, 1);
                break;
            case 1: // first of many echos
                view->setField(Dimension::Id::ReturnNumber, nextId, 1);
                break;
            default: // intermediate echo or last of many echos
                break;
            }
            view->setField(Dimension::Id::X, nextId,
                          (x - m_header->OrgX) / m_header->Units);
            view->setField(Dimension::Id::Y, nextId,
                          (y - m_header->OrgY) / m_header->Units);
            view->setField(Dimension::Id::Z, nextId,
                          (z - m_header->OrgZ) / m_header->Units);
        }

        if (m_format == TERRASOLID_Format_2)
        {
            int32_t x, y, z;
            uint8_t classification, echo_int, flag, mark;
            uint16_t flight_line, intensity;

            extractor >> x >> y >> z >> classification >> echo_int >> flag >>
                mark >> flight_line >> intensity;

            view->setField(Dimension::Id::X, nextId,
                          (x - m_header->OrgX) / m_header->Units);
            view->setField(Dimension::Id::Y, nextId,
                          (y - m_header->OrgY) / m_header->Units);
            view->setField(Dimension::Id::Z, nextId,
                          (z - m_header->OrgZ) / m_header->Units);
            view->setField(Dimension::Id::Classification, nextId,
                          classification);
            switch (echo_int)
            {
            case 0: // only echo
                view->setField(Dimension::Id::ReturnNumber, nextId, 1);
                view->setField(Dimension::Id::NumberOfReturns, nextId, 1);
                break;
            case 1: // first of many echos
                view->setField(Dimension::Id::ReturnNumber, nextId, 1);
                break;
            default: // intermediate echo or last of many echos
                break;
            }
            view->setField(Dimension::Id::Flag, nextId, flag);
            view->setField(Dimension::Id::Mark, nextId, mark);
            view->setField(Dimension::Id::PointSourceId, nextId, flight_line);
            view->setField(Dimension::Id::Intensity, nextId, intensity);
        }

        if (m_haveTime)
        {
            uint32_t t;

            extractor >> t;

            if (m_index == 0)
                m_baseTime = t;
            t -= m_baseTime; // Offset from the beginning of the read.
            // instead of GPS week.
            t /= 5; // 5000ths of a second to milliseconds
            view->setField(Dimension::Id::OffsetTime, nextId, t);
        }

        if (m_haveColor)
        {
            uint8_t red, green, blue, alpha;

            extractor >> red >> green >> blue >> alpha;

            view->setField(Dimension::Id::Red, nextId, red);
            view->setField(Dimension::Id::Green, nextId, green);
            view->setField(Dimension::Id::Blue, nextId, blue);
            view->setField(Dimension::Id::Alpha, nextId, alpha);
        }

        if (m_cb)
            m_cb(*view, nextId);

        nextId++;
        m_index++;
    }

    return count;
}


void TerrasolidReader::done(PointTableRef)
{
    m_istream.reset();
}

} // namespace pdal
