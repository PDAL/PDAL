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

#include <pdal/PointBuffer.hpp>
#include <pdal/FileUtils.hpp>

#include <map>

namespace pdal
{

void TerrasolidReader::initialize()
{
    std::istream* stream = FileUtils::openFile(m_filename);

    TerraSolidHeaderPtr h(new TerraSolidHeader);
    m_header.swap(h);
    Utils::read_n(*m_header, *stream, sizeof(TerraSolidHeader));

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

    delete stream;
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


void TerrasolidReader::addDimensions(PointContextRef ctx)
{
    m_size = 0;
    ctx.registerDim(Dimension::Id::Classification);
    ctx.registerDim(Dimension::Id::PointSourceId);
    ctx.registerDim(Dimension::Id::Intensity);
    ctx.registerDim(Dimension::Id::X);
    ctx.registerDim(Dimension::Id::Y);
    ctx.registerDim(Dimension::Id::Z);
    if (m_format == TERRASOLID_Format_2)
    {
        ctx.registerDim(Dimension::Id::ReturnNumber);
        ctx.registerDim(Dimension::Id::Flag);
        ctx.registerDim(Dimension::Id::Mark);
    }
    if (m_format == TERRASOLID_Format_1)
        m_size = 16;
    else if (m_format == TERRASOLID_Format_2)
        m_size = 20;

    if (m_haveTime)
    {
        ctx.registerDim(Dimension::Id::OffsetTime);
        m_size += 4;
    }

    if (m_haveColor)
    {
        ctx.registerDim(Dimension::Id::Red);
        ctx.registerDim(Dimension::Id::Green);
        ctx.registerDim(Dimension::Id::Blue);
        ctx.registerDim(Dimension::Id::Alpha);
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


void TerrasolidReader::ready(PointContextRef ctx)
{
    m_istream = FileUtils::openFile(m_filename);
    // Skip to the beginning of points.
    m_istream->seekg(56);
    m_index = 0;
}


point_count_t TerrasolidReader::read(PointBuffer& data, point_count_t count)
{
    count = std::min(count, getNumPoints() - m_index);

    uint8_t *buf = new uint8_t[m_size * count];
    Utils::read_n(buf, *m_istream, m_size * count);

    //See https://www.terrasolid.com/download/tscan.pdf
    // This spec is awful, but it's something.  
    // The scaling adjustments are different than what we used to do and
    // seem wrong (scaling the offset is odd), but that's what the document
    // says.
    // Also modified the fetch of time/color based on header flag (rather
    // than just not write the data into the buffer).
    PointId nextId = data.size();
    while (!eof())
    {
        uint8_t* p = buf + m_size * m_index;

        if (m_format == TERRASOLID_Format_1)
        {
            uint8_t classification = Utils::read_field<uint8_t>(p);
            data.setField(Dimension::Id::Classification, nextId,
                    classification);

            uint8_t flight_line = Utils::read_field<uint8_t>(p);
            data.setField(Dimension::Id::PointSourceId, nextId,
                flight_line);

            uint16_t echo_int = Utils::read_field<uint16_t>(p);
            data.setField(Dimension::Id::ReturnNumber, nextId, echo_int);

            int32_t x = Utils::read_field<int32_t>(p);
            data.setField(Dimension::Id::X, nextId,
                (x - m_header->OrgX) / m_header->Units);

            int32_t y = Utils::read_field<int32_t>(p);
            data.setField(Dimension::Id::Y, nextId,
                (y - m_header->OrgY) / m_header->Units);

            int32_t z = Utils::read_field<int32_t>(p);
            data.setField(Dimension::Id::Z, nextId,
                (z - m_header->OrgZ) / m_header->Units);
        }

        if (m_format == TERRASOLID_Format_2)
        {
            int32_t x = Utils::read_field<int32_t>(p);
            data.setField(Dimension::Id::X, nextId,
                (x - m_header->OrgX) / m_header->Units);

            int32_t y = Utils::read_field<int32_t>(p);
            data.setField(Dimension::Id::Y, nextId,
                (y - m_header->OrgY) / m_header->Units);

            int32_t z = Utils::read_field<int32_t>(p);
            data.setField(Dimension::Id::Z, nextId,
                (z - m_header->OrgZ) / m_header->Units);

            uint8_t classification = Utils::read_field<uint8_t>(p);
            data.setField(Dimension::Id::Classification, nextId,
                classification);

            uint8_t return_number = Utils::read_field<uint8_t>(p);
            data.setField(Dimension::Id::ReturnNumber, nextId,
                return_number);

            uint8_t flag = Utils::read_field<uint8_t>(p);
            data.setField(Dimension::Id::Flag, nextId, flag);

            uint8_t mark = Utils::read_field<uint8_t>(p);
            data.setField(Dimension::Id::Mark, nextId, mark);

            uint16_t flight_line = Utils::read_field<uint16_t>(p);
            data.setField(Dimension::Id::PointSourceId, nextId,
                flight_line);

            uint16_t intensity = Utils::read_field<uint16_t>(p);
            data.setField(Dimension::Id::Intensity, nextId, intensity);
        }

        if (m_haveTime)
        {
            uint32_t t = Utils::read_field<uint32_t>(p);
            if (m_index == 0)
                m_baseTime = t;
            t -= m_baseTime;    //Offset from the beginning of the read.
                                //instead of GPS week.
            t /= 5;             //5000ths of a second to milliseconds
            data.setField(Dimension::Id::OffsetTime, nextId, t);
        }

        if (m_haveColor)
        {
            uint8_t red = Utils::read_field<uint8_t>(p);
            data.setField(Dimension::Id::Red, nextId, red);

            uint8_t green = Utils::read_field<uint8_t>(p);
            data.setField(Dimension::Id::Green, nextId,  green);

            uint8_t blue = Utils::read_field<uint8_t>(p);
            data.setField(Dimension::Id::Blue, nextId, blue);

            uint8_t alpha = Utils::read_field<uint8_t>(p);
            data.setField(Dimension::Id::Alpha, nextId, alpha);
        }
        nextId++;
    }

    delete[] buf;

    return count;
}


void TerrasolidReader::done(PointContextRef ctx)
{
    FileUtils::closeFile(m_istream);
}

} // namespace pdal
