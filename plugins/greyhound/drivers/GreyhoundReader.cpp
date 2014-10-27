/******************************************************************************
* Copyright (c) 2014, Connor Manning (connor@hobu.co)
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

#include "GreyhoundReader.hpp"

#include "Exchanges.hpp"

CREATE_READER_PLUGIN(greyhound, pdal::drivers::greyhound::GreyhoundReader)

namespace pdal
{
namespace drivers
{
namespace greyhound
{

GreyhoundReader::GreyhoundReader()
    : Reader()
    , m_uri()
    , m_pipelineId()
    , m_sessionId()
    , m_dimData()
    , m_wsClient()
    , m_numPoints(0)
    , m_index(0)
    , m_pointByteSize(0)
{ }

GreyhoundReader::~GreyhoundReader()
{
    // Tell Greyhound we're done using this session.
    exchanges::Destroy destroyExchange(m_sessionId);
    m_wsClient.exchange(destroyExchange);
}

void GreyhoundReader::initialize()
{
    // Create remote PDAL session.
    exchanges::CreateSession createExchange(m_pipelineId);
    m_wsClient.exchange(createExchange);
    m_sessionId = createExchange.getSession();
}

void GreyhoundReader::processOptions(const Options& options)
{
    m_uri = options.getValueOrThrow<std::string>("uri");
    m_pipelineId = options.getValueOrThrow<std::string>("pipelineId");

    m_wsClient.initialize(m_uri);
}

void GreyhoundReader::addDimensions(PointContextRef pointContext)
{
    // Get Greyhound schema.
    exchanges::GetSchema schemaExchange(m_sessionId);
    m_wsClient.exchange(schemaExchange);

    m_dimData = schemaExchange.schema();

    for (auto dim : m_dimData)
    {
        pointContext.registerDim(dim.id, dim.type);
        m_pointByteSize += pdal::Dimension::size(dim.type);
    }
}

void GreyhoundReader::ready(PointContextRef)
{
    // Get number of points.
    exchanges::GetNumPoints numPointsExchange(m_sessionId);
    m_wsClient.exchange(numPointsExchange);
    m_numPoints = numPointsExchange.count();
}

point_count_t GreyhoundReader::read(
        PointBuffer& pointBuffer,
        const point_count_t count)
{
    // Read data.
    exchanges::Read readExchange(m_sessionId, m_index, count);
    m_wsClient.exchange(readExchange);
    std::vector<const std::string*> data(readExchange.data());

    std::size_t leftover(0);
    std::size_t offset(0);
    std::size_t numRead(0);

    for (std::size_t i(0); i < data.size(); ++i)
    {
        if (leftover != 0)
        {
            const std::string stitchedPoint(
                    data[i - 1]->substr(data[i - 1]->size() - leftover) +
                    data[i]->substr(0, offset));

            numRead += setPoints(pointBuffer, stitchedPoint.data(), 1);
        }

        const point_count_t wholePoints(
                ((data[i]->size() - offset) / m_pointByteSize));

        const std::size_t pointBoundedSize(wholePoints * m_pointByteSize);

        numRead +=
                setPoints(pointBuffer, data[i]->data() + offset, wholePoints);

        leftover = (data[i]->size() - offset) - pointBoundedSize;
        offset = m_pointByteSize - leftover;
    }

    m_index += numRead;

    return numRead;
}

bool GreyhoundReader::eof() const
{
    return m_index >= m_numPoints;
}

point_count_t GreyhoundReader::setPoints(
        PointBuffer& pointBuffer,
        const char* data,
        const point_count_t pointsToRead) const
{
    PointId nextId(pointBuffer.size());

    std::size_t dataOffset(0);
    point_count_t numRead(0);

    while (numRead < pointsToRead)
    {
        for (auto dim : m_dimData)
        {
            pointBuffer.setField(dim.id, dim.type, nextId, data + dataOffset);
            dataOffset += Dimension::size(dim.type);
        }

        ++nextId;
        ++numRead;
    }

    return numRead;
}

} // namespace greyhound
} // namespace drivers
} // namespace pdal

