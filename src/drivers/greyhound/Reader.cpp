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

#include <pdal/drivers/greyhound/Reader.hpp>
#include "Exchanges.hpp"

namespace pdal
{
namespace drivers
{
namespace greyhound
{

GreyhoundReader::GreyhoundReader(const Options& options)
    : Reader(options)
    , m_uri()
    , m_pipelineId()
    , m_sessionId()
    , m_numPoints()
    , m_wsClient()
{ }

GreyhoundReader::~GreyhoundReader()
{
    exchanges::Destroy destroyExchange(m_sessionId);
    m_wsClient.exchange(destroyExchange);
}

StageSequentialIterator* GreyhoundReader::createSequentialIterator() const
{
    return new iterators::sequential::Iterator(
            const_cast<WebSocketClient&>(m_wsClient),
            m_sessionId,
            m_numPoints,
            m_schema);
}

void GreyhoundReader::processOptions(const Options& options)
{
    m_uri = options.getValueOrThrow<std::string>("uri");
    m_pipelineId = options.getValueOrThrow<std::string>("pipelineId");

    m_wsClient.initialize(m_uri);
}

void GreyhoundReader::buildSchema(Schema* schema)
{
    // Create session.
    exchanges::CreateSession createExchange(m_pipelineId);
    m_wsClient.exchange(createExchange);
    m_sessionId = createExchange.getSession();

    // Get schema.
    exchanges::GetSchema schemaExchange(m_sessionId);
    m_wsClient.exchange(schemaExchange);
    m_schema = Schema::from_xml(schemaExchange.schema());

    const pdal::schema::index_by_index& idx(
            m_schema.getDimensions().get<pdal::schema::index>());

    // Populate passed-in schema.
    for (boost::uint32_t d = 0; d < idx.size(); ++d)
    {
        schema->appendDimension(idx[d]);
    }
}

void GreyhoundReader::ready(PointContext)
{
    // Get number of points.
    exchanges::GetNumPoints numPointsExchange(m_sessionId);
    m_wsClient.exchange(numPointsExchange);
    m_numPoints = numPointsExchange.count();
}

namespace iterators
{

sequential::Iterator::Iterator(
        WebSocketClient& wsClient,
        std::string sessionId,
        point_count_t numPoints,
        Schema schema)
    : m_wsClient(wsClient)
    , m_sessionId(sessionId)
    , m_numPoints(numPoints)
    , m_schema(schema)
{ }

point_count_t sequential::Iterator::readImpl(
        PointBuffer& pointBuffer,
        point_count_t count)
{
    // Read data.
    exchanges::Read readExchange(m_sessionId, m_index, count);
    m_wsClient.exchange(readExchange);
    std::vector<const std::string*> data(readExchange.data());

    std::size_t leftover(0);
    std::size_t offset(0);
    const std::size_t schemaByteSize(m_schema.getByteSize());

    std::size_t numRead(0);

    for (std::size_t i(0); i < data.size(); ++i)
    {
        if (leftover != 0)
        {
            std::string stitchedPoint = 
                data[i - 1]->substr(data[i - 1]->size() - leftover) +
                data[i]->substr(0, offset);

            pointBuffer.appendRaw(stitchedPoint.data(), 1);
            ++numRead;
        }

        const point_count_t wholePoints(
                ((data[i]->size() - offset) / schemaByteSize));
        const std::size_t pointBoundedSize(
            wholePoints * schemaByteSize);

        pointBuffer.appendRaw(data[i]->data() + offset, pointBoundedSize);

        numRead += wholePoints;
        leftover = (data[i]->size() - offset) - pointBoundedSize;
        offset = schemaByteSize - leftover;
    }

    m_index += numRead;

    return numRead;
}

boost::uint64_t sequential::Iterator::skipImpl(
        const boost::uint64_t pointsToSkip)
{
    const boost::uint64_t skipped(
            std::min<point_count_t>(pointsToSkip, m_numPoints - m_index));
    m_index = std::min<point_count_t>(m_index + pointsToSkip, m_numPoints);
    return skipped;
}

bool sequential::Iterator::atEndImpl() const
{
    return m_index >= m_numPoints;
}

} // namespace iterators

} // namespace greyhound
} // namespace drivers
} // namespace pdal

