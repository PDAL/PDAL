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
    , m_dims()
    , m_numPoints()
    , m_wsClient()
{ }

GreyhoundReader::~GreyhoundReader()
{
    exchanges::Destroy exchange(m_sessionId);
    m_wsClient.exchange(exchange);
}

Options GreyhoundReader::getDefaultOptions()
{
    Options options;
    return options;
}

std::vector<Dimension> GreyhoundReader::getDefaultDimensions()
{
    std::vector<Dimension> output;

    // TODO

    return output;
}

StageSequentialIterator* GreyhoundReader::createSequentialIterator() const
{
    return new iterators::sequential::Iterator(
            const_cast<WebSocketClient&>(m_wsClient), // TODO
            m_sessionId,
            m_numPoints,
            m_schema.getByteSize());
}

void GreyhoundReader::processOptions(const Options& options)
{
    m_uri = options.getValueOrThrow<std::string>("uri");
    m_pipelineId = options.getValueOrThrow<std::string>("pipelineId");

    m_wsClient.initialize(m_uri);
}

void GreyhoundReader::buildSchema(Schema* schema)
{
    Json::Value jsonResponse;
    Json::Reader jsonReader;

    // Create session.
    {
        exchanges::CreateSession exchange(m_pipelineId);
        m_wsClient.exchange(exchange);
        m_sessionId = exchange.getSession();
    }

    // Get schema
    {
        exchanges::GetSchema exchange(m_sessionId);
        m_wsClient.exchange(exchange);
        m_schema = Schema::from_xml(exchange.schema());

        schema = &m_schema;
    }
}

void GreyhoundReader::ready(PointContext ctx)
{
    Json::Value jsonResponse;
    Json::Reader jsonReader;

    // Get number of points.
    {
        exchanges::GetNumPoints exchange(m_sessionId);
        m_wsClient.exchange(exchange);
        m_numPoints = exchange.count();
    }
}

namespace iterators
{

sequential::Iterator::Iterator(
        WebSocketClient& wsClient,
        std::string sessionId,
        point_count_t numPoints,
        schema::size_type byteSize)
    : m_wsClient(wsClient)
    , m_sessionId(sessionId)
    , m_numPoints(numPoints)
    , m_byteSize(byteSize)
{ }

point_count_t sequential::Iterator::readImpl(
        PointBuffer& pointBuffer,
        point_count_t count)
{
    Json::Value jsonResponse;
    Json::Reader jsonReader;

    exchanges::Read exchange(m_sessionId, m_index, count);
    m_wsClient.exchange(exchange);
    std::vector<const std::string*> data(exchange.data());
    point_count_t numRead(exchange.numRead());

    std::vector<char>& rawBuffer(
            pointBuffer.context().rawPtBuf()->getBuffer());

    std::cout << "PB: " << pointBuffer.size() << std::endl;

    for (std::size_t i(0); i < data.size(); ++i)
    {
        std::memcpy(
                rawBuffer.data() + (m_index * m_byteSize),
                data[i]->data(),
                data[i]->size());

        m_index += data[i]->size();
    }

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

