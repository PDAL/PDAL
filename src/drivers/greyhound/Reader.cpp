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
    Json::Value jsonResponse;
    Json::Reader jsonReader;
    exchanges::Destroy exchange(m_pipelineId);
    m_wsClient.exchange(exchange);
    jsonReader.parse(exchange.res().at(0)->get_payload(), jsonResponse);

    if (jsonResponse["status"] != 1)
    {
        throw new pdal_error("Could not destroy Greyhound session");
    }
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
}

void GreyhoundReader::buildSchema(Schema* schema)
{
    // TODO
    std::vector<Dimension> dims = getDefaultDimensions();
    for (auto d = dims.begin(); d != dims.end(); ++d)
        m_dims.push_back(schema->appendDimension(*d));
}

void GreyhoundReader::ready(PointContext ctx)
{
    Json::Value jsonResponse;
    Json::Reader jsonReader;
    m_wsClient.initialize(m_uri);

    // Create session.
    {
        exchanges::CreateSession exchange(m_pipelineId);
        m_wsClient.exchange(exchange);
        jsonReader.parse(exchange.res().at(0)->get_payload(), jsonResponse);

        if (jsonResponse["status"] == 1)
        {
            m_sessionId = jsonResponse["session"].asString();
        }
        else
        {
            throw new pdal_error("Could not create Greyhound session");
        }
    }

    // Get number of points.
    {
        exchanges::GetNumPoints exchange(m_sessionId);
        m_wsClient.exchange(exchange);
        jsonReader.parse(exchange.res().at(0)->get_payload(), jsonResponse);

        if (jsonResponse["status"] == 1)
        {
            m_numPoints = jsonResponse["count"].asInt();
        }
        else
        {
            throw new pdal_error("Could not get number of points");
        }
    }

    // Get schema
    {
        exchanges::GetSchema exchange(m_sessionId);
        m_wsClient.exchange(exchange);
        jsonReader.parse(exchange.res().at(0)->get_payload(), jsonResponse);

        if (jsonResponse["status"] == 1)
        {
            m_schema = Schema::from_xml(jsonResponse["schema"].asString());
        }
        else
        {
            throw new pdal_error("Could not get schema");
        }
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
        PointBuffer& data,
        point_count_t count)
{
    Json::Value jsonResponse;
    Json::Reader jsonReader;
    point_count_t numRead(0);

    exchanges::Read exchange(m_sessionId, m_index, count);
    m_wsClient.exchange(exchange);
    jsonReader.parse(exchange.res().at(0)->get_payload(), jsonResponse);

    // TODO Should probably tally as we copy instead of trusting this.
    numRead = jsonResponse["pointsRead"].asInt();

    std::vector<char>& rawBuffer(data.context().rawPtBuf()->getBuffer());

    std::cout << "PB: " << data.size() << std::endl;

    // TODO Optimize
    for (std::size_t i = 1; i < exchange.res().size(); ++i)
    {
        std::memcpy(
                rawBuffer.data() + (m_index * m_byteSize),
                exchange.res().at(i)->get_payload().data(),
                exchange.res().at(i)->get_payload().size());

        m_index += exchange.res().at(i)->get_payload().size() / m_byteSize;
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

