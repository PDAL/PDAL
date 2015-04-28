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

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "readers.greyhound",
    "Greyhound Reader",
    "http://pdal.io/stages/readers.greyhound.html" );

CREATE_SHARED_PLUGIN(1, 0, GreyhoundReader, Reader, s_info)

std::string GreyhoundReader::getName() const { return s_info.name; }

GreyhoundReader::GreyhoundReader()
    : Reader()
    , m_url()
    , m_pipelineId()
    , m_sessionId()
    , m_wsClient()
    , m_numPoints(0)
    , m_index(0)
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
    m_url = options.getValueOrThrow<std::string>("url");
    m_pipelineId = options.getValueOrThrow<std::string>("pipelineId");

    m_wsClient.initialize(m_url);
}

void GreyhoundReader::addDimensions(PointLayoutPtr layout)
{
    // Get Greyhound schema.
    exchanges::GetSchema schemaExchange(m_sessionId);
    m_wsClient.exchange(schemaExchange);

    std::vector<exchanges::DimData> dimData = schemaExchange.schema();

    for (const auto& dim : dimData)
    {
        layout->registerDim(dim.id, dim.type);
    }

    m_layout = layout;
}

void GreyhoundReader::ready(PointTableRef)
{
    // Get number of points.
    exchanges::GetNumPoints numPointsExchange(m_sessionId);
    m_wsClient.exchange(numPointsExchange);
    m_numPoints = numPointsExchange.count();
}

point_count_t GreyhoundReader::read(
        PointViewPtr view,
        const point_count_t count)
{
    // Read data.
#ifdef PDAL_HAVE_LAZPERF
    exchanges::ReadCompressed readExchange(
#else
    exchanges::ReadUncompressed readExchange(
#endif
            view,
            m_layout,
            m_sessionId,
            m_index,
            count);

    m_wsClient.exchange(readExchange);
    return readExchange.numRead();
}

bool GreyhoundReader::eof() const
{
    return m_index >= m_numPoints;
}

} // namespace pdal

