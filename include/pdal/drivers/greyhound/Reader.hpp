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

#pragma once

#include <pdal/Reader.hpp>
#include <pdal/ReaderIterator.hpp>
#include <pdal/WebSocketClient.hpp>

namespace pdal
{
namespace drivers
{
namespace greyhound
{

struct DimData
{
    DimData(Dimension::Id::Enum id, Dimension::Type::Enum type)
        : id(id)
        , type(type)
    { }

    const Dimension::Id::Enum id;
    const Dimension::Type::Enum type;
};

class PDAL_DLL GreyhoundReader : public pdal::Reader
{
public:
    SET_STAGE_NAME("drivers.greyhound.reader", "Greyhound Reader")
    SET_STAGE_LINK("http://pdal.io/stages/drivers.greyhound.reader.html")
    SET_STAGE_ENABLED(true)

    GreyhoundReader(const Options& options);
    ~GreyhoundReader();

    virtual StageSequentialIterator* createSequentialIterator() const;

private:
    std::string m_uri;
    std::string m_pipelineId;
    std::string m_sessionId;
    std::vector<DimData> m_dimData;
    point_count_t m_numPoints;
    WebSocketClient m_wsClient;

    virtual void initialize();
    virtual void processOptions(const Options& options);
    virtual void addDimensions(PointContext pointContext);
    virtual void ready(PointContext ctx);
};

namespace iterators
{
namespace sequential
{

class PDAL_DLL Iterator:
    public pdal::ReaderSequentialIterator
{
public:
    Iterator(
            WebSocketClient& wsClient,
            std::string sessionId,
            std::vector<DimData> dimData,
            point_count_t numPoints);

private:
    virtual point_count_t readImpl(
            PointBuffer& pointBuffer,
            point_count_t count);

    virtual point_count_t readBufferImpl(PointBuffer& pointBuffer)
    {
        return readImpl(
                pointBuffer,
                std::numeric_limits<point_count_t>::max());
    }

    boost::uint64_t skipImpl(boost::uint64_t pointsToSkip);
    virtual bool atEndImpl() const;

    point_count_t setPoints(
            PointBuffer& pointBuffer,
            const char* data,
            point_count_t pointsToRead);

    WebSocketClient& m_wsClient;
    std::string m_sessionId;
    std::vector<DimData> m_dimData;
    point_count_t m_numPoints;
    std::size_t m_pointByteSize;
};

} // namespace sequential

} // namespace iterators

} // namespace greyhound
} // namespace drivers
} // namespace pdal

