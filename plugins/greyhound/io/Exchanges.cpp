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

#include "Exchanges.hpp"

namespace pdal
{
namespace exchanges
{

// Base Greyhound exchange

Exchange::Exchange(const std::string& command)
{
    m_req["command"] = command;
}

bool Exchange::checkStatus() const
{
    bool valid(false);

    if (res().size())
    {
        Json::Value jsonResponse;
        Json::Reader jsonReader;
        jsonReader.parse(res().at(0)->get_payload(), jsonResponse);

        if (jsonResponse.isMember("status") &&
            jsonResponse["status"].isIntegral() &&
            jsonResponse["status"].asInt() == 1)
        {
            valid = true;
        }
    }

    return valid;
}



// CreateSession exchange

CreateSession::CreateSession(const std::string& pipelineId)
    : Exchange("create")
    , m_session()
{
    m_req["pipelineId"] = pipelineId;
}

bool CreateSession::check()
{
    bool valid(false);

    if (checkStatus() && res().size() == 1)
    {
        Json::Value jsonResponse;
        Json::Reader jsonReader;
        jsonReader.parse(res().at(0)->get_payload(), jsonResponse);

        if (jsonResponse.isMember("session") &&
            jsonResponse["session"].isString())
        {
            m_session = jsonResponse["session"].asString();
            valid = true;
        }
    }

    return valid;
}

std::string CreateSession::getSession() const
{
    return m_session;
}



// GetNumPoints exchange

GetNumPoints::GetNumPoints(const std::string& sessionId)
    : Exchange("pointsCount")
    , m_count()
{
    m_req["session"] = sessionId;
}

bool GetNumPoints::check()
{
    bool valid(false);

    if (checkStatus() && res().size() == 1)
    {
        Json::Value jsonResponse;
        Json::Reader jsonReader;
        jsonReader.parse(res().at(0)->get_payload(), jsonResponse);

        if (jsonResponse.isMember("count") &&
            jsonResponse["count"].isIntegral())
        {
            m_count = jsonResponse["count"].asUInt();
            valid = true;
        }
    }

    return valid;
}

std::size_t GetNumPoints::count() const
{
    return m_count;
}



// GetSchema exchange

GetSchema::GetSchema(const std::string& sessionId)
    : Exchange("schema")
    , m_dimData()
{
    m_req["session"] = sessionId;
}

bool GetSchema::check()
{
    bool valid(false);

    if (checkStatus() && res().size() == 1)
    {
        Json::Value jsonResponse;
        Json::Reader jsonReader;
        jsonReader.parse(res().at(0)->get_payload(), jsonResponse);

        if (jsonResponse.isMember("schema") &&
            jsonResponse["schema"].isArray())
        {
            Json::Value jsonDimArray(jsonResponse["schema"]);

            for (std::size_t i(0); i < jsonDimArray.size(); ++i)
            {
                const Json::Value& jsonDim(
                        jsonDimArray[static_cast<Json::ArrayIndex>(i)]);

                const Dimension::Id::Enum id(
                        Dimension::id(jsonDim["name"].asString()));

                const Dimension::Type::Enum type(
                    static_cast<Dimension::Type::Enum>(
                        static_cast<int>(Dimension::fromName(
                            jsonDim["type"].asString())) |
                        std::stoi(jsonDim["size"].asString())));

                m_dimData.push_back(DimData(id, type));
            }

            valid = true;
        }
    }

    return valid;
}

std::vector<DimData> GetSchema::schema() const
{
    return m_dimData;
}



// Read exchange

Read::Read(
        PointViewPtr view,
        const PointLayoutPtr layout,
        const std::string& sessionId,
        bool compress,
        int offset,
        int count)
    : Exchange("read")
    , m_view(view)
    , m_layout(layout)
    , m_initialized(false)
    , m_error(false)
    , m_pointsToRead(0)
    , m_numBytes(0)
    , m_numBytesReceived(0)
    , m_data()
{
    m_req["session"] = sessionId;
    m_req["compress"] = compress;
    m_req["start"] = offset;
    if (count != -1) m_req["count"] = count;
}

bool Read::check()
{
    bool valid(false);

    if (!m_error && checkStatus() && res().size() >= 1)
    {
        Json::Value jsonResponse;
        Json::Reader jsonReader;
        jsonReader.parse(res().at(0)->get_payload(), jsonResponse);

        if (jsonResponse.isMember("numPoints") &&
            jsonResponse["numPoints"].isIntegral() &&
            jsonResponse.isMember("numBytes") &&
            jsonResponse["numBytes"].isIntegral())
        {
            m_pointsToRead =
                std::max<int>(jsonResponse["numPoints"].asInt(), 0);
            m_numBytes =
                std::max<int>(jsonResponse["numBytes"].asInt(), 0);

            if (m_pointsToRead * m_layout->pointSize() != m_numBytes)
            {
                valid = false;
                m_error = true;
            }
            else
            {
                valid = true;
            }
        }
    }

    return valid;
}

std::size_t Read::numRead() const
{
    return m_pointsToRead;
}

ReadUncompressed::ReadUncompressed(
        PointViewPtr view,
        const PointLayoutPtr layout,
        const std::string& sessionId,
        int offset,
        int count)
    : Read(view, layout, sessionId, false, offset, count)
{ }

bool ReadUncompressed::done()
{
    return (m_initialized && m_numBytesReceived >= m_numBytes) || m_error;
}

void ReadUncompressed::handleRx(const message_ptr message)
{
    if (!m_initialized)
    {
        m_initialized = check();
        if (!m_initialized) m_error = true;
    }
    else
    {
        if (message->get_opcode() == websocketpp::frame::opcode::binary)
        {
            const std::string& bytes(message->get_payload());
            const std::size_t rawNumBytes(bytes.size());
            const std::size_t stride(m_layout->pointSize());

            m_data.insert(m_data.end(), bytes.begin(), bytes.end());

            const std::size_t wholePoints(m_data.size() / stride);

            PointId nextId(m_view->size());
            const PointId doneId(nextId + wholePoints);

            const char* pos(m_data.data());

            while (nextId < doneId)
            {
                for (const auto& dim : m_layout->dims())
                {
                    m_view->setField(
                            dim,
                            m_layout->dimType(dim),
                            nextId,
                            pos);

                    pos += m_layout->dimSize(dim);
                }

                ++nextId;
            }

            m_numBytesReceived += rawNumBytes;
            m_data.assign(
                    m_data.begin() + wholePoints * stride,
                    m_data.end());
        }
        else
        {
            m_error = true;
        }
    }
}

#ifdef PDAL_HAVE_LAZPERF
ReadCompressed::ReadCompressed(
        PointViewPtr view,
        const PointLayoutPtr layout,
        const std::string& sessionId,
        int offset,
        int count)
    : Read(view, layout, sessionId, true, offset, count)
    , m_decompressionThread()
    , m_compressionStream()
    , m_decompressor(m_compressionStream, layout->dimTypes())
    , m_done(false)
    , m_doneCv()
    , m_doneMutex()
    , m_mutex()
{ }

bool ReadCompressed::done()
{
    std::unique_lock<std::mutex> lock(m_doneMutex);
    m_doneCv.wait(lock, [this]()->bool { return m_done; });
    return true;
}

void ReadCompressed::handleRx(const message_ptr message)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_initialized)
    {
        m_initialized = check();
        if (!m_initialized) m_error = true;

        m_data.resize(m_numBytes);

        m_decompressionThread = std::thread([this]()->void {
            m_decompressor.decompress(m_data.data(), m_data.size());

            const char* pos(m_data.data());
            for (PointId i(0); i < m_pointsToRead; ++i)
            {
                for (const auto& dim : m_layout->dims())
                {
                    m_view->setField(
                            dim,
                            m_layout->dimType(dim),
                            i,
                            pos);

                    pos += m_layout->dimSize(dim);
                }
            }

            m_done = true;
            m_doneCv.notify_all();
        });

        m_decompressionThread.detach();
    }
    else
    {
        if (message->get_opcode() == websocketpp::frame::opcode::binary)
        {
            const std::string& bytes(message->get_payload());

            m_compressionStream.putBytes(
                    reinterpret_cast<const uint8_t*>(bytes.data()),
                    bytes.size());
        }
        else
        {
            m_error = true;
        }
    }
}
#endif


// Destroy exchange

Destroy::Destroy(const std::string& sessionId)
    : Exchange("destroy")
{
    m_req["session"] = sessionId;
}

bool Destroy::check()
{
    return checkStatus() && res().size() == 1;
}

} // namespace exchanges
} // namespace pdal

