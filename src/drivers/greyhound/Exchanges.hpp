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

#include <string>

#include <pdal/WebSocketClient.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/pdal_error.hpp>

#include <pdal/drivers/greyhound/Reader.hpp>

namespace pdal
{
namespace drivers
{
namespace greyhound
{
namespace exchanges
{

class Exchange : public WebSocketExchange
{
protected:
    Exchange(const std::string& type)
    {
        m_req["command"] = type;
    }

    bool checkStatus() const
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
};

class CreateSession : public Exchange
{
public:
    CreateSession(const std::string& pipelineId)
        : Exchange("create")
        , m_session()
    {
        m_req["pipelineId"] = pipelineId;
    }

    virtual bool check()
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

    std::string getSession() const
    {
        return m_session;
    }

private:
    std::string m_session;
};

class GetNumPoints : public Exchange
{
public:
    GetNumPoints(const std::string& sessionId)
        : Exchange("pointsCount")
        , m_count()
    {
        m_req["session"] = sessionId;
    }

    virtual bool check()
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

    std::size_t count() const
    {
        return m_count;
    }

private:
    std::size_t m_count;
};

class GetSchema : public Exchange
{
public:
    GetSchema(const std::string& sessionId)
        : Exchange("schema")
        , m_dimData()
    {
        m_req["session"] = sessionId;
    }

    virtual bool check()
    {
        bool valid(false);

        if (checkStatus() && res().size() == 1)
        {
            Json::Value jsonResponse;
            Json::Reader jsonReader;
            jsonReader.parse(res().at(0)->get_payload(), jsonResponse);

            if (jsonResponse.isMember("schema") &&
                jsonResponse["schema"].isString())
            {
                Json::Value jsonSchema;
                jsonReader.parse(jsonResponse["schema"].asString(), jsonSchema);

                if (jsonSchema.isMember("dimensions") &&
                    jsonSchema["dimensions"].isArray())
                {
                    Json::Value jsonDimArray(jsonSchema["dimensions"]);

                    for (std::size_t i(0); i < jsonDimArray.size(); ++i)
                    {
                        const Json::Value& jsonDim(jsonDimArray[i]);

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
        }

        return valid;
    }

    std::vector<DimData> schema() const
    {
        return m_dimData;
    }

private:
    std::vector<DimData> m_dimData;
};

class Read : public Exchange
{
public:
    Read(const std::string& sessionId, int offset = 0, int count = -1)
        : Exchange("read")
        , m_initialized(false)
        , m_error(false)
        , m_pointsToRead(0)
        , m_numBytes(0)
        , m_numBytesReceived(0)
        , m_data()
    {
        m_req["session"] = sessionId;
        m_req["start"] = offset;
        if (count != -1) m_req["count"] = count;
    }

    virtual bool done() const
    {
        return (m_initialized && m_numBytesReceived >= m_numBytes) || m_error;
    }

    virtual bool check()
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

                valid = true;
            }
        }

        return valid;
    }

    virtual void handleRx(const message_ptr message)
    {
        if (!m_initialized)
        {
            if (check())
            {
                m_initialized = true;
            }
            else
            {
                m_error = true;
            }
        }
        else
        {
            if (message->get_opcode() == websocketpp::frame::opcode::binary)
            {
                m_data.push_back(&message->get_payload());
                m_numBytesReceived += message->get_payload().length();
            }
            else
            {
                m_error = true;
            }
        }
    }

    std::vector<const std::string*> data() const
    {
        return m_data;
    }

    std::size_t numRead() const
    {
        return m_pointsToRead;
    }

private:
    bool m_initialized;
    bool m_error;
    std::size_t m_pointsToRead;
    std::size_t m_numBytes;
    std::size_t m_numBytesReceived;
    std::vector<const std::string*> m_data;
};

class Destroy: public Exchange
{
public:
    Destroy(const std::string& sessionId)
        : Exchange("destroy")
    {
        m_req["session"] = sessionId;
    }

    virtual bool check()
    {
        return checkStatus() && res().size() == 1;
    }
};

} // namespace exchanges
} // namespace greyhound
} // namespace drivers
} // namespace pdal

