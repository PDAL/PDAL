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

#include <pdal/WebSocketClient.hpp>

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
};

class CreateSession: public Exchange
{
public:
    CreateSession(const std::string& pipelineId)
        : Exchange("create")
    {
        m_req["pipelineId"] = pipelineId;
    }
};

class GetNumPoints: public Exchange
{
public:
    GetNumPoints(const std::string& sessionId)
        : Exchange("pointsCount")
    {
        m_req["session"] = sessionId;
    }
};

class GetSchema: public Exchange
{
public:
    GetSchema(const std::string& sessionId)
        : Exchange("schema")
    {
        m_req["session"] = sessionId;
    }
};

class Read: public Exchange
{
public:
    Read(const std::string& sessionId, int offset = 0, int count = -1)
        : Exchange("read")
        , m_initialized(false)
        , m_numBytes(0)
        , m_numBytesReceived(0)
    {
        m_req["session"] = sessionId;
        m_req["start"] = offset;
        if (count != -1) m_req["count"] = count;
    }

    virtual bool done() const
    {
        return m_initialized && m_numBytesReceived >= m_numBytes;
    }

    virtual void handleRx(const message_ptr message)
    {
        if (!m_initialized)
        {
            Json::Value jsonValue;
            Json::Reader jsonReader;
            jsonReader.parse(message->get_payload(), jsonValue);
            
            if (jsonValue.isMember("bytesCount"))
            {
                m_numBytes = jsonValue["bytesCount"].asInt();
                m_initialized = true;
            }
            else
            {
                // TODO Handle this.
            }
        }
        else
        {
            if (message->get_opcode() == websocketpp::frame::opcode::binary)
            {
                m_numBytesReceived += message->get_payload().length();
            }
            else
            {
                // TODO Handle this.
            }
        }
    }

private:
    bool m_initialized;
    int m_numBytes;
    int m_numBytesReceived;
};

class Destroy: public Exchange
{
public:
    Destroy(const std::string& sessionId)
        : Exchange("destroy")
    {
        m_req["session"] = sessionId;
    }
};

} // namespace exchanges
} // namespace greyhound
} // namespace drivers
} // namespace pdal

