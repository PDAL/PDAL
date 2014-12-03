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

#include <condition_variable>
#include <mutex>

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

#include <json/json.h>

namespace pdal
{

typedef websocketpp::client<websocketpp::config::asio_client> asioClient;
typedef websocketpp::config::asio_client::message_type::ptr message_ptr;

class WebSocketExchange
{
public:
    const Json::Value& req() const { return m_req; }
    const std::vector<message_ptr>& res() const { return m_res; }
    void addResponse(message_ptr message)
    {
        m_res.push_back(message);
        handleRx(message);
    }

    virtual bool done() const { return true; }
    virtual bool check() { return true; }

protected:
    WebSocketExchange() : m_req(), m_res() { }
    virtual void handleRx(const message_ptr) { }

    Json::Value m_req;

private:
    std::vector<message_ptr> m_res;
};

class WebSocketClient
{
public:
    WebSocketClient(bool enableLogging = false);
    WebSocketClient(const std::string& uri, bool enableLogging = false);

    void initialize(const std::string& uri);
    void exchange(WebSocketExchange& exchange);

private:
    std::string m_uri;
    asioClient m_client;
    Json::Reader m_jsonReader;

    std::condition_variable m_cv;
    std::mutex m_mutex;

    bool m_initialized;
};

} // namespace pdal

