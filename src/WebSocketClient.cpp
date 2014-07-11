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

#include <thread>

#include <pdal/WebSocketClient.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

namespace pdal
{

WebSocketClient::WebSocketClient(bool enableLogging)
    : m_uri()
    , m_client()
    , m_jsonReader()
    , m_cv()
    , m_mutex()
    , m_initialized(false)
{
    if (!enableLogging)
    {
        m_client.clear_access_channels(websocketpp::log::alevel::all);
        m_client.clear_error_channels(websocketpp::log::elevel::all);
    }

    m_client.init_asio();
}

WebSocketClient::WebSocketClient(const std::string& uri, bool enableLogging)
    : m_uri(uri)
    , m_client()
    , m_jsonReader()
    , m_cv()
    , m_mutex()
    , m_initialized(true)
{
    if (!enableLogging)
    {
        m_client.clear_access_channels(websocketpp::log::alevel::all);
        m_client.clear_error_channels(websocketpp::log::elevel::all);
    }

    m_client.init_asio();
}

void WebSocketClient::initialize(const std::string& uri)
{
    m_uri = uri;
    m_initialized = true;
}

void WebSocketClient::exchange(WebSocketExchange& exchange)
{
    if (!m_initialized) return;

    bool gotRes(false);

    std::cout << "Exchanging..." << std::endl;
    std::cout << exchange.req().toStyledString() << std::endl;

    std::thread t([this, &exchange, &gotRes]()
    {
        m_client.set_open_handler(
            [this, &exchange](websocketpp::connection_hdl hdl)
        {
            std::cout << "In open handler" << std::endl;
            m_client.send(
                    hdl,
                    exchange.req().toStyledString(),
                    websocketpp::frame::opcode::text);
        });

        m_client.set_message_handler(
                [this, &exchange, &gotRes](
                    websocketpp::connection_hdl,
                    message_ptr msg)
        {
            std::cout << "In msg handler" << std::endl;
            exchange.addResponse(msg);

            if (exchange.done())
            {
                std::unique_lock<std::mutex> lock(this->m_mutex);
                gotRes = true;
                lock.unlock();
                this->m_cv.notify_all();
            }
        });

        websocketpp::lib::error_code ec;
        m_client.reset();
        m_client.connect(m_client.get_connection(m_uri, ec));
        m_client.run();
    });
    t.detach();

    std::cout << "Blocking return" << std::endl;
    std::unique_lock<std::mutex> lock(m_mutex);
    m_cv.wait(lock, [&gotRes]()->bool { return gotRes; });

    m_client.stop();
}

// TODO Temp hack
std::string WebSocketClient::exchangeDouble(const Json::Value& req)
{
    std::string res;

    if (!m_initialized)
    {
        return res;
    }

    bool gotRes(false);

    std::cout << "Exchanging..." << std::endl;
    std::cout << req.toStyledString() << std::endl;

    std::thread t([this, &req, &res, &gotRes]()
    {
        m_client.set_open_handler(
            [this, &req](websocketpp::connection_hdl hdl) {
            std::cout << "In double open handler" << std::endl;
            m_client.send(
                    hdl,
                    req.toStyledString(),
                    websocketpp::frame::opcode::text);
        });

        m_client.set_message_handler(
                [this, &res, &gotRes](
                    websocketpp::connection_hdl,
                    message_ptr msg) {
                /*
            std::unique_lock<std::mutex> lock(this->m_mutex);
            //this->m_jsonReader.parse(msg->get_payload(), res);
            gotRes = true;
            lock.unlock();
            this->m_cv.notify_all();
            */




            std::cout << "In first msg handler" << std::endl;

            m_client.set_message_handler(
                    [this, &res, &gotRes](
                        websocketpp::connection_hdl,
                        message_ptr msg) {
                std::cout << "In second msg handler" << std::endl;

                std::unique_lock<std::mutex> lock(this->m_mutex);
                if (msg->get_opcode() == websocketpp::frame::opcode::binary)
                    res = msg->get_payload();
                gotRes = true;
                lock.unlock();
                this->m_cv.notify_all();
            });
        });

        websocketpp::lib::error_code ec;
        m_client.reset();
        m_client.connect(m_client.get_connection(m_uri, ec));
        m_client.run();
    });
    t.detach();

    std::unique_lock<std::mutex> lock(m_mutex);
    m_cv.wait(lock, [&gotRes]()->bool { return gotRes; });

    m_client.stop();

    return res;
}

} // namespace pdal

