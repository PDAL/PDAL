/******************************************************************************
 * Copyright (c) 2018, Connor Manning
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
 *     * Neither the name of the Martin Isenburg or Iowa Department
 *       of Natural Resources nor the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
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

#include "Connector.hpp"

#include <pdal/pdal_types.hpp>
#include <pdal/pdal_config.hpp>
#include <curl/curl.h>
#include <pdal/util/FileUtils.hpp>





namespace pdal
{
namespace connector
{

std::string getUserAgent()
{
    std::stringstream output;

    output  << "pdal ("
            << pdal::Config::versionMajor() << "."
            << pdal::Config::versionMinor() << "."
            << pdal::Config::versionPatch() << ") "
            << " / curl (" << std::string (curl_version()) << ")";
    return output.str();
}

Connector::Connector()
    : m_arbiter(new arbiter::Arbiter()),
      m_httpDriver(new arbiter::drivers::Http( m_arbiter->httpPool()))
{
    if (m_headers.find("User-Agent") == m_headers.end())
    {
        m_headers.insert( std::make_pair("User-Agent", getUserAgent()));
    }
}

Connector::Connector(const StringMap& headers, const StringMap& query) :
    m_arbiter(new arbiter::Arbiter),
    m_headers(headers),
    m_query(query),
    m_httpDriver(new arbiter::drivers::Http( m_arbiter->httpPool()))
{
    if (m_headers.find("User-Agent") == m_headers.end())
    {
        m_headers.insert( std::make_pair("User-Agent", getUserAgent()));
    }
}

Connector::Connector(const std::string& filename, const StringMap& headers,
        const StringMap& query) :
    m_arbiter(new arbiter::Arbiter),
    m_headers(headers),
    m_query(query),
    m_filename(filename)
{
    if (m_headers.find("User-Agent") == m_headers.end())
    {
        m_headers.insert( std::make_pair("User-Agent", getUserAgent()));
    }
}


std::string Connector::get(const std::string& path) const
{
    if (m_arbiter->isLocal(path))
        return m_arbiter->get(path);
    else
        return m_arbiter->get(path, m_headers, m_query);
}

NL::json Connector::getJson(const std::string& path) const
{
    try
    {
        return NL::json::parse(get(path));
    }
    catch (NL::json::parse_error& err)
    {
        throw pdal_error("File '" + path + "' contained invalid JSON: " +
            err.what());
    }
}

std::vector<char> Connector::getBinary(const std::string& path) const
{
    if (m_arbiter->isLocal(path))
        return m_arbiter->getBinary(path);
    else
        return m_arbiter->getBinary(path, m_headers, m_query);
}


arbiter::LocalHandle Connector::getLocalHandle(const std::string& path) const
{
    if (m_arbiter->isLocal(path))
        return m_arbiter->getLocalHandle(path);
    else
        return m_arbiter->getLocalHandle(path, m_headers, m_query);
}

void Connector::put(const std::string& path, const std::vector<char>& buf) const
{
    if (m_arbiter->isLocal(path))
        m_arbiter->put(path, buf);
    else
        m_arbiter->put(path, buf, m_headers, m_query);
}

void Connector::put(const std::string& path, const std::string& data) const
{
    if (m_arbiter->isLocal(path))
        m_arbiter->put(path, data);
    else
        m_arbiter->put(path, data, m_headers, m_query);
}

void Connector::makeDir(const std::string& path) const
{
    if (m_arbiter->isLocal(path))
        arbiter::mkdirp(path);
}


StringMap Connector::headRequest(const std::string& path) const
{
    arbiter::http::Response r = m_httpDriver->internalHead(path);
    return r.headers();
}

std::vector<char> Connector::getBinary(uint64_t offset, int32_t size) const
{
    if (size <= 0)
        return std::vector<char>();

    if (m_arbiter->isLocal(m_filename))
    {
        std::vector<char> buf(size);
        std::ifstream in(m_filename, std::ios::binary);
        if (in.fail() )
        {
            std::string message = "Unable to open '" + m_filename + "'.";
            if (!pdal::FileUtils::fileExists(m_filename))
                message += " File does not exist.";
            throw pdal_error(message);
        }
        in.seekg(offset);
        in.read(buf.data(), size);
        return buf;
    }
    else
    {
        StringMap headers(m_headers);
        headers["Range"] = "bytes=" + std::to_string(offset) + "-" +
            std::to_string(offset + size - 1);
        return m_arbiter->getBinary(m_filename, headers, m_query);
    }
}

} // namespace connector
} // namespace pdal
