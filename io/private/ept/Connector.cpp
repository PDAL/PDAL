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

#include <boost/filesystem.hpp>
#include <boost/system/error_code.hpp>

#include <pdal/util/FileUtils.hpp>
#include <pdal/pdal_types.hpp>
#include <random>

namespace pdal
{

unsigned char random_char()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);
    return static_cast<unsigned char>(dis(gen));
}

std::string generate_hex(const unsigned int len)
{
    std::stringstream ss;
    for (unsigned int i = 0; i < len; i++)
    {
        auto rc = random_char();
        std::stringstream hexstream;
        hexstream << std::hex << int(rc);
        auto hex = hexstream.str();
        ss << (hex.length() < 2 ? '0' + hex : hex);
    }
    return ss.str();
}

std::string generate_uuid()
{
    return generate_hex(4) + '-' + generate_hex(2) + "-" + generate_hex(2) +
           "-" + generate_hex(2) + "-" + generate_hex(6);
}

StringMap s_cache;
std::string s_uuid = generate_uuid();
std::mutex s_cacheMutex;

Connector::Connector() : m_arbiter(new arbiter::Arbiter()) {}

Connector::Connector(const StringMap& headers, const StringMap& query,
                     bool useCache)
    : m_arbiter(new arbiter::Arbiter), m_headers(headers), m_query(query),
      m_useCache(useCache)
{
}

std::string Connector::getUniquePath(const std::string& path) const
{
    auto ret(pdalboost::filesystem::temp_directory_path());
    ret /= "pdalcache";
    ret /= s_uuid;
    makeDir(ret.string());
    ret /= generate_uuid();
    ret += FileUtils::extension(path);
    return ret.string(); // should check if exists or not
}

std::string Connector::getCache(const std::string& path) const
{
    std::lock_guard<std::mutex> guard(s_cacheMutex);
    if (!s_cache.count(path))
    {
        s_cache.insert(std::make_pair(path, getUniquePath(path)));
        put(s_cache[path], m_arbiter->get(path, m_headers, m_query));
    }
    return s_cache[path];
}

std::string Connector::get(const std::string& path) const
{
    if (m_arbiter->isLocal(path))
        return m_arbiter->get(path);
    else if (m_useCache)
    {

        return get(getCache(path));
    }
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
        throw pdal_error("File '" + path +
                         "' contained invalid JSON: " + err.what());
    }
}

std::string Connector::getBinaryCache(const std::string& path) const
{
    std::lock_guard<std::mutex> guard(s_cacheMutex);
    if (!s_cache.count(path))
    {
        s_cache.insert(std::make_pair(path, getUniquePath(path)));
        put(s_cache[path], m_arbiter->getBinary(path, m_headers, m_query));
    }
    return s_cache[path];
}

std::vector<char> Connector::getBinary(const std::string& path) const
{
    if (m_arbiter->isLocal(path))
        return m_arbiter->getBinary(path);
    else if (m_useCache)
        return getBinary(getBinaryCache(path));
    else
        return m_arbiter->getBinary(path, m_headers, m_query);
}

arbiter::LocalHandle Connector::getLocalHandleCache(const std::string & path) const
{
	std::lock_guard<std::mutex> guard(s_cacheMutex);
    if (!s_cache.count(path))
    {
        s_cache.insert(std::make_pair(path, getUniquePath(path)));
        put(s_cache[path], m_arbiter->getBinary(path, m_headers, m_query));
    }
    return getLocalHandle(s_cache[path]);
}

arbiter::LocalHandle Connector::getLocalHandle(const std::string& path) const
{
    if (m_arbiter->isLocal(path))
        return m_arbiter->getLocalHandle(path);
    else if (m_useCache)
        return getLocalHandleCache(path);
    else
        return m_arbiter->getLocalHandle(path, m_headers, m_query);
}

void Connector::put(const std::string& path, const std::vector<char>& buf) const
{
    if (m_arbiter->isLocal(path))
        return m_arbiter->put(path, buf);
    else
        return m_arbiter->put(path, buf, m_headers, m_query);
}

void Connector::put(const std::string& path, const std::string& data) const
{
    if (m_arbiter->isLocal(path))
        return m_arbiter->put(path, data);
    else
        return m_arbiter->put(path, data, m_headers, m_query);
}

void Connector::makeDir(const std::string& path) const
{
    if (m_arbiter->isLocal(path))
        arbiter::mkdirp(path);
}

} // namespace pdal