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

#include <pdal/util/FileUtils.hpp>
#include <pdal/pdal_types.hpp>

namespace pdal
{

std::string cacheStructureFile(FileUtils::tmpDirectory() + "map.json");

Connector::Connector() : m_arbiter(new arbiter::Arbiter()) {}

Connector::Connector(const StringMap& headers, const StringMap& query,
                     bool useCache)
    : m_arbiter(new arbiter::Arbiter), m_headers(headers), m_query(query),
      m_useCache(useCache)
{
}

NL::json Connector::getCacheList() const
{
    FileUtils::LockFile lockFile("connector");
    if (lockFile.isLocked())
    {
        if (FileUtils::fileExists(cacheStructureFile))
        {
            std::istream *in = FileUtils::openFile(cacheStructureFile,false);
            NL::json list = NL::json::parse(*in);
            FileUtils::closeFile(in);
            return list;
        }
    }

    return NL::json::array();
}

void Connector::writeCacheList(const NL::json& list) const
{
    FileUtils::LockFile lockFile("connector");
    if (lockFile.isLocked())
    {
        std::ostream *out = FileUtils::createFile(cacheStructureFile,false);
        *out << list.dump();
        FileUtils::closeFile(out);
    }
}

std::string Connector::getCacheFile(const std::string& path) const
{
    auto list = getCacheList();
    if (!list.is_array())
        throw pdal::pdal_error("cache map should be an array " + list.dump());


    for (size_t i = 0; i < list.size(); ++i)
    {
        const NL::json& el = list.at(i);
        if (el["origin"].get<std::string>().compare(path) == 0)
            return el["local"].get<std::string>();
    }
    std::string tmpFileName(FileUtils::tmpFileName(FileUtils::extension(path)));
    put(tmpFileName,m_arbiter->getBinary(path, m_headers, m_query));
    NL::json newEl;
    newEl["origin"] = path;
    newEl["local"] = tmpFileName;
    list.push_back(newEl);
    writeCacheList(list);
    return tmpFileName;
}

std::string Connector::get(const std::string& path) const
{
    if (m_arbiter->isLocal(path))
        return m_arbiter->get(path);
    else if (m_useCache)
        return get(getCacheFile(path));
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

std::vector<char> Connector::getBinary(const std::string& path) const
{
    if (m_arbiter->isLocal(path))
        return m_arbiter->getBinary(path);
    else if (m_useCache)
        return getBinary(getCacheFile(path));
    else
        return m_arbiter->getBinary(path, m_headers, m_query);
}

arbiter::LocalHandle Connector::getLocalHandle(const std::string& path) const
{
    if (m_arbiter->isLocal(path))
        return m_arbiter->getLocalHandle(path);
    else if (m_useCache)
        return getLocalHandle(getCacheFile(path));
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