/******************************************************************************
* Copyright (c) 2025, Hobu Inc.
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

#include "FileSpec.hpp"

#include <nlohmann/json.hpp>

#include <pdal/util/private/JsonSupport.hpp>
#include <pdal/private/FileSpecHelper.hpp>

namespace pdal
{

struct FileSpec::Private
{
    std::filesystem::path m_path;
    StringMap m_headers;
    StringMap m_query;

    Utils::StatusWithReason parse(NL::json& node);
    Utils::StatusWithReason extractPath(NL::json& node);
    Utils::StatusWithReason extractHeaders(NL::json& node);
    Utils::StatusWithReason extractQuery(NL::json& node);
};

namespace
{

bool extractStringMap(NL::json& node, StringMap& map)
{
    if (!node.is_object())
        return false;
    for (auto& [key, val] : node.items())
    {
        if (val.is_string())
            map.insert({key, val});
        else
            return false;
    }
    return true;
}

} // unnamed namespace

FileSpec::FileSpec() : m_p(new Private)
{}

FileSpec::FileSpec(const std::string& pathOrJson) : m_p(new Private)
{
    (void)ingest(pathOrJson);
}

FileSpec::~FileSpec()
{}

FileSpec::FileSpec(const FileSpec& other) : m_p(new Private)
{
    *m_p = *other.m_p;
}

FileSpec& FileSpec::operator=(const FileSpec& other)
{
    *m_p = *other.m_p;
    return *this;
}

FileSpec::FileSpec(FileSpec&& other)
{
    m_p = std::move(other.m_p);
}

FileSpec& FileSpec::operator=(FileSpec&& other)
{
    m_p = std::move(other.m_p);
    return *this;
}

bool FileSpec::valid() const
{
    return !m_p->m_path.empty();
}

bool FileSpec::onlyFilename() const
{
    return m_p->m_headers.empty() && m_p->m_query.empty();
}

std::filesystem::path FileSpec::filePath() const
{
    return m_p->m_path;
}

void FileSpec::setFilePath(const std::string& path)
{
    m_p->m_path = path;
}

void FileSpec::setFilePath(const std::filesystem::path& path)
{
    m_p->m_path = path;
}

StringMap FileSpec::query() const
{
    return m_p->m_query;
}

StringMap FileSpec::headers() const
{
    return m_p->m_headers;
}

Utils::StatusWithReason FileSpec::ingest(const std::string& pathOrJson)
{
    auto isJson = [](const std::string& s) -> bool
    {
        static constexpr std::array<std::pair<char, char>, 3> delims
            { { { '{', '}' }, { '[', ']' }, { '"', '"' } } };

        std::string t = s;
        Utils::trim(t);

        if (t.size() < 2)
            return false;
        for (const std::pair<char, char>& d : delims)
            if (t.front() == d.first && t.back() == d.second)
                return true;
        return false;
    };

    NL::json json;
    if (isJson(pathOrJson))
    {
        auto status = Utils::parseJson(pathOrJson, json);
        if (!status)
            return status;
    }
    // assuming input is a filename
    else
        json = NL::json(pathOrJson);

    return m_p->parse(json);
}

Utils::StatusWithReason FileSpec::Private::parse(NL::json& node)
{
    if (node.is_null())
        return { -1, "'filename' argument contains no data" };
    if (node.is_string())
        m_path = node.get<std::string>();
    else if (node.is_object())
    {
        auto status = extractPath(node);
        if (!status)
            return { -1, status.what() };
        status = extractHeaders(node);
        if (!status)
            return { -1, status.what() };
        status = extractQuery(node);
        if (!status)
            return { -1, status.what() };
        if (!node.empty())
            return { -1, "Invalid item in filename object: " + node.dump() };
    }
    else
        return { -1, "'filename' must be specified as a string." };
    return true;
}


Utils::StatusWithReason FileSpec::Private::extractPath(NL::json& node)
{
    auto it = node.find("path");
    if (it == node.end())
        return { -1, "'filename' object must contain 'path' member." };
    NL::json& val = *it;
    if (!val.is_null())
    {
        if (val.is_string())
            m_path = val.get<std::string>();
        else
            return { -1, "'filename' object 'path' member must be specified as a string." };
        node.erase(it);
    }
    return true;
}

Utils::StatusWithReason FileSpec::Private::extractHeaders(NL::json& node)
{
    auto it = node.find("headers");
    if (it == node.end())
        return true;
    NL::json& val = *it;
    if (!val.is_null())
    {
        if (!extractStringMap(val, m_headers))
            return { -1, "'filename' sub-argument 'headers' must be an object of "
                "string key-value pairs." };
    }
    node.erase(it);
    return true;
}

Utils::StatusWithReason FileSpec::Private::extractQuery(NL::json& node)
{
    auto it = node.find("query");
    if (it == node.end())
        return true;
    NL::json& val = *it;
    if (!val.is_null())
    {
        if (!extractStringMap(val, m_query))
            return { -1, "'filename' sub-argument 'query' must be an object of "
                "string key-value pairs." };
    }
    node.erase(it);
    return true;
}

// Provide access to the private 'parse' function.
Utils::StatusWithReason FileSpecHelper::parse(FileSpec& spec, NL::json& node)
{
    return spec.m_p->parse(node);
}

std::ostream& operator << (std::ostream& out, const FileSpec& spec)
{
    if (spec.onlyFilename())
        return out << spec.filePath().string();

    NL::json json;
    json["path"] = spec.m_p->m_path.string();
    if (!spec.m_p->m_headers.empty())
        json["headers"] = spec.m_p->m_headers;
    if (!spec.m_p->m_query.empty())
        json["query"] = spec.m_p->m_query;

    out << json;
    return out;
}

} // namespace pdal
