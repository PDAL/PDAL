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

namespace pdal
{

namespace
{

bool extractStringMap(nlohmann::json& node, StringMap& map)
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


Utils::StatusWithReason FileSpec::ingest(const std::string& pathOrJson)
{
    nlohmann::json json;
    size_t pos = Utils::extractSpaces(pathOrJson, 0);
    if (pathOrJson[pos] == '{' || pathOrJson[pos] == '[')
    {
        auto status = Utils::parseJson(pathOrJson, json);
        if (!status)
            return status;
    }
    // assuming input is a filename
    else
        json = nlohmann::json(pathOrJson);

    return parse(json);
}

Utils::StatusWithReason FileSpec::parse(nlohmann::json& node)
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

Utils::StatusWithReason FileSpec::extractPath(nlohmann::json& node)
{
    auto it = node.find("path");
    if (it == node.end())
        return { -1, "'filename' object must contain 'path' member." };
    nlohmann::json& val = *it;
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

Utils::StatusWithReason FileSpec::extractQuery(nlohmann::json& node)
{
    auto it = node.find("query");
    if (it == node.end())
        return true;
    nlohmann::json& val = *it;
    if (!val.is_null())
    {
        if (!extractStringMap(val, m_query))
            return { -1, "'filename' sub-argument 'query' must be an object of "
                "string key-value pairs." };
    }
    node.erase(it);
    return true;
}

Utils::StatusWithReason FileSpec::extractHeaders(nlohmann::json& node)
{
    auto it = node.find("headers");
    if (it == node.end())
        return true;
    nlohmann::json& val = *it;
    if (!val.is_null())
    {
        if (!extractStringMap(val, m_headers))
            return { -1, "'filename' sub-argument 'headers' must be an object of "
                "string key-value pairs." };
    }
    node.erase(it);
    return true;
}

std::ostream& operator << (std::ostream& out, const FileSpec& spec)
{
    nlohmann::json json;
    json["path"] = spec.m_path.string();
    if (!spec.m_headers.empty())
        json["headers"] = spec.m_headers;
    if (!spec.m_query.empty())
        json["query"] = spec.m_query;

    out << json;
    return out;
}


} // namespace pdal
