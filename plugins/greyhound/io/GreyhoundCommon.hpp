/******************************************************************************
* Copyright (c) 2017, Connor Manning (connor@hobu.co)
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

#include <cstddef>
#include <string>

#include <nlohmann/json.hpp>

#include <pdal/pdal_types.hpp>
#include <pdal/PointLayout.hpp>

#include "bounds.hpp"

namespace pdal
{

namespace greyhound = entwine;

static inline NL::json parse(const std::string& data)
{
    NL::json j;

    try
    {
        j= NL::json::parse(data);
    }
    catch (const NL::json::parse_error& err)
    {
        throw pdal_error(std::string("Error during parsing: ") + err.what());
    }
    return j;
}

//ABELL
/**
static inline std::string dense(const Json::Value& json)
{
    Json::StreamWriterBuilder builder;
    builder.settings_["indentation"] = "";
    return Json::writeString(builder, json);
}
**/

static inline NL::json layoutToSchema(PointLayout& layout)
{
    NL::json schema;

    layout.finalize();
    for (const Dimension::Id id : layout.dims())
    {
        const Dimension::Detail& d(*layout.dimDetail(id));
        const std::string name(layout.dimName(id));

        NL::json j {
            { "name", name },
            { "type", Dimension::toName(base(d.type())) },
            { "size", Dimension::size(d.type()) }
        };
        schema.push_back(j);
    }

    return schema;
}

struct GreyhoundArgs
{
    std::string url;
    std::string resource;
    std::string sbounds;
    std::size_t depthBegin = 0;
    std::size_t depthEnd = 0;
    std::string tilePath;
    NL::json filter;
    NL::json dims;
    NL::json schema;
    double buffer = 0;
};

class GreyhoundParams
{
public:
    GreyhoundParams() { }
    GreyhoundParams(const GreyhoundArgs& args);
    GreyhoundParams(std::string resourceRoot, NL::json params)
        : m_url(resourceRoot)
        , m_params(params)
    {
        auto it = m_params.find("obounds");
        if (it != m_params.end())
        {
            m_obounds = *it;
            m_params.erase(it);
        }
    }

    std::string root() const
        { return m_url; }
    std::string qs() const;

    NL::json& operator[](std::string key)
        { return m_params[key]; }
    NL::json toJson() const
    {
        NL::json j(m_params);
        if (!m_obounds.is_null())
            j["obounds"] = m_obounds;
        return j;
    }

    NL::json obounds() const
        { return m_obounds; }
    void removeMember(std::string key)
    {
        try
        {
            m_params.erase(key);
        }
        catch (...)
        {}
    }

private:
    std::string extractUrl(const GreyhoundArgs& args) const;
    NL::json extractParams(const GreyhoundArgs& args);

    NL::json m_obounds;
    std::string m_url;
    NL::json m_params;
};

} // namespace pdal

