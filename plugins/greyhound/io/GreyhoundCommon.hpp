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

#include <json/json.h>

#include <pdal/pdal_types.hpp>
#include <pdal/PointLayout.hpp>

#include "bounds.hpp"

namespace pdal
{

namespace greyhound = entwine;

static inline Json::Value parse(const std::string& data)
{
    Json::Value json;
    Json::Reader reader;

    if (data.size())
    {
        if (!reader.parse(data, json, false))
        {
            const std::string jsonError(reader.getFormattedErrorMessages());
            if (!jsonError.empty())
            {
                throw pdal_error("Error during parsing: " + jsonError);
            }
        }
    }

    return json;
}

static inline std::string dense(const Json::Value& json)
{
    Json::StreamWriterBuilder builder;
    builder.settings_["indentation"] = "";
    return Json::writeString(builder, json);
}

static inline Json::Value layoutToSchema(PointLayout& layout)
{
    Json::Value schema;

    layout.finalize();
    for (const Dimension::Id id : layout.dims())
    {
        const Dimension::Detail& d(*layout.dimDetail(id));
        const std::string name(layout.dimName(id));

        Json::Value j;
        j["name"] = name;
        j["type"] = Dimension::toName(base(d.type()));
        j["size"] = static_cast<int>(Dimension::size(d.type()));
        schema.append(j);
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
    Json::Value filter;
    Json::Value dims;
    Json::Value schema;
    double buffer = 0;
};

class GreyhoundParams
{
public:
    GreyhoundParams() { }
    GreyhoundParams(const GreyhoundArgs& args);
    GreyhoundParams(const std::string& resourceRoot, Json::Value params)
        : m_url(resourceRoot)
        , m_params(params)
    {
        if (m_params.isMember("obounds"))
        {
            m_obounds = m_params["obounds"];
            m_params.removeMember("obounds");
        }
    }

    std::string root() const { return m_url; }
    std::string qs() const;

    Json::Value& operator[](std::string key) { return m_params[key]; }
    Json::Value toJson() const
    {
        Json::Value json(m_params);
        if (!m_obounds.isNull())
            json["obounds"] = m_obounds;
        return json;
    }

    Json::Value obounds() const { return m_obounds; }
    void removeMember(std::string key) { m_params.removeMember(key); }

private:
    std::string extractUrl(const GreyhoundArgs& args) const;
    Json::Value extractParams(const GreyhoundArgs& args);

    Json::Value m_obounds;

    std::string m_url;
    Json::Value m_params;
};

} // namespace pdal

