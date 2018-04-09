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

#include "GreyhoundCommon.hpp"

#include <regex>
#include <set>
#include <sstream>

#include <pdal/pdal_features.hpp>
#include <pdal/util/Bounds.hpp>

namespace pdal
{

namespace
{

const std::set<std::string> greyhoundParams
{
    "bounds", "depth", "depthBegin", "depthEnd", "filter", "schema", "compress",
    "name"
};

}

GreyhoundParams::GreyhoundParams(const GreyhoundArgs& args)
    : m_url(extractUrl(args))
    , m_params(extractParams(args))
{ }

std::string GreyhoundParams::extractUrl(const GreyhoundArgs& args) const
{
    std::string s;

    // The url may be of the form:
    //      server.com
    //      server.com/resource/asdf/read?<query>
    //
    // We want to extract, from either form (resource may be specified
    // separately):
    //      server.com/resource/asdf/

    // Strip off the query string.
    s = args.url.substr(0, args.url.find('?'));

    if (std::regex_match(s, std::regex(".+/resource/.+/read$")))
    {
        // If the URL is a fully-specified READ query, strip off the "read".
        if (args.resource.size())
            throw pdal_error("Cannot specify resource twice");

        s = s.substr(0, s.size() - 4);
    }

    if (s.empty())
        throw pdal_error("No server specified");

    if (args.resource.size())
    {
        // Otherwise, if resource is specified separately, add it now.
        s = s + (s.back() == '/' ? "" : "/") + "resource/" + args.resource;
    }

    if (s.back() != '/')
        s += '/';

    if (s.find("http://") != 0 && s.find("https://") != 0)
        s = "http://" + s;

    return s;
}

Json::Value GreyhoundParams::extractParams(const GreyhoundArgs& args)
{
    Json::Value json;

    const std::size_t m(args.url.find('?'));
    std::string q(
            args.url.substr(m != std::string::npos ? m + 1 : args.url.size()));

    // First, extract any parameters specified as part of a fully-qualified
    // `read` URL.
    if (q.size())
    {
        std::size_t a(0);
        std::size_t b(q.find_first_of('&'));
        if (b == std::string::npos && a < q.size())
            b = q.size();

        while (b != std::string::npos)
        {
            const std::string curr(q.substr(a, b - a));
            const std::string k(curr.substr(0, curr.find_first_of('=')));
            const std::string v(curr.substr(curr.find_first_of('=') + 1));

            // For Greyhound-specific query parameters, parse them into their
            // proper values since the writer may look at them to recreate the
            // query.  For example, the bounds may need to be manipulated.
            // When we pack them into the query-string, we'll make sure to
            // stringify them.  For other parameters, keep them as strings to
            // pass them along untouched.
            if (greyhoundParams.count(k))
                json[k] = parse(v);
            else
                json[k] = v;

            a = b + 1;
            b = q.find_first_of('&', a);
            if (b == std::string::npos && a < q.size())
                b = q.size();
        }
    }

    if (args.sbounds.size())
    {
        greyhound::Bounds gbounds;
        if (args.sbounds.find('(') != std::string::npos)
        {
            // This is a PDAL-specified bounds.
            Bounds pdalBounds;
            std::istringstream iss(args.sbounds);
            iss >> pdalBounds;
            if (pdalBounds.is3d())
            {
                const auto box(pdalBounds.to3d());
                gbounds = greyhound::Bounds(box.minx, box.miny, box.minz,
                        box.maxx, box.maxy, box.maxz);
            }
            else
            {
                const auto box(pdalBounds.to2d());
                gbounds = greyhound::Bounds(box.minx, box.miny,
                        box.maxx, box.maxy);
            }
        }
        else
        {
            gbounds = greyhound::Bounds(parse(args.sbounds));
        }

        json["bounds"] = gbounds.toJson();
    }

    if (args.buffer)
    {
        if (json["bounds"].isNull())
            throw pdal_error("Cannot specify `buffer` without `bounds`");
        m_obounds = json["bounds"];

        json["bounds"] = greyhound::Bounds(json["bounds"])
            .growBy(args.buffer).toJson();
    }

    if (args.depthBegin)
        json["depthBegin"] = static_cast<Json::UInt64>(args.depthBegin);
    if (args.depthEnd)
        json["depthEnd"] = static_cast<Json::UInt64>(args.depthEnd);

    Json::Value f(args.filter);

    if (f.isString())
        f = parse(f.asString());

    if (args.tilePath.size())
        f["Path"] = args.tilePath;

    if (!f.isNull())
        json["filter"] = f;

    if (!args.schema.isNull())
        json["schema"] = args.schema;

#ifdef PDAL_HAVE_LAZPERF
    json["compress"] = true;
#endif

    return json;
}

std::string GreyhoundParams::qs() const
{
    std::string s;

    auto add([&s](std::string k, std::string v)
    {
        s += (s.size() ? '&' : '?') + k + '=' + v;
    });

    for (const std::string key : m_params.getMemberNames())
    {
        if (greyhoundParams.count(key))
            add(key, dense(m_params[key]));
        else
            add(key, m_params[key].asString());
    }

    return s;
}

} // namespace pdal

