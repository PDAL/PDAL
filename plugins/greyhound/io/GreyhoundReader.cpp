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

#include "GreyhoundReader.hpp"

#include <pdal/pdal_features.hpp>
#include <pdal/compression/LazPerfCompression.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

static PluginInfo const s_info
{
    "readers.greyhound",
    "Greyhound Reader",
    "http://pdal.io/stages/readers.greyhound.html"
};

CREATE_SHARED_STAGE(GreyhoundReader, s_info)

std::string GreyhoundReader::getName() const { return s_info.name; }

void GreyhoundReader::addArgs(ProgramArgs& args)
{
    args.add("url", "URL", m_args.url);
    args.add("resource", "Resource name", m_args.resource);
    args.add("bounds", "Bounds", m_args.sbounds);
    args.add("depth_begin", "Beginning depth to query", m_args.depthBegin);
    args.add("depth_end", "Ending depth to query", m_args.depthEnd);
    args.add("tile_path", "Index-optimized tile selection", m_args.tilePath);
    args.add("filter", "Query filter", m_args.filter);
    args.add("dims", "Dimension names to read", m_args.dims);
    args.add("buffer", "Ratio by which to bloat the requested bounds.  The "
            "buffered portion, if writers.greyhound is later used, will not be "
            "written - this allows edge effect mitigation.", m_args.buffer);
}

void GreyhoundReader::initialize(PointTableRef table)
{
    // Parse the URL and query parameters (although we won't handle the schema
    // until addDimensions) and fetch the dataset `info`.

    Json::Value config;
    if (log()->getLevel() > LogLevel::Debug4)
        config["arbiter"]["verbose"] = true;
    m_arbiter.reset(new arbiter::Arbiter(config));

    // If this stage was parsed from a string parameter rather than JSON object
    // specification, normalize it to our URL.
    if (m_filename.size() && m_args.url.empty())
    {
        m_args.url = m_filename;
        const std::string pre("greyhound://");
        if (m_args.url.find(pre) == 0)
            m_args.url = m_args.url.substr(pre.size());
    }

    m_params = GreyhoundParams(m_args);

    log()->get(LogLevel::Debug) << "Fetching info from " << m_params.root() <<
        std::endl;

    try
    {
        m_info = parse(m_arbiter->get(m_params.root() + "info"));
    }
    catch (std::exception& e)
    {
        throw pdal_error(std::string("Failed to fetch info: ") + e.what());
    }

    if (m_info.isMember("srs"))
        setSpatialReference(m_info["srs"].asString());
}

void GreyhoundReader::addDimensions(PointLayoutPtr layout)
{
    std::map<std::string, const Json::Value*> remote;
    for (const auto& d : m_info["schema"])
        remote[d["name"].asString()] = &d;

    // The dimensions we will query are determined in the following order, the
    // first of which is present taking precedence:
    //  - The `dims` PDAL option on this stage.
    //  - A `schema` query parameter parsed from the `url` option.
    //  - Finally, fall back to the full `info.schema`.
    if (m_args.dims.isNull())
    {
        // If no dimensions are explicitly listed, only include the indexed
        // schema - omitting any appended dimensions.  Those must be explicitly
        // specified if desired.
        Json::Value nativeSchema;
        for (const Json::Value d : m_info["schema"])
            if (!d["addon"].asBool())
                nativeSchema.append(d);

        const Json::Value& readSchema = m_params["schema"].isNull() ?
            nativeSchema : m_params["schema"];

        for (const auto& d : readSchema)
        {
            m_args.dims.append(d["name"].asString());
        }
    }

    auto isXyz([](const std::string& name)
    {
        return name == "X" || name == "Y" || name == "Z";
    });

    // Build the request layout from the specified `dims`.
    for (const auto& n : m_args.dims)
    {
        const auto name(n.asString());

        if (!remote.count(name))
            throw pdal_error(name + " does not exist in the remote schema");

        const auto& d(*remote.at(name));

        const int baseType(
                Utils::toNative(Dimension::fromName(d["type"].asString())));

        const int size(d["size"].asInt());

        const Dimension::Type type(
                isXyz(name) ?
                    Dimension::Type::Double :
                    static_cast<Dimension::Type>(baseType | size));

        layout->registerOrAssignDim(name, type);
        m_readLayout.registerOrAssignDim(name, type);
    }

    if (!m_params.obounds().isNull())
    {
        layout->registerDim(Dimension::Id::Omit);
    }

    // We'll keep track of the point ordering here in case any intermediate
    // filters reorder them.  We need to write them with a 1:1 mapping to the
    // original query.
    layout->registerDim(Dimension::Id::PointId);

    m_params["schema"] = layoutToSchema(m_readLayout);

    log()->get(LogLevel::Debug) << "Schema: " << m_params["schema"] <<
        std::endl;
}

void GreyhoundReader::prepared(PointTableRef table)
{
    MetadataNode queryNode(table.privateMetadata("greyhound"));
    queryNode.add("info", dense(m_info));
    queryNode.add("root", m_params.root());
    queryNode.add("params", dense(m_params.toJson()));
}

point_count_t GreyhoundReader::read(PointViewPtr view, point_count_t count)
{
    const std::string url(m_params.root() + "read" + m_params.qs());
    log()->get(LogLevel::Debug) << "Reading: " << url << std::endl;

    auto response(m_arbiter->getBinary(url));
    const std::size_t pointSize(view->layout()->pointSize());

    uint32_t numPoints(0);
    std::copy(
            response.data() + response.size() - sizeof(uint32_t),
            response.data() + response.size(),
            reinterpret_cast<char*>(&numPoints));

    log()->get(LogLevel::Debug) <<
        "Fetched " << numPoints << " points" << std::endl;
    log()->get(LogLevel::Debug) <<
        "Fetched " << response.size() << " bytes" << std::endl;

    response.resize(response.size() - sizeof(uint32_t));

    const auto dimTypes(m_readLayout.dimTypes());
#ifdef PDAL_HAVE_LAZPERF
    auto cb = [this, &view, &dimTypes, numPoints](char *buf, size_t bufsize)
    {
        view->setPackedPoint(dimTypes, view->size(), buf);
        if (m_cb)
            m_cb(*view, view->size() - 1);
    };
    LazPerfDecompressor(cb, dimTypes, numPoints).
        decompress(response.data(), response.size());
#else
    const char* end(response.data() + response.size());
    for (const char* pos(response.data()); pos < end; pos += pointSize)
    {
        view->setPackedPoint(dimTypes, view->size(), pos);
        if (m_cb)
            m_cb(*view, view->size() - 1);
    }
#endif
    if (!m_params.obounds().isNull())
    {
        greyhound::Bounds obounds(m_params.obounds());
        greyhound::Point p;

        for (std::size_t i(0); i < view->size(); ++i)
        {
            p.x = view->getFieldAs<double>(Dimension::Id::X, i);
            p.y = view->getFieldAs<double>(Dimension::Id::Y, i);
            p.z = view->getFieldAs<double>(Dimension::Id::Z, i);

            if (!obounds.contains(p))
                view->setField(Dimension::Id::Omit, i, 1);
        }
    }

    for (std::size_t i(0); i < view->size(); ++i)
    {
        view->setField(Dimension::Id::PointId, i, i);
    }

    return numPoints;
}

} // namespace pdal

