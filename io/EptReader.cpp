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

#include "EptReader.hpp"

#include <limits>

#include "private/EptSupport.hpp"
#include "LasReader.hpp"

#include <arbiter/arbiter.hpp>

#include <pdal/util/Algorithm.hpp>

namespace pdal
{

namespace
{
    const StaticPluginInfo s_info
    {
        "readers.ept",
        "EPT Reader",
        "http://pdal.io/stages/reader.ept.html",
        { "ept" }
    };

    Dimension::Type getType(const Json::Value& dim)
    {
        if (dim.isMember("scale"))
            return Dimension::Type::Double;

        const std::string type(dim["type"].asString());
        const uint64_t size(dim["size"].asUInt64());

        if (type == "signed")
        {
            switch (size)
            {
                case 1: return Dimension::Type::Signed8;
                case 2: return Dimension::Type::Signed16;
                case 4: return Dimension::Type::Signed32;
                case 8: return Dimension::Type::Signed64;
            }
        }
        else if (type == "unsigned")
        {
            switch (size)
            {
                case 1: return Dimension::Type::Unsigned8;
                case 2: return Dimension::Type::Unsigned16;
                case 4: return Dimension::Type::Unsigned32;
                case 8: return Dimension::Type::Unsigned64;
            }
        }
        else if (type == "float")
        {
            switch (size)
            {
                case 4: return Dimension::Type::Float;
                case 8: return Dimension::Type::Double;
            }
        }

        return Dimension::Type::None;
    }
}

CREATE_STATIC_STAGE(EptReader, s_info);

EptReader::EptReader()
{}

EptReader::~EptReader()
{}

std::string EptReader::getName() const { return s_info.name; }

void EptReader::addArgs(ProgramArgs& args)
{
    args.add("bounds", "Bounds to fetch", m_args.boundsArg());
    args.add("origin", "Origin of source file to fetch", m_args.originArg());
    args.add("threads", "Number of worker threads", m_args.threadsArg());
    args.add("resolution", "Resolution limit", m_args.resolutionArg());
}

BOX3D EptReader::Args::bounds() const
{
    // If already 3D (which is non-empty), return it as-is.
    if (m_bounds.is3d())
        return m_bounds.to3d();

    // If empty, return maximal extents to select everything.
    const BOX2D b(m_bounds.to2d());
    if (b.empty())
    {
        double mn((std::numeric_limits<double>::lowest)());
        double mx((std::numeric_limits<double>::max)());
        return BOX3D(mn, mn, mn, mx, mx, mx);
    }

    // Finally if 2D and non-empty, convert to 3D with all-encapsulating
    // Z-values.
    return BOX3D(
            b.minx, b.miny, (std::numeric_limits<double>::lowest)(),
            b.maxx, b.maxy, (std::numeric_limits<double>::max)());
}

void EptReader::initialize()
{
    m_root = m_filename;

    const std::string prefix("ept://");
    if (m_root.substr(0, prefix.size()) == prefix)
    {
        m_root = m_root.substr(prefix.size());
    }

    m_arbiter.reset(new arbiter::Arbiter());
    m_ep.reset(new arbiter::Endpoint(m_arbiter->getEndpoint(m_root)));
    m_pool.reset(new Pool(m_args.threads()));
    auto& debug(log()->get(LogLevel::Debug));

    debug << "Endpoint: " << m_ep->prefixedRoot() << std::endl;
    m_info.reset(new EptInfo(parse(m_ep->get("ept.json"))));
    debug << "Got EPT info" << std::endl;
    debug << "SRS: " << m_info->srs() << std::endl;
    setSpatialReference(m_info->srs());

    // Figure out our query parameters.
    m_queryBounds = m_args.bounds();
    handleOriginQuery();

    // Figure out our max depth.
    const double queryResolution(m_args.resolution());
    if (queryResolution)
    {
        double currentResolution =
            (m_info->bounds().maxx - m_info->bounds().minx) / m_info->span();

        debug << "Root resolution: " << currentResolution << std::endl;

        // To select the current resolution level, we need depthEnd to be one
        // beyond it - this is a non-inclusive parameter.
        ++m_depthEnd;

        while (currentResolution > queryResolution)
        {
            currentResolution /= 2;
            ++m_depthEnd;
        }

        debug << "Query resolution:  " << queryResolution << "\n";
        debug << "Actual resolution: " << currentResolution << "\n";
        debug << "Depth end: " << m_depthEnd << "\n";
    }

    debug << "Query bounds: " << m_queryBounds << "\n";
    debug << "Threads: " << m_pool->size() << std::endl;
}

void EptReader::handleOriginQuery()
{
    if (m_args.origin().empty()) return;

    const std::string search(m_args.origin());
    log()->get(LogLevel::Debug) << "Searching sources for " << search <<
        std::endl;

    const Json::Value sources(parse(m_ep->get("ept-sources/list.json")));
    log()->get(LogLevel::Debug) << "Fetched sources list" << std::endl;

    if (!sources.isArray())
    {
        throwError("Unexpected sources list: " + sources.toStyledString());
    }

    if (search.find_first_not_of("0123456789") == std::string::npos)
    {
        // If the origin search is integral, then the OriginId value has been
        // specified directly.
        m_queryOriginId = std::stoll(search);
    }
    else
    {
        // Otherwise it's a file path (or part of one - for example selecting
        // by a basename or a tile ID rather than a full path is convenient).
        // Find it within the sources list, and make sure it's specified
        // uniquely enough to select only one file.
        for (Json::ArrayIndex i(0); i < sources.size(); ++i)
        {
            const Json::Value& entry(sources[i]);
            if (entry["id"].asString().find(search) != std::string::npos)
            {
                if (m_queryOriginId != -1)
                {
                    throwError("Origin search ID is not unique");
                }
                m_queryOriginId = static_cast<int64_t>(i);
            }
        }
    }

    if (m_queryOriginId == -1)
    {
        throwError("Failed lookup of origin: " + search);
    }

    if (m_queryOriginId >= sources.size())
    {
        throwError("Invalid origin ID");
    }

    // Now that we have our OriginId value, clamp the bounds to select only the
    // data sources that overlap the selected origin.

    const Json::Value& found(sources[static_cast<Json::ArrayIndex>(
                m_queryOriginId)]);

    BOX3D q(toBox3d(found["bounds"]));

    // Clip the bounds to the queried origin bounds.  Don't just overwrite it -
    // it's possible that both a bounds and an origin are specified.
    m_queryBounds.clip(q);

    log()->get(LogLevel::Debug) << "Query origin " <<
        m_queryOriginId << ": " << found["id"].asString() <<
        std::endl;
}

QuickInfo EptReader::inspect()
{
    QuickInfo qi;

    initialize();

    qi.m_bounds = toBox3d(m_info->json()["boundsConforming"]);
    qi.m_srs = m_info->srs();
    qi.m_pointCount = m_info->points();

    for (Json::ArrayIndex i(0); i < m_info->schema().size(); ++i)
    {
        qi.m_dimNames.push_back(m_info->schema()[i]["name"].asString());
    }

    qi.m_valid = true;

    return qi;
}

void EptReader::addDimensions(PointLayoutPtr layout)
{
    const Json::Value& schema(m_info->schema());
    m_remoteLayout.reset(new FixedPointLayout());

    for (Json::ArrayIndex i(0); i < schema.size(); ++i)
    {
        const Json::Value& dim(schema[i]);
        const std::string name(dim["name"].asString());

        // If the dimension has a scale, make sure we register it as a double
        // rather than its serialized type.
        const Dimension::Type type = getType(dim);

        log()->get(LogLevel::Debug) << "Registering dim " << name << ": " <<
            Dimension::interpretationName(type) << std::endl;

        layout->registerOrAssignDim(name, type);
        m_remoteLayout->registerOrAssignDim(name, type);
    }

    m_remoteLayout->finalize();

    using D = Dimension::Id;

    m_dimTypes = m_remoteLayout->dimTypes();
    for (DimType& dt : m_dimTypes)
    {
        // If the data is stored as Laszip, then PDAL's LasReader will unscale
        // XYZ for us.
        if (m_info->dataType() == EptInfo::DataType::Laszip &&
                (dt.m_id == D::X || dt.m_id == D::Y || dt.m_id == D::Z))
        {
            continue;
        }

        const Json::Value dim(m_info->dim(m_remoteLayout->dimName(dt.m_id)));
        if (dim.isNull())
            throwError("Invalid dimension lookup");
        if (dim.isMember("scale"))
            dt.m_xform.m_scale.m_val = dim["scale"].asDouble();
        if (dim.isMember("offset"))
            dt.m_xform.m_offset.m_val = dim["offset"].asDouble();
    }

    auto dimIndex = [this](Dimension::Id id)->uint64_t
    {
        for (uint64_t i(0); i < m_remoteLayout->dims().size(); ++i)
        {
            if (m_remoteLayout->dims()[i] == id)
            {
                return i;
            }
        }
        throw pdal_error("Invalid dimIndex lookup");
    };

    m_xyzTransforms[0] = m_dimTypes[dimIndex(D::X)].m_xform;
    m_xyzTransforms[1] = m_dimTypes[dimIndex(D::Y)].m_xform;
    m_xyzTransforms[2] = m_dimTypes[dimIndex(D::Z)].m_xform;
}

void EptReader::ready(PointTableRef table)
{
    m_overlapKeys.clear();
    m_overlapPoints = 0;

    // Determine all overlapping data files we'll need to fetch.
    overlaps();

    log()->get(LogLevel::Debug) << "Overlap nodes: " << m_overlapKeys.size() <<
        std::endl;
    log()->get(LogLevel::Debug) << "Overlap points: " << m_overlapPoints <<
        std::endl;

    if (m_overlapPoints > 1e8)
    {
        log()->get(LogLevel::Warning) << m_overlapPoints <<
            " will be downloaded" << std::endl;
    }
}

void EptReader::overlaps()
{
    // Determine all the keys that overlap the queried area by traversing the
    // EPT hierarchy (see https://git.io/fAiuR).  Because this may require
    // fetching lots of JSON files, it'll run in our thread pool.
    const Key key(m_info->bounds());
    const Json::Value hier(parse(m_ep->get(
                    "ept-hierarchy/" + key.toString() + ".json")));
    overlaps(hier, key);
    m_pool->await();
}

void EptReader::overlaps(const Json::Value& hier, const Key& key)
{
    if (!key.b.overlaps(m_queryBounds)) return;
    if (m_depthEnd && key.d >= m_depthEnd) return;
    const int64_t np(hier[key.toString()].asInt64());
    if (!np) return;

    if (np == -1)
    {
        // If the hierarchy points value here is -1, then we need to fetch the
        // hierarchy subtree corresponding to this root.
        log()->get(LogLevel::Debug) << "Hierarchy: " << key.toString() <<
            std::endl;

        m_pool->add([this, key]()
        {
            const auto subRoot(parse(m_ep->get(
                            "ept-hierarchy/" + key.toString() + ".json")));
            overlaps(subRoot, key);
        });
    }
    else
    {
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_overlapKeys.insert(key);
            m_overlapPoints += static_cast<uint64_t>(np);
        }

        for (uint64_t dir(0); dir < 8; ++dir)
        {
            overlaps(hier, key.bisect(dir));
        }
    }
}

PointViewSet EptReader::run(PointViewPtr view)
{
    uint64_t i(0);
    for (const Key& key : m_overlapKeys)
    {
        log()->get(LogLevel::Debug) << "Data " << ++i << "/" <<
            m_overlapKeys.size() << ": " << key.toString() << std::endl;

        m_pool->add([this, &view, &key]()
        {
            if (m_info->dataType() == EptInfo::DataType::Laszip)
                readLaszip(*view, key);
            else
                readBinary(*view, key);
        });
    }

    m_pool->await();

    log()->get(LogLevel::Debug) << "Done reading!" << std::endl;

    PointViewSet views;
    views.insert(view);
    return views;
}

void EptReader::readLaszip(PointView& dst, const Key& key) const
{
    // If the file is remote (HTTP, S3, Dropbox, etc.), getLocalHandle will
    // download the file and `localPath` will return the location of the
    // downloaded file in a temporary directory.  Otherwise it's a no-op.
    auto handle(m_ep->getLocalHandle("ept-data/" + key.toString() + ".laz"));

    PointTable table;

    Options options;
    options.add("filename", handle->localPath());
    options.add("use_eb_vlr", true);

    LasReader reader;
    reader.setOptions(options);

    std::lock_guard<std::mutex> lock(m_mutex);
    reader.prepare(table);

    for (auto& src : reader.execute(table))
    {
        PointRef pr(*src);
        for (uint64_t i(0); i < src->size(); ++i)
        {
            pr.setPointId(i);
            process(dst, pr);
        }
    }
}

void EptReader::readBinary(PointView& dst, const Key& key) const
{
    auto data(m_ep->getBinary("ept-data/" + key.toString() + ".bin"));
    ShallowPointTable table(*m_remoteLayout, data.data(), data.size());
    PointRef pr(table);

    std::lock_guard<std::mutex> lock(m_mutex);

    for (uint64_t i(0); i < table.numPoints(); ++i)
    {
        pr.setPointId(i);
        process(dst, pr);
    }
}

void EptReader::process(PointView& dst, PointRef& pr) const
{
    using D = Dimension::Id;

    const point_count_t pointId(dst.size());

    const double x = pr.getFieldAs<double>(D::X) *
        m_xyzTransforms[0].m_scale.m_val + m_xyzTransforms[0].m_offset.m_val;
    const double y = pr.getFieldAs<double>(D::Y) *
        m_xyzTransforms[1].m_scale.m_val + m_xyzTransforms[1].m_offset.m_val;
    const double z = pr.getFieldAs<double>(D::Z) *
        m_xyzTransforms[2].m_scale.m_val + m_xyzTransforms[2].m_offset.m_val;

    const bool selected = m_queryOriginId == -1 ||
        pr.getFieldAs<int64_t>(D::OriginId) == m_queryOriginId;

    if (selected && m_queryBounds.contains(x, y, z))
    {
        dst.setField(Dimension::Id::X, pointId, x);
        dst.setField(Dimension::Id::Y, pointId, y);
        dst.setField(Dimension::Id::Z, pointId, z);

        for (const DimType& dt : m_dimTypes)
        {
            if (dt.m_id != D::X && dt.m_id != D::Y && dt.m_id != D::Z)
            {
                const double d = pr.getFieldAs<double>(dt.m_id) *
                    dt.m_xform.m_scale.m_val + dt.m_xform.m_offset.m_val;

                dst.setField(dt.m_id, pointId, d);
            }
        }
    }
}

Json::Value EptReader::parse(const std::string& data) const
{
    Json::Value json;
    Json::Reader reader;

    if (!reader.parse(data, json, false))
    {
        const std::string jsonError(reader.getFormattedErrorMessages());
        if (!jsonError.empty())
        {
            throwError("Error during parsing: " + jsonError);
        }
    }

    return json;
}

} // namespace pdal

