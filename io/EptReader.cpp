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
    log()->get(LogLevel::Debug) << "Endpoint: " << m_ep->prefixedRoot() <<
        std::endl;

    m_info.reset(new EptInfo(parse(m_ep->get("entwine.json"))));
    log()->get(LogLevel::Debug) << "Got EPT info" << std::endl;

    if (!m_info->srs().empty())
        setSpatialReference(m_info->srs());

    // Figure out our query parameters.
    m_queryBounds = m_args.bounds();
    handleOriginQuery();

    const auto& scale(m_info->scale());
    const auto& offset(m_info->offset());

    log()->get(LogLevel::Debug) << "Query bounds: " << m_queryBounds <<
        std::endl;
    log()->get(LogLevel::Debug) << "Scale: " <<
        scale[0] << ", " << scale[1] << ", " << scale[2] << std::endl;
    log()->get(LogLevel::Debug) << "Offset: " <<
        offset[0] << ", " << offset[1] << ", " << offset[2] << std::endl;
    log()->get(LogLevel::Debug) << "Threads: " << m_pool->size() << std::endl;
}

void EptReader::handleOriginQuery()
{
    if (m_args.origin().empty()) return;

    const std::string search(m_args.origin());
    log()->get(LogLevel::Debug) << "Searching files for " << search <<
        std::endl;

    const Json::Value files(parse(m_ep->get("entwine-files.json")));
    log()->get(LogLevel::Debug) << "Fetched files list" << std::endl;

    if (!files.isArray())
    {
        throwError("Unexpected entwine-files list: " + files.toStyledString());
    }

    if (search.find_first_not_of("0123456789") == std::string::npos)
    {
        // If the origin search is integral, then the OriginId value has been
        // specified directly.
        m_queryOriginId.reset(new uint64_t(std::stoull(search)));
    }
    else
    {
        // Otherwise it's a file path (or part of one - for example selecting
        // by a basename or a tile ID rather than a full path is convenient).
        // Find it within the files list, and make sure it's specified uniquely
        // enough to select only one filt.
        for (Json::ArrayIndex i(0); i < files.size(); ++i)
        {
            const Json::Value& entry(files[i]);
            if (entry["path"].asString().find(search) != std::string::npos)
            {
                if (m_queryOriginId)
                {
                    throwError("Origin search path is not unique");
                }
                m_queryOriginId.reset(new uint64_t(i));
            }
        }
    }

    if (!m_queryOriginId)
    {
        throwError("Failed lookup of origin: " + search);
    }

    if (*m_queryOriginId >= files.size())
    {
        throwError("Invalid origin ID");
    }

    // Now that we have our OriginId value, clamp the bounds to select only the
    // data files that overlap the selected origin.

    const Json::Value& found(files[static_cast<Json::ArrayIndex>(
                *m_queryOriginId)]);

    BOX3D q(toBox3d(found["bounds"]));

    if (m_info->json().isMember("scale"))
    {
        // Make sure floating point precision doesn't make us discard points
        // from a file query.  We'll be checking the OriginId anyway so we have
        // leeway to bloat the query bounds as needed.
        const auto& scale(m_info->scale());
        q.minx -= std::fabs(scale[0]);
        q.miny -= std::fabs(scale[1]);
        q.minz -= std::fabs(scale[2]);
        q.maxx += std::fabs(scale[0]);
        q.maxy += std::fabs(scale[1]);
        q.maxz += std::fabs(scale[2]);
    }

    // Clip the bounds to the queried origin bounds.  Don't just overwrite it -
    // it's possible that both a bounds and an origin are specified.
    m_queryBounds.clip(q);

    log()->get(LogLevel::Debug) << "Query origin " <<
        *m_queryOriginId << ": " << found["path"].asString() <<
        std::endl;
}

QuickInfo EptReader::inspect()
{
    QuickInfo qi;

    initialize();

    qi.m_bounds = toBox3d(m_info->json()["boundsConforming"]);
    qi.m_srs = m_info->srs();
    qi.m_pointCount = m_info->numPoints();

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

    // Register XYZ directly since they may appear as scaled int32 values in the
    // schema.  Either way we want doubles.
    layout->registerDim(Dimension::Id::X);
    layout->registerDim(Dimension::Id::Y);
    layout->registerDim(Dimension::Id::Z);

    for (Json::ArrayIndex i(0); i < schema.size(); ++i)
    {
        const Json::Value& dim(schema[i]);
        const std::string name(dim["name"].asString());
        log()->get(LogLevel::Debug) << "Registering dim " << name << std::endl;
        const Dimension::Type type(Dimension::type(dim["type"].asString()));
        layout->registerOrAssignDim(name, type);
        m_remoteLayout->registerOrAssignDim(name, type);
    }

    m_remoteLayout->finalize();
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

    using D = Dimension::Id;

    // Since we might need to revert a scale/offset for XYZ if the dataType is
    // binary, they'll be handled separately.
    m_dimTypes = table.layout()->dimTypes();
    Utils::remove_if(m_dimTypes, [](const pdal::DimType& d)
    {
        return d.m_id == D::X || d.m_id == D::Y || d.m_id == D::Z;
    });
}

void EptReader::overlaps()
{
    // Determine all the keys that overlap the queried area by traversing the
    // EPT hierarchy (see https://git.io/fAiuR).  Because this may require
    // fetching lots of JSON files, it'll run in our thread pool.
    const Key key(m_info->bounds());
    const Json::Value hier(parse(m_ep->get("h/" + key.toString() + ".json")));
    overlaps(hier, key);
    m_pool->await();
}

void EptReader::overlaps(const Json::Value& hier, const Key& key)
{
    if (!key.b.overlaps(m_queryBounds)) return;
    const uint64_t np(hier[key.toString()].asUInt64());
    if (!np) return;

    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_overlapKeys.insert(key);
        m_overlapPoints += np;
    }

    if (m_info->hierarchyStep() != 0 && key.d % m_info->hierarchyStep() == 0)
    {
        log()->get(LogLevel::Debug) << "Hierarchy: " << key.toString() <<
            std::endl;

        m_pool->add([this, key]()
        {
            const auto next(parse(m_ep->get("h/" + key.toString() + ".json")));
            for (uint64_t dir(0); dir < 8; ++dir)
            {
                overlaps(next, key.bisect(dir));
            }
        });
    }
    else
    {
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
    auto handle(m_ep->getLocalHandle(key.toString() + ".laz"));

    PointTable table;

    Options options;
    options.add("filename", handle->localPath());
    options.add("use_eb_vlr", true);

    LasReader reader;
    reader.setOptions(options);

    std::lock_guard<std::mutex> lock(m_mutex);
    reader.prepare(table);

    // While the laszip data type will contain a scale/offset in the Entwine
    // metadata, the LasReader will handle the transformation.
    for (auto& src : reader.execute(table))
    {
        PointRef pr(*src);
        for (uint64_t i(0); i < src->size(); ++i)
        {
            pr.setPointId(i);
            process(dst, pr, false);
        }
    }
}

void EptReader::readBinary(PointView& dst, const Key& key) const
{
    auto data(m_ep->getBinary(key.toString() + ".bin"));
    ShallowPointTable table(*m_remoteLayout, data.data(), data.size());
    PointRef pr(table);

    std::lock_guard<std::mutex> lock(m_mutex);

    // Binary data scale/offset needs to be handled here.
    for (uint64_t i(0); i < table.numPoints(); ++i)
    {
        pr.setPointId(i);
        process(dst, pr, true);
    }
}

void EptReader::process(PointView& dst, PointRef& pr, bool unscale) const
{
    const auto pointId(dst.size());
    double x = pr.getFieldAs<double>(Dimension::Id::X);
    double y = pr.getFieldAs<double>(Dimension::Id::Y);
    double z = pr.getFieldAs<double>(Dimension::Id::Z);
    const uint64_t o = pr.getFieldAs<uint64_t>(Dimension::Id::OriginId);

    if (unscale)
    {
        x = x * m_info->scale()[0] + m_info->offset()[0];
        y = y * m_info->scale()[1] + m_info->offset()[1];
        z = z * m_info->scale()[2] + m_info->offset()[2];
    }

    const bool selected = !m_queryOriginId || o == *m_queryOriginId;

    if (selected && m_queryBounds.contains(x, y, z))
    {
        dst.setField(Dimension::Id::X, pointId, x);
        dst.setField(Dimension::Id::Y, pointId, y);
        dst.setField(Dimension::Id::Z, pointId, z);

        char* pos(dst.getPoint(pointId));

        for (const DimType& dim : m_dimTypes)
        {
            pr.getField(pos + dst.layout()->dimOffset(dim.m_id),
                    dim.m_id, dim.m_type);
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

