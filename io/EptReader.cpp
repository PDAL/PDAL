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

#include <nlohmann/json.hpp>

#include <pdal/ArtifactManager.hpp>
#include <pdal/pdal_features.hpp>

#include "private/octree/Connector.hpp"
#include "private/octree/EptArtifact.hpp"
#include "private/octree/EptInfo.hpp"
#include "private/octree/EptSupport.hpp"
#include "private/octree/Types.hpp"

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


EptReader::EptReader() : m_eptArgs(new EptArgs), m_e(new EptPrivate), m_artifactMgr(nullptr)
{}

EptReader::~EptReader()
{}

std::string EptReader::getName() const { return s_info.name; }

void EptReader::addArgs(ProgramArgs& args)
{
    EptArgs &a = *m_eptArgs;
    addBaseArgs(args);
    args.add("origin", "Origin of source file to fetch", a.origin);
    args.add("addons", "Mapping of addon dimensions to their output directory", a.addons);
    args.add("header", "Header fields to forward with HTTP requests", a.headers);
    args.add("query", "Query parameters to forward with HTTP requests", a.query);
}


void EptReader::setForwards(StringMap& headers, StringMap& query)
{
    try
    {
        if (!m_eptArgs->headers.is_null())
            headers = m_eptArgs->headers.get<StringMap>();
    }
    catch (const std::exception& err)
    {
        throwError(std::string("Error parsing 'headers': ") + err.what());
    }

    try
    {
        if (!m_eptArgs->query.is_null())
            query = m_eptArgs->query.get<StringMap>();
    }
    catch (const std::exception& err)
    {
        throwError(std::string("Error parsing 'query': ") + err.what());
    }
}


QuickInfo EptReader::inspect()
{
    initialize();

    return inspectBase();
}


void EptReader::initialize()
{
    StringMap headers;
    StringMap query;
    setForwards(headers, query);
    m_p->connector.reset(new Connector(headers, query));

    try
    {
        m_e->adjFilename = m_filename;
        if (Utils::startsWith(m_e->adjFilename, "ept://"))
        {
            m_e->adjFilename = m_e->adjFilename.substr(6);
            if (!Utils::endsWith(m_e->adjFilename, "/ept.json"))
                m_e->adjFilename += "/ept.json";
        }
        m_e->info.reset(new EptInfo(m_p->connector->getJson(m_e->adjFilename)));
#ifndef PDAL_HAVE_ZSTD
        if (m_e->info->dataType() == EptInfo::DataType::Zstandard)
        {
            throwError("Cannot read Zstandard dataType: "
                "PDAL must be configured with WITH_ZSTD=On");
        }
#endif
        setSpatialReference(m_e->info->srs());
        m_e->addons = Addon::load(*m_p->connector, m_eptArgs->addons);
    }
    catch (const arbiter::ArbiterError& err)
    {
        throwError(err.what());
    }

    try
    {
        handleOriginQuery();
    }
    catch (std::exception& e)
    {
        throwError(e.what());
    }

    initializeBase();
}


void EptReader::handleOriginQuery()
{
    const std::string&  search = m_eptArgs->origin;

    if (search.empty())
        return;

    log()->get(LogLevel::Debug) << "Searching sources for " << search << std::endl;
    std::string filename = ept::sourcesDir(m_e->adjFilename) + "list.json";
    NL::json sources;
    try
    {
        sources = m_p->connector->getJson(filename);
    }
    catch (const arbiter::ArbiterError& err)
    {
        throwError(err.what());
    }

    if (!sources.is_array())
        throwError("Unexpected sources list: " + sources.dump());

    if (search.find_first_not_of("0123456789") == std::string::npos)
    {
        // If the origin search is integral, then the OriginId value has been
        // specified directly.
        m_e->queryOriginId = std::stoll(search);
    }
    else
    {
        // Otherwise it's a file path (or part of one - for example selecting
        // by a basename or a tile ID rather than a full path is convenient).
        // Find it within the sources list, and make sure it's specified
        // uniquely enough to select only one file.
        for (size_t i = 0; i < sources.size(); ++i)
        {
            const NL::json& el = sources.at(i);
            if (el["id"].get<std::string>().find(search) != std::string::npos)
            {
                if (m_e->queryOriginId != -1)
                    throwError("Origin search ID is not unique.");
                m_e->queryOriginId = static_cast<int64_t>(i);
            }
        }
    }

    if (m_e->queryOriginId == -1)
        throwError("Failed lookup of origin: " + search);

    if (m_e->queryOriginId >= (int64_t)sources.size())
        throwError("Origin ID larger than number of sources.");

    // Now that we have our OriginId value, clamp the bounds to select only the
    // data sources that overlap the selected origin.
    const NL::json found(sources.at(m_e->queryOriginId));

    try
    {
        BOX3D q(ept::toBox3d(found["bounds"]));

        if (m_p->clip.box.valid())
            m_p->clip.box.clip(q);
        else
            m_p->clip.box = q;

        log()->get(LogLevel::Debug) << "Query origin " << m_e->queryOriginId <<
            ": " << found["id"].get<std::string>() << std::endl;
    }
    catch (std::exception& e)
    {
        throwError(e.what());
    }
}


void EptReader::addDimensions(PointLayoutPtr layout)
{
    for (auto& el : m_e->info->dims())
    {
        const std::string& name = el.first;
        const DimType& dt = el.second;

        if (dt.m_xform.nonstandard())
            layout->registerOrAssignDim(name, Dimension::Type::Double);
        else
            layout->registerOrAssignDim(name, dt.m_type);
    }

    for (Addon& addon : m_e->addons)
        addon.setExternalId(layout->registerOrAssignDim(addon.name(), addon.type()));
}


void EptReader::ready(PointTableRef table)
{
    // These may not exist, in general they are only needed to track point
    // origins and ordering for an EPT writer.
    m_nodeIdDim = table.layout()->findDim("EptNodeId");
    m_pointIdDim = table.layout()->findDim("EptPointId");
    if (table.supportsView())
        m_artifactMgr = &table.artifactManager();

    readyBase(table);
}


void EptReader::calcOverlaps()
{
    EptAccessor rootAccessor(Key(), 0);

    m_p->hierarchy.reset(new Hierarchy(ept::hierarchyDir(m_e->adjFilename)));
    baseCalcOverlaps(*m_p->hierarchy, rootAccessor);

    for (auto& addon : m_e->addons)
        baseCalcOverlaps(addon.hierarchy(), rootAccessor);
}


//ABELL - Can't throw from these calls because they may execute in a worker thread. Fix.
HierarchyPage EptReader::fetchHierarchyPage(Hierarchy& hierarchy, const Accessor& acc) const
{
    NL::json j;

    if (acc.key() != Key() && m_e->hierarchyStep == 0)
        m_e->hierarchyStep = acc.key().d;
    try
    {
        std::string filename = hierarchy.source() + acc.key().toString() + ".json";
        j = m_p->connector->getJson(filename);
    }
    catch (const arbiter::ArbiterError& err)
    {
        throwError(err.what());
    }

    if (!j.is_object())
        throwError("Invalid EPT hierarchy page for key '" + acc.key().toString() + "'.\n");

    HierarchyPage page;
    for (auto& el : j.items())
    {
        NL::json k = el.key();
        NL::json v = el.value();
        if (!k.is_string())
        {
            log()->get(LogLevel::Error) << "Non-string key found in EPT hierarchy - "
                "skipping.\n";
            continue;
        }
        std::string skey = k.get<std::string>();
        Key key(skey);
        if (!key.valid())
        {
            log()->get(LogLevel::Error) << "Invalid key '" << skey << "' found in EPT " <<
                "hierarchy - skipping.\n";
            continue;
        }

        if (!v.is_number_integer())
        {
            log()->get(LogLevel::Error) << "Invalid point count found in EPT hierarchy " <<
                " for key '" << skey << "' - skipping.\n";
            continue;
        }
        int64_t count = v.get<int64_t>();
        if (count < -1 || count > (std::numeric_limits<int32_t>::max)())
        {
            log()->get(LogLevel::Error) << "Point count '" << count << "' out of range for " <<
                "key '" << skey << "' in EPT hierarchy - skipping.\n";
            continue;
        }
        page.insert(AccessorPtr(new EptAccessor(key, (int32_t)count)));
    }
    return page;
}


double EptReader::rootNodeHalfWidth() const
{
    const BOX3D& extent = m_e->info->rootExtent();
    return (extent.maxx - extent.minx) / 2;
}


void EptReader::rootNodeCenter(double& x, double& y, double& z) const
{
    const BOX3D& extent = m_e->info->rootExtent();
    x = (extent.maxx + extent.minx) / 2;
    y = (extent.maxy + extent.miny) / 2;
    z = (extent.maxz + extent.minz) / 2;
}


double EptReader::rootNodeSpacing() const
{
    const BOX3D& extent = m_e->info->rootExtent();
    return (extent.maxx - extent.minx) / m_e->info->span();
}


BOX3D EptReader::rootNodeExtent() const
{
    return m_e->info->rootExtent();
}


BOX3D EptReader::pointBounds() const
{
    return m_e->info->pointBounds();
}


point_count_t EptReader::pointCount() const
{
    return m_e->info->points();
}


StringList EptReader::dimNames() const
{
    StringList names;

    for (auto& p : m_e->info->dims())
        names.push_back(p.first);
    return names;
}


TilePtr EptReader::makeTile(const Accessor& accessor) const
{
    const EptAccessor& eptAccessor = static_cast<const EptAccessor &>(accessor);
    return TilePtr(new EptTile(eptAccessor, *m_e->info, m_e->addons));
}


bool EptReader::passesPointFilter(PointRef& p, double x, double y, double z) const
{
    int64_t originId = p.getFieldAs<int64_t>(Dimension::Id::OriginId);
    if (m_e->queryOriginId != -1 && originId != m_e->queryOriginId)
        return false;

    return passesBasePointFilter(p, x, y, z);
}


// This code runs in a single thread, so doesn't need locking.
bool EptReader::processPoint(const Tile& tile, PointRef& src, PointRef& dst)
{
    using namespace Dimension;

    if (!passesPointFilter(src, src.getFieldAs<double>(Id::X), src.getFieldAs<double>(Id::Y),
            src.getFieldAs<double>(Id::Z)))
        return false;

    for (auto& el : m_e->info->dims())
    {
        const DimType& dt = el.second;
        dst.setField(dt.m_id, src.getFieldAs<double>(dt.m_id));
    }
    // Special EPT fields.
    const EptTile& eptTile = static_cast<const EptTile &>(tile);
    dst.setField(m_nodeIdDim, eptTile.nodeId());
    dst.setField(m_pointIdDim, src.pointId());

    for (Addon& addon : m_e->addons)
    {
        Dimension::Id srcId = addon.localId();

        //ABELL - It's dumb where we have to do a map search for each addon. Fix somehow.
        BasePointTable *t = eptTile.addonTable(srcId);
        if (!t)
            continue;
        PointRef addonPoint(*t, src.pointId());
        double val = addonPoint.getFieldAs<double>(srcId);
        dst.setField(addon.externalId(), val);
    }
    return true;
}


point_count_t EptReader::read(PointViewPtr view, point_count_t count)
{
    point_count_t numRead = baseRead(view, count);

    // If we're using the addon writer, transfer the info and hierarchy
    // to that stage.
    if (m_nodeIdDim != Dimension::Id::Unknown)
    {
        EptArtifactPtr artifact
            (new EptArtifact(std::move(m_e->info), std::move(m_p->hierarchy),
                std::move(m_p->connector), m_e->hierarchyStep));
        m_artifactMgr->put("ept", artifact);
    }

    return numRead;
}

} // namespace pdal
