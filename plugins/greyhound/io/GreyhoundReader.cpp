/******************************************************************************
* Copyright (c) 2014, Connor Manning (connor@hobu.co)
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
#include "bbox.hpp"
#include "dir.hpp"
#include <pdal/pdal_macros.hpp>
#include <pdal/Compression.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "readers.greyhound",
    "Greyhound Reader",
    "http://pdal.io/stages/readers.greyhound.html" );

CREATE_SHARED_PLUGIN(1, 0, GreyhoundReader, Reader, s_info)

std::string GreyhoundReader::getName() const { return s_info.name; }

GreyhoundReader::GreyhoundReader()
    : Reader()
    , m_url()
    , m_resource()
    , m_numPoints(0)
    , m_index(0)
    , m_depthBegin(0)
    , m_depthEnd(std::numeric_limits<uint32_t>::max())
    , m_baseDepth(0)
    , m_stopSplittingDepth(0)
    , m_split(0)
    , m_retryCount(1)
    , m_timeout(0)
    , m_splitCountThreshold(0)
{ }

GreyhoundReader::~GreyhoundReader()
{
}

DimTypeList GreyhoundReader::getSchema(const Json::Value& jsondata) const
{
    DimTypeList output;

    if (jsondata.isMember("schema") &&
        jsondata["schema"].isArray())
    {
        Json::Value jsonDimArray(jsondata["schema"]);

        for (std::size_t i(0); i < jsonDimArray.size(); ++i)
        {
            const Json::Value& jsonDim(
                    jsonDimArray[static_cast<Json::ArrayIndex>(i)]);

            const Dimension::Id id(
                    Dimension::id(jsonDim["name"].asString()));

            const Dimension::Type type(
                static_cast<Dimension::Type>(
                    static_cast<int>(Dimension::fromName(
                        jsonDim["type"].asString())) |
                    std::stoi(jsonDim["size"].asString())));

            DimType d;
            d.m_id = id;
            d.m_type = type;
            output.push_back(d);
        }
    }

    return output;
}
BOX3D GreyhoundReader::getBounds(const Json::Value& jsondata, const std::string& memberName) const
{
    BOX3D output;
    if (jsondata.isMember(memberName) &&
        jsondata[memberName].isArray())
    {
        Json::Value bounds(jsondata[memberName]);

        output.minx = bounds[0].asDouble();
        output.miny = bounds[1].asDouble();
        output.minz = bounds[2].asDouble();
        output.maxx = bounds[3].asDouble();
        output.maxy = bounds[4].asDouble();
        output.maxz = bounds[5].asDouble();
    }
    else
    {
        throw pdal_error("Greyhound info response has no \"" + memberName + "\" member");
    }

    return output;
}

Json::Value GreyhoundReader::fetch(const std::string& url) const
{
    Json::Value config;
    if (log()->getLevel() > LogLevel::Debug4)
        config["arbiter"]["verbose"] = true;
    config["http"]["timeout"] = m_timeout;
    arbiter::Arbiter a(config);
    auto response = a.get(url);

    Json::Value jsonResponse;
    Json::Reader jsonReader;
    jsonReader.parse(response, jsonResponse);

    return jsonResponse;

}


void GreyhoundReader::initialize(PointTableRef table)
{
    std::string info_url = m_url + "/resource/" + m_resource + "/info";
    log()->get(LogLevel::Info) << "fetching info URL " << info_url << std::endl;

    m_resourceInfo = fetch(info_url);

    m_dimData = getSchema(m_resourceInfo);
    m_conformingBounds= getBounds(m_resourceInfo, "boundsConforming");
    m_stopSplittingDepth = std::log(m_resourceInfo["numPoints"].asInt64()) / std::log(4);

    std::string srs = m_resourceInfo["srs"].asString();
    setSpatialReference(SpatialReference(srs));

    m_baseDepth = m_resourceInfo["baseDepth"].asUInt();

    PointLayoutPtr layout = table.layout();

}

pdal::greyhound::BBox makeBox(BOX3D bounds)
{
    pdal::greyhound::Point minimum;
    pdal::greyhound::Point maximum;

    minimum.x = bounds.minx;
    minimum.y = bounds.miny;
    minimum.z = bounds.minz;

    maximum.x = bounds.maxx;
    maximum.y = bounds.maxy;
    maximum.z = bounds.maxz;
    pdal::greyhound::BBox box(minimum, maximum, true);
    return box;

}
BOX3D zoom(BOX3D query, BOX3D fullBox, int& split)
{

    pdal::greyhound::BBox queryBox = makeBox(query);
    pdal::greyhound::BBox currentBox = makeBox(fullBox);

    while (currentBox.contains(queryBox))
    {
        currentBox.go(pdal::greyhound::getDirection(queryBox.mid(), currentBox.mid()));
        split++;
    }

    BOX3D output;
    output.minx = currentBox.min().x; output.maxx = currentBox.max().x;
    output.miny = currentBox.min().y; output.maxy = currentBox.max().y;
    output.minz = currentBox.min().z; output.maxz = currentBox.max().z;

    return output;
}

uint64_t sumHierarchy(const Json::Value& tree)
{
    uint64_t output(0);
    if (!tree.isMember("n")) return output;

    output += tree["n"].asUInt64();

    auto summarize = [tree](const std::string& name)
    {
        uint64_t output(0);
        if (tree.isMember(name))
        {
            output += tree[name]["n"].asUInt64();
            output += sumHierarchy(tree[name]);
        }
        return output;
    };

    output += summarize("nwu");
    output += summarize("neu");
    output += summarize("swu");
    output += summarize("seu");
    output += summarize("nwd");
    output += summarize("ned");
    output += summarize("swd");
    output += summarize("sed");

    return output;
}


QuickInfo GreyhoundReader::inspect()
{
    QuickInfo qi;
    std::unique_ptr<PointLayout> layout(new PointLayout());

    PointTable table;
    initialize(table);
    addDimensions(layout.get());

    Dimension::IdList dims = layout->dims();
    for (auto di = dims.begin(); di != dims.end(); ++di)
        qi.m_dimNames.push_back(layout->dimName(*di));
    qi.m_srs = getSpatialReference();
    qi.m_valid = true;

    int split(0);

    BOX3D fullBounds = getBounds(m_resourceInfo, "bounds");
    BOX3D currentBounds = zoom(m_queryBounds, fullBounds, split);
    m_split = split;

    uint32_t depthBegin = std::max(m_depthBegin, m_baseDepth + 1 + m_split);
    uint32_t depthEnd = 28;

    Json::Value response = fetchHierarchy(currentBounds, depthBegin, depthEnd);
    uint64_t count = sumHierarchy(response);

    qi.m_pointCount = count;
    qi.m_bounds = getBounds(m_resourceInfo, "boundsConforming");

    done(table);

    return qi;
}

void GreyhoundReader::addArgs(ProgramArgs& args)
{
    args.add("url", "URL", m_url);
    args.add("resource", "Resource ID", m_resource);
    args.add("timeout", "Request timeout (milliseconds)", m_timeout, 60000u);
    args.add("bounds", "Bounding cube", m_queryBounds);
    args.add("depth_begin", "Beginning depth to query", m_depthBegin);
    args.add("depth_end", "Ending depth to query", m_depthEnd);
    args.add("retries", "How many times to retry", m_retryCount, 1u);
    args.add("split_threshold", "Point count for which to start splitting queries", m_splitCountThreshold, 50000llu);
}


void GreyhoundReader::addDimensions(PointLayoutPtr layout)
{
    for (auto& dim: m_dimData)
    {
        layout->registerDim(dim.m_id, dim.m_type);
    }
}




std::string stringifyBounds(BOX3D bounds)
{
    std::stringstream sbounds;
    sbounds << std::fixed;
    sbounds << "[" << bounds.minx << "," << bounds.miny << "," << bounds.minz;
    sbounds << "," << bounds.maxx << "," << bounds.maxy << "," << bounds.maxz << "]";
    return sbounds.str();
}

Json::Value GreyhoundReader::fetchHierarchy(BOX3D bounds, uint32_t depthBegin, uint32_t depthEnd)  const
{

    std::stringstream url;
    url << m_url << "/resource/" << m_resource;
    url << "/hierarchy?bounds=" << arbiter::http::sanitize(stringifyBounds(bounds));
    url << "&depthBegin=" << depthBegin;
    url << "&depthEnd=" << depthEnd;

    log()->get(LogLevel::Info) << "fetching hierarchy URL " << url.str() << std::endl;

    Json::Value response = fetch(url.str());
    return response;
}

void GreyhoundReader::ready(PointTableRef)
{


}


point_count_t GreyhoundReader::readDirection(const greyhound::BBox& currentBox,
                                            const greyhound::BBox& queryBox,
                                            uint32_t& depthBegin,
                                            uint32_t& depthEnd,
                                            point_count_t count,
                                            PointViewPtr view,
                                            const Json::Value& hierarchy)
{

    point_count_t output(0);
    if (!currentBox.overlaps(queryBox))
        return output;

    using namespace pdal::greyhound;

    auto makeDirBox = [](greyhound::BBox box, greyhound::Dir direction)
    {
        BBox dirBox = box.get(direction);
        return dirBox;
    };



    if (currentBox.overlaps(queryBox))
    {
        BOX3D currentBounds;
        currentBounds.minx = currentBox.min().x; currentBounds.maxx = currentBox.max().x;
        currentBounds.miny = currentBox.min().y; currentBounds.maxy = currentBox.max().y;
        currentBounds.minz = currentBox.min().z; currentBounds.maxz = currentBox.max().z;

        Json::Value hierarchy = fetchHierarchy(currentBounds, depthBegin, depthEnd);
        point_count_t belowUs = sumHierarchy(hierarchy);
        point_count_t currentLevel(0);
        if (hierarchy.isMember("n"))
            currentLevel = hierarchy["n"].asUInt64();

        if (belowUs  > m_splitCountThreshold )
        {
            Json::Value hierarchy = fetchHierarchy(currentBounds, depthBegin, depthEnd);
            point_count_t belowUs = sumHierarchy(hierarchy);
            if (hierarchy.isMember("swd"))
                output += readDirection(makeDirBox(currentBox, Dir::swd), queryBox, depthBegin, depthEnd, count, view, hierarchy["swd"]);
            if (hierarchy.isMember("sed"))
                output += readDirection(makeDirBox(currentBox, Dir::sed), queryBox, depthBegin, depthEnd, count, view, hierarchy["sed"]);
            if (hierarchy.isMember("nwd"))
                output += readDirection(makeDirBox(currentBox, Dir::nwd), queryBox, depthBegin, depthEnd, count, view, hierarchy["nwd"]);
            if (hierarchy.isMember("ned"))
                output += readDirection(makeDirBox(currentBox, Dir::ned), queryBox, depthBegin, depthEnd, count, view, hierarchy["ned"]);
            if (hierarchy.isMember("swu"))
                output += readDirection(makeDirBox(currentBox, Dir::swu), queryBox, depthBegin, depthEnd, count, view, hierarchy["swu"]);
            if (hierarchy.isMember("seu"))
                output += readDirection(makeDirBox(currentBox, Dir::seu), queryBox, depthBegin, depthEnd, count, view, hierarchy["seu"]);
            if (hierarchy.isMember("nwu"))
                output += readDirection(makeDirBox(currentBox, Dir::nwu), queryBox, depthBegin, depthEnd, count, view, hierarchy["nwu"]);
            if (hierarchy.isMember("neu"))
                output += readDirection(makeDirBox(currentBox, Dir::neu), queryBox, depthBegin, depthEnd, count, view, hierarchy["neu"]);
        }
        else
        {
            if (belowUs)
                output += this->readLevel(view, count, currentBounds, depthBegin, depthEnd);
        }

    }

    return output;

};



point_count_t GreyhoundReader::read(
        PointViewPtr view,
        const point_count_t count)
{
    point_count_t output(0);
    using namespace pdal::greyhound;

    // if the base depth is greater than
    // what the user gave use, we use that
    // if it isn't, we use base depth + 1 (base depth has 0 points)
//


    int split(0);

    BOX3D fullBounds = getBounds(m_resourceInfo, "bounds");
    BOX3D currentBounds = zoom(m_queryBounds, fullBounds, split);
    m_split = split;

    uint32_t depthBegin = std::max(m_depthBegin, m_baseDepth + 1 + m_split);
    uint32_t depthEnd = depthBegin + 1;

    Json::Value hierarchy = fetchHierarchy(currentBounds, depthBegin, depthEnd);

    point_count_t belowUs = sumHierarchy(hierarchy);
    if (!belowUs)
        return output;

    BBox queryBox = makeBox(m_queryBounds);
    BBox currentBox = makeBox(currentBounds);

    while (depthEnd <= m_depthEnd)
    {

        if (depthEnd >= m_stopSplittingDepth)
        {
            depthEnd = m_depthEnd;
        }
        output += readDirection(currentBox, queryBox, depthBegin, depthEnd, count, view, hierarchy);
        depthBegin++;
        depthEnd = depthBegin + 1;
    }

    return output;

}

point_count_t GreyhoundReader::readLevel(
        PointViewPtr view,
        const point_count_t count,
        BOX3D bounds,
        uint32_t depthBegin,
        uint32_t depthEnd)
{


    std::string bounds_str = stringifyBounds(bounds);
    std::stringstream url;
    url << m_url << "/resource/" << m_resource;
    url << "/read?bounds=" << arbiter::http::sanitize(stringifyBounds(bounds));
    url << "&depthBegin=" << depthBegin;
    url << "&depthEnd=" << depthEnd;

#ifdef PDAL_HAVE_LAZPERF
    url << "&compress=true";
#endif

    log()->get(LogLevel::Info) << "fetching read URL " << url.str() << std::endl;

    Json::Value config;
    if (log()->getLevel() > LogLevel::Debug4)
        config["arbiter"]["verbose"] = true;
    config["http"]["timeout"] = 20000;
    arbiter::Arbiter a(config);
    uint32_t retries(0);
    std::vector<char> response;
    for (uint32_t i = 0; i <= m_retryCount; ++i)
    {
        try
        {
            response = a.getBinary(url.str());
            break;
        } catch (arbiter::ArbiterError&)
        {
            continue;
        }
    }

    PointId nextId = view->size();
    point_count_t numRead = 0;

    log()->get(LogLevel::Info) << "Fetched "
                               << response.size()
                               << " bytes from "
                               << m_url << std::endl;
    if (!response.size())
    {
        return numRead;
    }

    const uint32_t numPoints = *reinterpret_cast<const uint32_t*>(response.data() + response.size() - sizeof(uint32_t));

    log()->get(LogLevel::Info) << "Fetched "
                               << response.size()
                               << " bytes and "
                               << numPoints << " points from"
                               << m_url << std::endl;

#ifdef PDAL_HAVE_LAZPERF
    SignedLazPerfBuf buf(response);
    LazPerfDecompressor<SignedLazPerfBuf> decompressor(buf, m_dimData);

    std::vector<char> ptBuf(decompressor.pointSize());
    while (numRead < numPoints)
    {
        char* outbuf = ptBuf.data();
        point_count_t numWritten =
            decompressor.decompress(outbuf, ptBuf.size());

        double x(0.0); double y(0.0); double z(0.0);

        for (auto di = m_dimData.begin(); di != m_dimData.end(); ++di)
        {
            view->setField(di->m_id, di->m_type, nextId, outbuf);
            outbuf += Dimension::size(di->m_type);
        }

        x = view->getFieldAs<double>(Dimension::Id::X, nextId);
        y = view->getFieldAs<double>(Dimension::Id::Y, nextId);
        z = view->getFieldAs<double>(Dimension::Id::Z, nextId);

        if (m_queryBounds.contains(x,y, z))
        {
            // overwrite this point id if we were not inside
            // the box
            nextId++;
        }

        numRead++;

        if (m_cb)
            m_cb(*view, nextId);
    }
#else

    raise pdal_error("uncompressed not implemented!");
#endif
    return numRead;
}

bool GreyhoundReader::eof() const
{
    return m_index >= m_numPoints;
}

void GreyhoundReader::done(PointTableRef)
{
}

} // namespace pdal

