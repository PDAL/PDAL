/******************************************************************************
* Copyright (c) 2018, Hobu Inc.
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
*     * Neither the name of Hobu, Inc. nor the
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

#include "InfoFilter.hpp"

#include <cmath>

#include <pdal/PDALUtils.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "filters.info",
    "Gather basic info about points.",
    "http://pdal.org/stages/filters.info.html"
};

CREATE_STATIC_STAGE(InfoFilter, s_info)

std::string InfoFilter::getName() const { return s_info.name; }

void InfoFilter::addArgs(ProgramArgs& args)
{
    args.add("point,p", "Point to dump\n--point=\"1-5,10,100-200\" (0 indexed)",
        m_pointSpec);
    args.add("query",
         "Return points in order of distance from the specified "
         "location (2D or 3D)\n"
         "--query Xcoord,Ycoord[,Zcoord][/count]",
         m_querySpec);
}


void InfoFilter::parsePointSpec()
{
    auto parseInt = [this](const std::string& s)
    {
        uint32_t i;

        if (!Utils::fromString(s, i))
            throwError("Invalid integer '" + s + "in 'point' option");
        return i;
    };

    auto addRange = [this, &parseInt](const std::string& begin,
        const std::string& end)
    {
        PointId low = parseInt(begin);
        PointId high = parseInt(end);
        if (low > high)
            throwError("Invalid range in 'point' option: '" +
            begin + "-" + end);
        while (low <= high)
            m_idList.push_back(low++);
    };

    Utils::trim(m_pointSpec);

    StringList ranges = Utils::split2(m_pointSpec, ',');
    for (std::string& s : ranges)
    {
        StringList limits = Utils::split(s, '-');
        if (limits.size() == 1)
            m_idList.push_back(parseInt(limits[0]));
        else if (limits.size() == 2)
            addRange(limits[0], limits[1]);
        else
            throwError("Invalid point range in 'point' option: " + s);
    }
}


void InfoFilter::parseQuerySpec()
{
    // See if there's a provided point count.
    StringList parts = Utils::split2(m_querySpec, '/');
    if (parts.size() == 2)
    {
        if (!Utils::fromString(parts[1], m_queryCount))
            throwError("Invalid query count in 'query' option: " + parts[1]);
    }
    else if (parts.size() != 1)
        throwError("Invalid point location specification. Sytax: "
            "--query=\"X,Y[/count]\"");

    //ABELL - This doesn't match syntax below, but keeping because
    // history.
    auto seps = [](char c) { return (c == ',' || c == '|' || c == ' '); };

    StringList tokens = Utils::split2(parts[0], seps);
    if (tokens.size() != 2 && tokens.size() != 3)
        throwError("Invalid point location specification. Sytax: "
            "--query=\"X,Y[/count]\"");

    bool ok = true;
    ok &= Utils::fromString(tokens[0], m_queryX);
    ok &= Utils::fromString(tokens[1], m_queryY);
    if (tokens.size() == 3)
        ok &= Utils::fromString(tokens[1], m_queryZ);
    if (!ok)
        throwError("Invalid point location specification. Sytax: "
            "--query=\"X,Y[/count]\"");
}


void InfoFilter::prepared(PointTableRef table)
{
    m_dims = table.layout()->dimTypes();
    m_pointSize = table.layout()->pointSize();
    if (m_pointSpec.size())
        parsePointSpec();
    if (m_querySpec.size())
        parseQuerySpec();
}


void InfoFilter::initialize(PointTableRef table)
{
    getMetadata().add(table.layout()->toMetadata());
}


void InfoFilter::ready(PointTableRef)
{
    m_count = 0;
    m_idCur = m_idList.begin();
}


void InfoFilter::filter(PointView& view)
{
    PointRef point(view, 0);
    for (PointId idx = 0; idx < view.size(); ++idx)
    {
        point.setPointId(idx);
        processOne(point);
    }
}


bool InfoFilter::processOne(PointRef& point)
{
    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);
    double z = point.getFieldAs<double>(Dimension::Id::Z);

    // Accumulate min/max.
    m_bounds.grow(x, y, z);

    // Create metadata for requsted points.
    // We may get a point list or a query list, but not both.
    if (m_idCur != m_idList.end() && *m_idCur == m_count)
    {
        // This is kind of a fake, in that the points in the list aren't
        // near anything, but it makes for a single set of code to
        // extract the metadata of reported points.
        std::vector<char> buf(m_pointSize);
        point.getPackedData(m_dims, buf.data());
        m_results.emplace_back(m_count, 0, std::move(buf));
        m_idCur++;
    }
    else if (m_querySpec.size() && m_queryCount)
    {
        double dist = std::pow(x - m_queryX, 2) + std::pow(y - m_queryY, 2);
        if (!std::isnan(m_queryZ))
            dist += std::pow(z - m_queryZ, 2);
        if (m_results.size() < m_queryCount || dist < m_results.back().m_dist)
        {
            std::vector<char> buf(m_pointSize);
            point.getPackedData(m_dims, buf.data());
            NearPoint np(m_count, dist, std::move(buf));
            m_results.insert(
                std::upper_bound(m_results.begin(), m_results.end(), np),
                std::move(np));
            if (m_results.size() > m_queryCount)
                m_results.pop_back();
        }
    }
    m_count++;
    return true;
}


void InfoFilter::done(PointTableRef table)
{
    // Point list
    MetadataNode points("points");
    for (NearPoint& np: m_results)
    {
        MetadataNode node("point");
        const char *buf = np.m_data.data();
        Everything e;
        for (DimType& dt : m_dims)
        {
            size_t dimSize = Dimension::size(dt.m_type);
            std::copy(buf, buf + dimSize, reinterpret_cast<char *>(&e));
            double d = Utils::toDouble(e, dt.m_type);
            node.add(table.layout()->dimName(dt.m_id), d);
            buf += dimSize;
        }
        node.add("PointId", np.m_id);
        points.add(node);
    }
    if (points.hasChildren())
        getMetadata().add(points);

    // Bounds
    getMetadata().add(Utils::toMetadata(m_bounds));

    // Point count
    getMetadata().add("num_points", m_count);

    // Dimension names
    std::string dims;
    for (auto di = m_dims.begin(); di != m_dims.end();)
    {
        dims += table.layout()->dimName(di->m_id);
        if (++di != m_dims.end())
            dims += ", ";
    }
    getMetadata().add("dimensions", dims);

    // Spatial reference
    getMetadata().add(table.anySpatialReference().toMetadata());
}

} // namespace pdal
