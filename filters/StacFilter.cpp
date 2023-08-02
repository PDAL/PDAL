/******************************************************************************
* Copyright (c) 2023, Kyle Mann (kyle@hobu.com)
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

#include "StacFilter.hpp"
#include "StatsFilter.hpp"
#include <pdal/Polygon.hpp>
#include <pdal/util/FileUtils.hpp>


namespace pdal
{

using namespace stats;

static PluginInfo const s_info = PluginInfo(
    "filters.stac",
    "Collect information on lidar data and form STAC compatible GeoJSON Feature.",
    "http://pdal.io/stages/filters.stac.html" );

StacFilter::StacFilter() {};
StacFilter::~StacFilter() {};

CREATE_STATIC_STAGE(StacFilter, s_info);

std::string StacFilter::getName() const {
    return s_info.name;
}

void StacFilter::addArgs(ProgramArgs& args)
{
    args.add("input_file", "Input file.",  m_inputFile);
}

void StacFilter::filter(PointView& view)
{
    PointRef point(view, 0);
    for (PointId idx = 0; idx < view.size(); ++idx)
    {
        point.setPointId(idx);
        processOne(point);
    }
}

bool StacFilter::processOne(PointRef& point)
{
    for (auto p = m_stats.begin(); p != m_stats.end(); ++p)
    {
        Dimension::Id d = p->first;
        Summary& c = p->second;
        c.insert(point.getFieldAs<double>(d));
    }
    return true;
}

void StacFilter::prepared(PointTableRef table)
{
    PointLayoutPtr layout(table.layout());

    for (auto& id : layout->dims())
    {
        m_stats.insert(std::make_pair(id,
            Summary(layout->dimName(id), Summary::NoEnum, true)));
    }
}

void StacFilter::done(PointTableRef table)
{
    extractMetadata(table);
}

void addBox(MetadataNode& n, std::string name, const BOX3D& box)
{
    n.add(name, box.minx);
    n.add(name, box.miny);
    n.add(name, box.minz);
    n.add(name, box.maxx);
    n.add(name, box.maxy);
    n.add(name, box.maxz);
}

void StacFilter::extractMetadata(PointTableRef table)
{
    std::string stem = FileUtils::stem(m_inputFile);
    std::string fileExt = FileUtils::extension(m_inputFile);

    //Base STAC object
    MetadataNode properties = m_metadata.add("properties");
    MetadataNode id = m_metadata.add("id", stem);

    //extensions - pointcloud and projection extensions
    m_metadata.add("extensions", "https://stac-extensions.github.io/pointcloud/v1.0.0/schema.json");
    m_metadata.add("extensions", "https://stac-extensions.github.io/projection/v1.1.0/schema.json");

    //links - empty because this is defaulting to stdout.
    MetadataNode self = m_metadata.addList("links");
    self.add("rel", "derived_from");
    self.add("target", m_inputFile);

    //assets - add source file to data asset
    MetadataNode assets = m_metadata.add("assets");
    MetadataNode data;
    data.add("href", m_inputFile);
    data.add("title", "Lidar data");
    assets.add(data.clone("data"));

    uint32_t position(0);
    point_count_t count = 0;
    bool bNoPoints(true);
    for (auto di = m_stats.begin(); di != m_stats.end(); ++di)
    {
        Summary& s = di->second;

        bNoPoints = (bool)s.count();
        count = s.count();

        // MetadataNode pcStats = properties.addList("pc:statistics");
        // pcStats.add("position", position++);
        // s.extractMetadata(pcStats);
    }

    //POINTCLOUD EXTENSIONS
    properties.add("pc:count", count);
    properties.add("pc:type", "lidar");
    properties.add("pc:encoding", fileExt);

    // If we have X, Y, & Z dims, output bboxes
    auto xs = m_stats.find(Dimension::Id::X);
    auto ys = m_stats.find(Dimension::Id::Y);
    auto zs = m_stats.find(Dimension::Id::Z);
    if (xs != m_stats.end() &&
        ys != m_stats.end() &&
        zs != m_stats.end() &&
        bNoPoints)
    {
        BOX3D box(xs->second.minimum(), ys->second.minimum(),
            zs->second.minimum(), xs->second.maximum(), ys->second.maximum(),
            zs->second.maximum());
        Polygon p(box);
        addBox(properties, "proj:bbox", box);

        // MetadataNode bbox = Utils::toMetadata(box);
        // MetadataNode metadata = box_metadata.add("native");

        MetadataNode projgeom = properties.addWithType("proj:geometry",
            p.json(), "json", "GeoJSON boundary");
        // MetadataNode bbox = metadata.add(mbox);
        SpatialReference ref = table.anySpatialReference();

        // if we don't get an SRS from the PointTableRef,
        // we won't add another metadata node
        if (!ref.empty())
        {
            p.setSpatialReference(ref);
            properties.add("proj:wkt2", ref.getWKT2());
            // properties.addWithType("proj:projjson", ref.getPROJJSON(), "json",
            //     "PROJ JSON");

            if (p.transform("EPSG:4326"))
            {
                BOX3D ddbox = p.bounds();
                // MetadataNode epsg_4326_box = Utils::toMetadata(ddbox);
                m_metadata.addWithType("geometry", p.json(), "json", "GeoJSON boundary");
                addBox(m_metadata, "bbox", ddbox);
            }
        }
        else {
            SpatialReference r("EPSG:4326");
            properties.add("proj:epsg", 4326);
            properties.add("proj:wkt2", r.getWKT2());
            m_metadata.addWithType("geometry", p.json(), "json", "GeoJSON boundary");
            addBox(m_metadata, "bbox", box);
            // properties.addWithType("proj:projjson", r.getPROJJSON(), "json",
            //     "PROJ JSON");

        }
    }

}

}//pdal