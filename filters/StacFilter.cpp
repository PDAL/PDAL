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
    args.add("filename", "Input file.",  m_filename);
    args.add("default_srs", "Default SRS of the file if no SRS is found.",
        m_defaultSrs);
    args.add("pctype", "Pointcloud type, adjusts 'pc:type' key in properties"
        " section of STAC Feature. Available options: lidar, eopc, radar,"
        " sonar, other. Default: 'lidar'", m_pcType, "lidar");
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
    std::string stem = FileUtils::stem(m_filename);
    std::string absPath = FileUtils::toAbsolutePath(m_filename);

    //Base STAC object
    MetadataNode id = m_metadata.add("id", stem);
    MetadataNode properties = m_metadata.add("properties");
    properties.add("datetime", "2022-11-15T16:06:02.616469Z");
    m_metadata.add("type", "Feature");
    m_metadata.add("stac_version", "1.0.0");

    //extensions - pointcloud and projection extensions
    m_metadata.add("extensions", "https://stac-extensions.github.io/pointcloud/v1.0.0/schema.json");
    m_metadata.add("extensions", "https://stac-extensions.github.io/projection/v1.1.0/schema.json");

    //links
    MetadataNode self = m_metadata.addList("links");
    self.add("rel", "derived_from");
    self.add("href", absPath);

    //assets - add source file to data asset
    MetadataNode assets = m_metadata.add("assets");
    MetadataNode data;
    data.add("href", absPath);
    data.add("title", "Lidar data");
    assets.add(data.clone("data"));

    pointcloud(properties, table);
    projection(properties, table);

}

//Handle the pointcloud extension
void StacFilter::pointcloud(MetadataNode& properties, PointTableRef& table)
{
    std::string fileExt = FileUtils::extension(m_filename);
    uint32_t position(0);
    point_count_t count = 0;
    bool bNoPoints(true);

    //Add dimension statistics
    for (auto di = m_stats.begin(); di != m_stats.end(); ++di)
    {
        Summary& s = di->second;

        bNoPoints = (bool)s.count();
        count = s.count();

        MetadataNode pcStats = properties.addList("pc:statistics");
        pcStats.add("position", position++);
        s.extractMetadata(pcStats);
    }
    if (!bNoPoints)
        throw pdal_error("No points found!");
    properties.add("pc:count", count);
    properties.add("pc:type", "lidar");
    properties.add("pc:encoding", fileExt);
    auto dims = table.layout()->toMetadata().children();
    for (auto& c: dims)
        properties.add(c.clone("pc:schemas"));

}

//Handle projection extension and geometry/bbox calculations
void StacFilter::projection(MetadataNode& properties, PointTableRef& table)
{
    //Make sure base x, y, z components exist and there are points to look at
    auto xs = m_stats.find(Dimension::Id::X);
    auto ys = m_stats.find(Dimension::Id::Y);
    auto zs = m_stats.find(Dimension::Id::Z);
    if (xs == m_stats.end() || ys == m_stats.end() || zs == m_stats.end())
    {
        throw pdal_error("Missing x, y, or z component.");
    }

    //Projection and base geometry/bbox info. Base Bbox and geometry in stac
    //should be in 4326, and anthing prefaced with 'proj' should be in the
    //native srs. If there is no srs derived from the file, default it to 4326.
    BOX3D box(xs->second.minimum(), ys->second.minimum(),
        zs->second.minimum(), xs->second.maximum(), ys->second.maximum(),
        zs->second.maximum());
    Polygon p(box);

    addBox(properties, "proj:bbox", box);

    MetadataNode projgeom = properties.addWithType("proj:geometry",
        p.json(), "json", "GeoJSON boundary");
    SpatialReference ref = table.anySpatialReference();
    if (ref.empty())
    {
        if (m_defaultSrs.empty())
            throw pdal_error("Missing SRS, and no default SRS was provided.");
        ref.set(m_defaultSrs);
        if (!ref.valid())
            throw pdal_error("User defined default SRS is invalid.");
    }

    p.setSpatialReference(ref);
    properties.add("proj:wkt2", ref.getWKT2());
    properties.addWithType("proj:projjson", ref.getPROJJSON(), "json",
        "PROJ JSON");

    auto status = p.transform("EPSG:4326");
    if (status)
    {
        BOX3D ddbox = p.bounds();
        m_metadata.addWithType("geometry", p.json(), "json", "GeoJSON boundary");
        addBox(m_metadata, "bbox", ddbox);
    }
    else
        throw pdal_error(status.what());

}


}//pdal