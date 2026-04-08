/******************************************************************************
 * Copyright (c) 2023, Kyle Mann (kyle@hobu.co)
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
 *     * Neither the name of the Martin Isenburg or Iowa Department
 *       of Natural Resources nor the names of its contributors may be
 *       used to endorse or promote products derived from this software
 *       without specific prior written permission.
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

#include <pdal/PDALUtils.hpp>
#include <pdal/util/FileUtils.hpp>

namespace pdal
{

class StacKey
{
public:
    StacKey(std::string key):
        m_key(key)
        {};
    ~StacKey(){};

    inline bool operator()(const MetadataNode& n) {return n.name() == m_key;};
private:
    std::string m_key;
};

inline MetadataNode getChild(MetadataNode& m, std::string key)
{
    StacKey sk(key);
    MetadataNode n = m.findChild(sk);
    if (n.empty())
        throw std::out_of_range(key);
    else
        return n;
}

inline std::string getDateStr(std::string year, std::string doy)
{
    std::tm tm = { };
    tm.tm_mday = std::stoi(doy);
    tm.tm_mon = 0;
    tm.tm_year = std::stoi(year)-1900;
    tm.tm_isdst = -1;
    std::time_t time = std::mktime(&tm);
    const struct std::tm *ntm = std::gmtime(&time);

    std::ostringstream oss;
    oss << std::put_time(ntm, "%Y-%m-%dT00:00:00Z");
    return oss.str();
}


inline void stacPointcloud(MetadataNode& root, MetadataNode& statsMeta,
    MetadataNode& readerMeta, MetadataNode& props, std::string pcType)
{
    std::string filename = getChild(root, "filename").value();
    std::string fileExt = FileUtils::extension(filename);

    uint32_t position(0);
    point_count_t count = 0;
    bool bNoPoints(true);
    //Gather stac information for poinctloud extension
    auto pc_stats = statsMeta.findChildren([](MetadataNode& n)
        { return n.name()=="statistic"; });
    auto&& pc_count = getChild(readerMeta, "num_points");
    auto pc_schemas = getChild(readerMeta, "schema").findChildren(
        [](MetadataNode& n)
        { return n.name()=="dimensions"; });

    for (auto& stat: pc_stats) {
        props.addList(stat.clone("pc:statistics"));
    }
    for (auto& schema: pc_schemas)
    {
        props.addList(schema.clone("pc:schemas"));
    }
    props.add(pc_count.clone("pc:count"));
    props.add("pc:encoding", fileExt);
    props.add("pc:type", pcType);
}

inline void addBox(MetadataNode& n, MetadataNode& box, std::string name)
{
    n.addWithType(name, getChild(box, "minx").value(), "double", "");
    n.addWithType(name, getChild(box, "miny").value(), "double", "");
    n.addWithType(name, getChild(box, "minz").value(), "double", "");
    n.addWithType(name, getChild(box, "maxx").value(), "double", "");
    n.addWithType(name, getChild(box, "maxy").value(), "double", "");
    n.addWithType(name, getChild(box, "maxz").value(), "double", "");
}

inline void stacProjection(MetadataNode& root, MetadataNode& statsMeta,
    MetadataNode& readerMeta, MetadataNode& stac)
{
    MetadataNode props = getChild(stac, "properties");

    MetadataNode bbox = getChild(statsMeta, "bbox");

    MetadataNode epsg4326 = getChild(bbox, "EPSG:4326");
    MetadataNode stacGeom = getChild(epsg4326, "boundary");
    MetadataNode stacBbox = getChild(epsg4326, "bbox");
    stac.add(stacGeom.clone("geometry"));
    addBox(stac, stacBbox, "bbox");

    // don't need to throw with this one, can exist without proj
    // extension if need be.
    if (!bbox.findChild("native").empty())
    {
        stac.add("stac_extensions", "https://stac-extensions.github.io/projection/v1.1.0/schema.json");
        MetadataNode native = getChild(bbox, "native");
        MetadataNode projGeom = getChild(native, "boundary");
        MetadataNode projBbox = getChild(native, "bbox");
        MetadataNode srs = getChild(readerMeta, "srs");
        MetadataNode projJson = getChild(srs, "json");
        MetadataNode projWkt2 = getChild(srs, "wkt");
        addBox(props, projBbox, "proj:bbox");
        props.add(projGeom.clone("proj:geometry"));
        props.add(projJson.clone("proj:projjson"));
        props.add(projWkt2.clone("proj:wkt2"));
    }
}

inline void addStacMetadata(MetadataNode& root, MetadataNode& statsMeta,
    MetadataNode& readerMeta, MetadataNode& infoMeta, std::string pcType)
{
    try {
        MetadataNode stac;
        std::string filename = getChild(root, "filename").value();
        std::string stem = FileUtils::stem(filename);
        std::string absPath = FileUtils::toAbsolutePath(filename);

        //Base STAC object
        MetadataNode id = stac.add("id", stem);
        MetadataNode properties = stac.add("properties");

        //TODO make sure these are available
        //For now, if there isn't date similar to laz/las/copc then use now.
        try
        {
            std::string doy = getChild(readerMeta, "creation_doy").value();
            std::string year = getChild(readerMeta, "creation_year").value();
            properties.add("datetime", getDateStr(year, doy));
        } catch (std::exception &)
        {
            auto&& datetime = getChild(infoMeta, "now");
            properties.add("datetime", datetime);
        }

        stac.add("type", "Feature");
        stac.add("stac_version", "1.0.0");

        //links
        MetadataNode self = stac.addList("links");
        self.add("rel", "derived_from");
        self.add("href", absPath);

        //assets - add source file to data asset
        MetadataNode assets = stac.add("assets");
        MetadataNode data;
        data.add("href", absPath);
        data.add("title", "Pointcloud data");
        assets.add(data.clone("data"));

        stac.add("stac_extensions", "https://stac-extensions.github.io/pointcloud/v1.0.0/schema.json");
        stacPointcloud(root, statsMeta, infoMeta, properties, pcType);
        stacProjection(root, statsMeta, readerMeta, stac);
        root.add(stac.clone("stac"));
    }
    catch (const std::out_of_range& e)
    {
        MetadataNode msg;
        std::stringstream message;
        message << "Failed to create STAC Feature with missing key. '" << e.what() << "'";
        msg.addWithType("status", "error", "string", "Error status");
        msg.addWithType("message", message.str(), "string", "Error message");
        root.add(msg.clone("stac"));
    }
}


}