/******************************************************************************
* Copyright (c) 2013, Howard Butler (hobu.inc@gmail.com)
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

#include "InfoKernel.hpp"

#include <ctime>
#include <algorithm>

#include <pdal/pdal_config.hpp>
#include <pdal/pdal_features.hpp>

#include <filters/InfoFilter.hpp>
#include <pdal/KDIndex.hpp>
#include <pdal/PipelineWriter.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/StageFactory.hpp>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "kernels.info",
    "Info Kernel",
    "http://pdal.io/apps/info.html"
};

CREATE_STATIC_KERNEL(InfoKernel, s_info)

std::string InfoKernel::getName() const { return s_info.name; }

InfoKernel::InfoKernel() : m_showStats(false), m_showSchema(false),
    m_showAll(false), m_showMetadata(false), m_boundary(false),
    m_showSummary(false), m_needPoints(false), m_statsStage(nullptr),
    m_hexbinStage(nullptr), m_infoStage(nullptr), m_reader(nullptr)
{}


void InfoKernel::validateSwitches(ProgramArgs& args)
{
    int functions = 0;

    if (!m_usestdin && m_inputFile.empty())
        throw pdal_error("No input file specified.");


    // All isn't really all.
    if (m_showAll)
    {
        m_showStats = true;
        m_showMetadata = true;
        m_showSchema = true;
        m_boundary = true;
    }


    if (m_stac)
    {
        if (m_queryPoint.size())
            throw pdal_error("'query' option incompatible with 'stac' option.");
        if (m_pointIndexes.size())
            throw pdal_error("'point' option incompatible with 'stac' option.");

        functions++;
        m_needPoints = true;
    }
    if (m_boundary)
    {
        functions++;
        m_needPoints = true;
    }
    if (m_queryPoint.size())
    {
        functions++;
        m_needPoints = true;
    }
    if (m_pointIndexes.size())
    {
        functions++;
        m_needPoints = true;
    }
    if (m_showSchema)
        functions++;
    if (m_showMetadata)
        functions++;
    if (m_showSummary)
        functions++;
    if (m_pipelineFile.size())
        functions++;
    if (m_showStats || functions == 0 )
    {
        functions++;
        m_showStats = true;
        m_needPoints = true;
    }

    if (m_pointIndexes.size() && m_queryPoint.size())
        throw pdal_error("'point' option incompatible with 'query' option.");

    if (m_showSummary && functions > 1)
        throw pdal_error("'summary' option incompatible with other "
            "specified options.");

    if (!m_showStats && m_enumerate.size())
        throw pdal_error("'enumerate' option requires 'stats' option.");
    if (!m_showStats && m_dimensions.size())
        throw pdal_error("'dimensions' option requires 'stats' option.");
}


void InfoKernel::addSwitches(ProgramArgs& args)
{
    args.add("input,i", "Input file name", m_inputFile).setOptionalPositional();
    args.add("all", "Dump statistics, schema and metadata", m_showAll);
    args.add("point,p", "Point to dump\n--point=\"1-5,10,100-200\" (0 indexed)",
        m_pointIndexes);
    args.add("query",
         "Return points in order of distance from the specified "
         "location (2D or 3D)\n"
         "--query Xcoord,Ycoord[,Zcoord][/count]",
         m_queryPoint);
    args.add("stats", "Dump stats on all points (reads entire dataset)",
        m_showStats);
    args.add("boundary", "Compute a hexagonal hull/boundary of dataset",
        m_boundary);
    args.add("dimensions", "Dimensions on which to compute statistics",
        m_dimensions);
    args.add("enumerate", "Dimensions whose values should be enumerated",
        m_enumerate);
    args.add("schema", "Dump the schema", m_showSchema);
    args.add("pipeline-serialization", "Output filename for pipeline "
        "serialization", m_pipelineFile);
    args.add("summary", "Dump summary of the info", m_showSummary);
    args.add("stac", "Dump STAC Item representation of the info.", m_stac);
    args.add("pc_type", "Pointcloud type for STAC generation (lidar, "
        "eopc, radar, sonar, other).", m_pcType, "lidar");
    args.add("metadata", "Dump file metadata info", m_showMetadata);
    args.add("stdin,s", "Read a pipeline file from standard input", m_usestdin);
}


// Note that the same information can come from the info filter, but
// this avoids point reads.
MetadataNode InfoKernel::dumpSummary(const QuickInfo& qi)
{
    MetadataNode summary;
    summary.add("num_points", qi.m_pointCount);
    if (qi.m_srs.valid())
    {
        MetadataNode srs = qi.m_srs.toMetadata();
        summary.add(srs);
    }
    if (qi.m_bounds.valid())
    {
        MetadataNode bounds = Utils::toMetadata(qi.m_bounds);
        summary.add(bounds.clone("bounds"));
    }

    std::string dims;
    auto di = qi.m_dimNames.begin();
    while (di != qi.m_dimNames.end())
    {
        dims += *di;
        ++di;
        if (di != qi.m_dimNames.end())
           dims += ", ";
    }
    if (dims.size())
        summary.add("dimensions", dims);

    if (!qi.m_metadata.empty() && qi.m_metadata.valid())
    {
        summary.add(qi.m_metadata.clone("metadata"));
    }

    return summary;
}

void InfoKernel::makeReader(const std::string& filename)
{
    Options rOps;
    if (!m_needPoints)
        rOps.add("count", 0);
    m_reader = &(m_manager.makeReader(filename, m_driverOverride, rOps));
}


void InfoKernel::makePipeline()
{
    Stage *stage = m_reader;

    Options iOps;
    if (m_queryPoint.size())
        iOps.add("query", m_queryPoint);
    if (m_pointIndexes.size())
        iOps.add("point", m_pointIndexes);
    stage = m_infoStage =
        &(m_manager.makeFilter("filters.info", *stage, iOps));

    if (m_showStats)
    {
        Options filterOptions;
        if (m_dimensions.size())
            filterOptions.add({"dimensions", m_dimensions});
        if (m_enumerate.size())
            filterOptions.add({"enumerate", m_enumerate});
        stage = m_statsStage =
            &m_manager.makeFilter("filters.stats", *stage, filterOptions);
    }

    if (m_stac)
    {
        // filters required for stac: metadata, stats,
        if (stage != m_statsStage)
        {
            Options stacOps;
            if (m_enumerate.size())
                stacOps.add({"enumerate", m_enumerate});
            m_stacStage = stage = &m_manager.makeFilter("filters.stats", *stage, stacOps);
        }
        else
            m_stacStage = stage;
    }
    if (m_boundary)
        m_hexbinStage = &m_manager.makeFilter("filters.hexbin", *stage);
}


MetadataNode InfoKernel::run(const std::string& filename)
{
    MetadataNode root;

    makeReader(filename);
    root.add("filename", filename);
    root.add("pdal_version", Config::fullVersionString());

    if (m_showSummary)
    {
        QuickInfo qi = m_manager.getStage()->preview();
        if (!qi.valid())
            throw pdal_error("No summary data available for '" +
                filename + "'.");
        root.add(dumpSummary(qi).clone("summary"));
    }
    else
    {
        makePipeline();
        if (m_needPoints || m_showMetadata)
            m_manager.execute(ExecMode::PreferStream);
        else
            m_manager.prepare();
        dump(root);
    }

    std::time_t now
    = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream t;
    t << std::put_time( std::localtime( &now ), "%FT%T%z" );
    root.add("reader", m_reader->getName());
    root.add("now", t.str());

    uintmax_t size = Utils::fileSize(filename);
    if (size)
        root.add("file_size", size);

    return root;
}


void InfoKernel::dump(MetadataNode& root)
{
    if (m_pipelineFile.size() > 0)
        PipelineWriter::writePipeline(m_manager.getStage(), m_pipelineFile);

    // Reader stage.
    if (m_showMetadata)
        root.add(m_reader->getMetadata().clone("metadata"));

    // Info stage.
    auto info = dynamic_cast<InfoFilter *>(m_infoStage);
    MetadataNode mdata = info->getMetadata();
    MetadataNode points = mdata.findChild("points");
    if (points)
        root.add(points);

    if (m_showSchema)
        root.add(mdata.findChild("schema"));

    // Stats stage.
    if (m_showStats)
        root.add(m_statsStage->getMetadata().clone("stats"));

    // Hexbin stage.
    if (m_hexbinStage)
    {
        MetadataNode node = m_hexbinStage->getMetadata();
        if (node.findChild("error"))
        {
            std::string poly = info->bounds().to2d().toWKT();
            std::string poly_geojson = info->bounds().to2d().toGeoJSON();

            MetadataNode m("boundary");
            m.add("boundary", poly, "Simple boundary of polygon");
            m.add("boundary_json", poly_geojson, "Simple boundary of polygon");
            root.add(m);
        }
        else
            root.add(m_hexbinStage->getMetadata().clone("boundary"));
    }

    // STAC stage.
    if (m_stac)
    {
        MetadataNode stats;
        if (!m_showStats)
            stats = m_stacStage->getMetadata();
        else
            stats = root.findChild("stats");

        MetadataNode meta;
        if (!m_showMetadata)
            meta = m_reader->getMetadata().clone("metadata");
        else
            meta = root.findChild("metadata");

        addStac(root, stats, meta, mdata);
    }
}

int InfoKernel::execute()
{
    std::string filename = (m_usestdin ? std::string("STDIN") : m_inputFile);
    MetadataNode root = run(filename);
    Utils::toJSON(root, std::cout);

    return 0;
}

std::string getDateStr(std::string year, std::string doy)
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

void InfoKernel::addStac(MetadataNode& root, MetadataNode& stats,
    MetadataNode& meta, MetadataNode& mdata)
{
    MetadataNode stac;
    std::string filename = root.findChild("filename").value();
    std::string stem = FileUtils::stem(filename);
    std::string absPath = FileUtils::toAbsolutePath(filename);

    //Base STAC object
    MetadataNode id = stac.add("id", stem);
    MetadataNode properties = stac.add("properties");

    //TODO make sure these are available
    //For now, if there isn't date similar to laz/las/copc then use now.
    try
    {
        std::string doy = meta.findChild("creation_doy").value();
        std::string year = meta.findChild("creation_year").value();
        properties.add("datetime", getDateStr(year, doy));
    } catch (std::exception &e)
    {
        auto&& datetime = mdata.findChild("now");
        properties.add("datetime", datetime);
    }


    // TODO add from metadata?
    stac.add("type", "Feature");
    stac.add("stac_version", "1.0.0");

    //stac_extensions - pointcloud and projection extensions
    stac.add("stac_extensions", "https://stac-extensions.github.io/pointcloud/v1.0.0/schema.json");
    stac.add("stac_extensions", "https://stac-extensions.github.io/projection/v1.1.0/schema.json");

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
    auto&& enc = meta.findChild("global_encoding");
    stacPointcloud(root, stats, mdata, properties);
    stacProjection(root, stats, meta, stac);

    root.add(stac.clone("stac"));
}

void InfoKernel::stacPointcloud(MetadataNode& root, MetadataNode& stats,
    MetadataNode& meta, MetadataNode& props)
{
    std::string filename = root.findChild("filename").value();
    std::string fileExt = FileUtils::extension(filename);

    uint32_t position(0);
    point_count_t count = 0;
    bool bNoPoints(true);
    //Gather stac information for poinctloud extension
    auto pc_stats = stats.findChildren([](MetadataNode& n)
        { return n.name()=="statistic"; });
    auto&& pc_count = meta.findChild("num_points");
    auto pc_schemas = meta.findChild("schema").findChildren(
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
    props.add("pc:type", m_pcType);
}

void addBox(MetadataNode& n, MetadataNode& box, std::string name)
{
    n.addWithType(name, box.findChild("minx").value(), "double", "");
    n.addWithType(name, box.findChild("miny").value(), "double", "");
    n.addWithType(name, box.findChild("minz").value(), "double", "");
    n.addWithType(name, box.findChild("maxx").value(), "double", "");
    n.addWithType(name, box.findChild("maxy").value(), "double", "");
    n.addWithType(name, box.findChild("maxz").value(), "double", "");
}

void InfoKernel::stacProjection(MetadataNode& root, MetadataNode& stats,
    MetadataNode& meta, MetadataNode& stac)
{
    MetadataNode&& props = stac.findChild("properties");

    MetadataNodeList bbox = stats.findChild("bbox").children();

    MetadataNode& epsg4326 = bbox[0];
    MetadataNode&& stacGeom = epsg4326.findChild("boundary");
    MetadataNode&& stacBbox = epsg4326.findChild("bbox");

    MetadataNode& native = bbox[1];
    MetadataNode&& projGeom = native.findChild("boundary");
    MetadataNode&& projBbox = native.findChild("bbox");
    MetadataNode&& srs = meta.findChild("srs");
    MetadataNode&& projJson = srs.findChild("json");
    MetadataNode&& projWkt2 = srs.findChild("wkt");

    addBox(props, projBbox, "proj:bbox");
    props.add(projGeom.clone("proj:geometry"));
    props.add(projJson.clone("proj:projjson"));
    props.add(projWkt2.clone("proj:wkt2"));

    stac.add(stacGeom.clone("geometry"));
    addBox(stac, stacBbox, "bbox");
}



} // namespace pdal
