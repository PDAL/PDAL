#include "StacIndex.hpp"
#include "Dataset.hpp"

namespace pdal
{
namespace tindex
{

StacIndexBuilder::StacIndexBuilder(const Args& args, const std::string& pcType,
    bool statistics, NL::json& staticFields)
    : TIndexProcessor(args, "assets.data.href", "proj:wkt2", "Parquet", "EPSG:4326", "EPSG:4326"),
      m_writeStats(statistics)
{
#if GDAL_VERSION_NUM <= GDAL_COMPUTE_VERSION(3,9,0)
    throw TIndexError("STAC GeoParquet support requires GDAL 3.9.0 or later");
#endif
    // Add STAC-specific fields
    m_srsField = m_dataset->defineField("proj:projjson", OFTString, OFSTJSON);
    m_datetimeField = m_dataset->defineField("datetime", OFTDateTime);
    // Empty field, so that it stays in the schema.
    m_linksField = m_dataset->defineField("links", OFTString, OFSTJSON);
    m_idField = m_dataset->defineField("id", OFTString);
    m_pcCountField = m_dataset->defineField("pc:count", OFTInteger64);
    m_pcEncodingField = m_dataset->defineField("pc:encoding", OFTString);
    m_pcSchemasField = m_dataset->defineField("pc:schemas", OFTString, OFSTJSON);
    // Optional fields that require filters.stats
    if (m_writeStats)
    {
        m_pcStatsField = m_dataset->defineField("pc:statistics", OFTString, OFSTJSON);
        m_projBboxField = m_dataset->defineField("proj:bbox", OFTRealList);
    }

    // Add static fields
    std::vector<std::string> extensions = { "https://stac-extensions.github.io/projection/v1.1.0/",
        "https://stac-extensions.github.io/pointcloud/v1.0.0/" };
    std::vector <std::string> assetRoles = { "data" };
    m_staticFields.push_back(m_dataset->defineField("stac_extensions", extensions));
    m_staticFields.push_back(m_dataset->defineField("assets.data.roles", assetRoles));
    m_staticFields.push_back(m_dataset->defineField("stac_version", STAC_VERSION));
    m_staticFields.push_back(m_dataset->defineField("pc:type", pcType));
    for (auto& [key, value] : staticFields.items())
        m_staticFields.push_back(m_dataset->defineField(key, value));

    // Sort fields alphabetically for better structure
    m_dataset->sortFields();
}

FileInfoPtr StacIndexBuilder::makeFileInfo(const std::string& filename)
{
    return std::make_unique<StacFileInfo>(filename);
}

void StacIndexBuilder::fillFileInfo(FileInfoPtr& fileInfo)
{
    PipelineManager manager;
    manager.commonOptions() = m_commonOptions;
    manager.stageOptions() = m_stageOptions;

    Stage& reader = manager.makeReader(fileInfo->m_filename, "");
    Stage* stats(nullptr);
    if (m_writeStats)
        stats = &(manager.makeFilter("filters.stats", reader));

    // We have 4 cases here: fast or slow boundary, with and w/o stats.
    // Depending on the case (and what reader is used), the stacFileInfo fields
    // will be set differently.
    if (runBoundary(fileInfo, manager))
    {
        StacFileInfo& stacFileInfo = static_cast<StacFileInfo&>(*fileInfo);

        MetadataNode readerMeta = reader.getMetadata();
        // Only add stats metadata if a stats filter was added
        MetadataNode statsMeta;
        if (stats)
            statsMeta = stats->getMetadata();

        MetadataNode schema = 
            manager.pointTable().layout()->toMetadata().clone("schema");
        // Schema may not exist if fastBoundary was run; already added to 
        // the StacFileInfo in that case.
        if (schema)
            stacFileInfo.addSchema(schema);
        stacFileInfo.addMetadata(readerMeta, statsMeta);
    }
}

bool StacIndexBuilder::fastBoundary(PipelineManager& manager, FileInfoPtr& fileInfo)
{
    StacFileInfo& stacFileInfo = static_cast<StacFileInfo&>(*fileInfo);

    Stage* reader = manager.stages().front();
    QuickInfo qi = reader->preview();
    if (!qi.valid())
        return false;

    stacFileInfo.m_boundary = qi.m_bounds.to2d().toWKT();
    if (!qi.m_srs.empty())
        stacFileInfo.m_srs = qi.m_srs.getWKT();
    stacFileInfo.m_gridHeight = 0.0;

    // If the optional stats filter was added in fillFileInfo, we still need 
    // to execute the whole thing in order to get stats metadata.
    if (manager.stages().size() > 1)
    {
        // If fastBoundary is run after a slowBoundary failure, we can't rerun
        // the manager. This is a really stupid fix.
        PointTable table;
        Stage* stage = manager.getStage();
        stage->prepare(table);
        stage->execute(table);
        return true;
    }

    // If there isn't a stats filter, get the point count from quickInfo. Otherwise
    // it will be set from the stats metadata.
    stacFileInfo.m_count = qi.m_pointCount;

    // If the manager isn't executed, dimensions won't be in the layout, so we 
    // have to make a schema from the dimension names.
    // Structure copied from PointLayout::toMetadata()
    MetadataNode root("schema");
    for (std::string& dimName : qi.m_dimNames)
    {
        MetadataNode dim("dimensions");
        dim.add("name", dimName);
        Dimension::Id id = Dimension::id(dimName);
        Dimension::Type t = Dimension::defaultType(id);
        dim.add("type", Dimension::toName(Dimension::base(t)));
        dim.add("size", Dimension::size(t));
        root.addList(dim);
    }
    stacFileInfo.addSchema(root);

    return true;
}

void StacIndexBuilder::createExtraFields(const FileInfoPtr& fileInfo,
    Feature& feature)
{
    StacFileInfo& stacFileInfo = static_cast<StacFileInfo&>(*fileInfo);

    for (auto& field : m_staticFields)
        feature.setField(field);

    feature.setField(m_idField, FileUtils::getFilename(stacFileInfo.m_filename));
    feature.setField(m_srsField, SpatialReference(stacFileInfo.m_srs).getPROJJSON());
    feature.setField(m_pcCountField, stacFileInfo.m_count);
    feature.setField(m_pcEncodingField, stacFileInfo.m_encoding);
    feature.setField(m_datetimeField, stacFileInfo.datetime());
    feature.setField(m_pcSchemasField, stacFileInfo.schemas());
    if (m_writeStats)
    {
        feature.setField(m_pcStatsField, stacFileInfo.statistics());
        feature.setField(m_projBboxField, stacFileInfo.bbox());
    }
}

} // namespace pdal
} // namespace tindex
