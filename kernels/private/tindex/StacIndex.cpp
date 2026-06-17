#include "StacIndex.hpp"
#include "Dataset.hpp"

namespace pdal
{
namespace tindex
{

StacIndexBuilder::StacIndexBuilder(const Args& args, const std::string& pcType,
    bool statistics)
    : TIndexProcessor(args, "assets.data.href", "proj:wkt2", "Parquet", "EPSG:4326", "EPSG:4326"),
      m_pcType(pcType), m_writeStats(statistics)
{
#if GDAL_VERSION_NUM <= GDAL_COMPUTE_VERSION(3,9,0)
    throw TIndexError("STAC GeoParquet support requires GDAL 3.9.0 or later");
#endif

    // Add STAC-specific fields
    m_srsField = m_dataset->defineField("proj:projjson", OFTString, OFSTJSON);
    m_datetimeField = m_dataset->defineField("datetime", OFTDateTime);
    m_linksField = m_dataset->defineField("links", OFTString, OFSTJSON);
    m_idField = m_dataset->defineField("id", OFTString);
    m_stacExtensionsField = m_dataset->defineField("stac_extensions", OFTStringList);
    m_stacVersionField = m_dataset->defineField("stac_version", OFTString);
    m_pcCountField = m_dataset->defineField("pc:count", OFTInteger);
    m_pcEncodingField = m_dataset->defineField("pc:encoding", OFTString);
    m_pcTypeField = m_dataset->defineField("pc:type", OFTString);
    m_pcSchemasField = m_dataset->defineField("pc:schemas", OFTString, OFSTJSON);
    // Maybe write an empty field instead, so that it stays in the schema
    if (m_writeStats)
        m_pcStatsField = m_dataset->defineField("pc:statistics", OFTString, OFSTJSON);

    m_extensions = { "https://stac-extensions.github.io/projection/v1.1.0/",
        "https://stac-extensions.github.io/pointcloud/v1.0.0/" };
}

FileInfoPtr StacIndexBuilder::makeFileInfo(const std::string& filename)
{
    return std::make_unique<StacFileInfo>(filename);
}

void StacIndexBuilder::fillFileInfo(FileInfoPtr& fileInfo)
{
    StacFileInfo& stacFileInfo = static_cast<StacFileInfo&>(*fileInfo);

    PipelineManager manager;
    manager.commonOptions() = m_commonOptions;
    manager.stageOptions() = m_stageOptions;

    Stage& reader = manager.makeReader(stacFileInfo.m_filename, "");
    // Could add an info filter here if the reader isn't LAS/COPC
    //Stage& info = manager.makeFilter("filters.info", reader);
    Stage* stats(nullptr);
    if (m_writeStats)
        stats = &(manager.makeFilter("filters.stats", reader));

    if (runBoundary(stacFileInfo, manager))
    {
        MetadataNode readerMeta = reader.getMetadata();
        MetadataNode statsMeta;
        if (stats)
            statsMeta = stats->getMetadata();
        MetadataNode schema = 
            manager.pointTable().layout()->toMetadata().clone("schema");
        stacFileInfo.addMetadata(readerMeta, statsMeta, schema);
    }
}

bool StacIndexBuilder::fastBoundary(PipelineManager& manager, FileInfo& fileInfo)
{
    Stage* reader = manager.stages().front();
    // Would be ideal to avoid the preview call. Maybe can get info from the metadata?
    QuickInfo qi = reader->preview();
    if (!qi.valid())
        return false;

    fileInfo.m_boundary = qi.m_bounds.to2d().toWKT();
    if (!qi.m_srs.empty())
        fileInfo.m_srs = qi.m_srs.getWKT();
    fileInfo.m_gridHeight = 0.0;

    // If there's a stats filter we still need to execute the whole thing.
    // Maybe should make a simpler check.
    if (manager.stages().size() > 1)
        return !(manager.execute(ExecMode::PreferStream).m_mode == ExecMode::None);
    return true;
}

void StacIndexBuilder::createExtraFields(const FileInfoPtr& fileInfo,
    Feature& feature)
{
    StacFileInfo& stacFileInfo = static_cast<StacFileInfo&>(*fileInfo);

    feature.setField(m_linksField, stacFileInfo.links());
    feature.setField(m_idField, FileUtils::getFilename(stacFileInfo.m_filename));
    feature.setField(m_stacVersionField, STAC_VERSION);
    feature.setField(m_stacExtensionsField, m_extensions);
    feature.setField(m_srsField, SpatialReference(stacFileInfo.m_srs).getPROJJSON());
    feature.setField(m_pcCountField, stacFileInfo.count());
    feature.setField(m_pcEncodingField, stacFileInfo.encoding());
    feature.setField(m_pcTypeField, m_pcType);
    feature.setField(m_datetimeField, stacFileInfo.datetime());
    // Not sure if schema and statistics need to be native parquet lists or if json is ok
    feature.setField(m_pcSchemasField, stacFileInfo.schemas());
    if (m_writeStats)
        feature.setField(m_pcStatsField, stacFileInfo.statistics());
}

} // namespace pdal
} // namespace tindex
