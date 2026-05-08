#include "StacIndex.hpp"
#include "Dataset.hpp"

#include <ogr_api.h>
#include <cpl_string.h>

namespace pdal
{
namespace tindex
{

StacIndexBuilder::StacIndexBuilder(const Args& args, const std::string& pcType)
    : TIndexBuilder(args, "assets.data.href", "proj:wkt2", "Parquet", "EPSG:4326", "EPSG:4326"),
      m_pcType(pcType)
{
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
    m_pcStatsField = m_dataset->defineField("pc:statistics", OFTString, OFSTJSON);

    m_extensions = { "https://stac-extensions.github.io/projection/v1.1.0/",
        "https://stac-extensions.github.io/pointcloud/v1.0.0/" };
}

FileInfoPtr StacIndexBuilder::makeFileInfo(const std::string& filename)
{
    return std::make_unique<StacFileInfo>(filename);
}

void StacIndexBuilder::getFileInfo(FileInfoPtr& fileInfo)
{
    StacFileInfo& stacFileInfo = static_cast<StacFileInfo&>(*fileInfo);

    PipelineManager manager;
    manager.commonOptions() = m_commonOptions;
    manager.stageOptions() = m_stageOptions;

    Stage& reader = manager.makeReader(stacFileInfo.m_filename, "");
    Stage& info = manager.makeFilter("filters.info", reader);
    Stage& stats = manager.makeFilter("filters.stats", info);

    if (runBoundary(stats, stacFileInfo, manager))
    {
        MetadataNode readerMeta = reader.getMetadata();
        MetadataNode statsMeta = stats.getMetadata();
        MetadataNode infoMeta = info.getMetadata();
        stacFileInfo.addMetadata(statsMeta, readerMeta, infoMeta, m_pcType);
    }
}

void StacIndexBuilder::createExtraFields(const FileInfoPtr& fileInfo,
    Feature& feature)
{
    // removing newlines to get rid of dead space in the parquet file. Not strictly necessary
    auto stripNewline = [](const std::string& s) -> std::string
    {
        std::string out;

        for (char c : s)
            if (c != '\n' && c != '\r')
                out.push_back(c);
        return out;
    };

    StacFileInfo& stacFileInfo = static_cast<StacFileInfo&>(*fileInfo);

    std::string linksJson = stripNewline(Utils::toJSON(stacFileInfo.rootChildren("links")));
    std::string schema = stripNewline(Utils::toJSON(
        stacFileInfo.propertiesChildren("pc:schemas")));
    std::string stats = stripNewline(Utils::toJSON(
        stacFileInfo.propertiesChildren("pc:statistics")));
    std::string projJson = stripNewline(SpatialReference(stacFileInfo.m_srs).getPROJJSON());

    feature.setField(m_linksField, linksJson);
    feature.setField(m_idField, FileUtils::getFilename(stacFileInfo.m_filename));
    feature.setField(m_stacVersionField, STAC_VERSION);
    feature.setField(m_stacExtensionsField, m_extensions);
    feature.setField(m_srsField, projJson);
    feature.setField(m_pcCountField, stacFileInfo.propertiesChild("pc:count").value<int>());
    feature.setField(m_pcEncodingField, stacFileInfo.propertiesChild("pc:encoding").value());
    feature.setField(m_pcTypeField, m_pcType);
    // Not sure if schema and statistics need to be native parquet lists or if json is ok
    feature.setField(m_pcSchemasField, schema);
    feature.setField(m_pcStatsField, stats);
}

} // namespace pdal
} // namespace tindex
