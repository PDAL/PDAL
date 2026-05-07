#include "StacIndex.hpp"

#include <ogr_api.h>
#include <cpl_string.h>

namespace pdal
{
namespace tindex   
{

StacIndex::StacIndex(const Args& args, const std::string& pcType)
    : TIndexBuilder(args, "assets.data.href", "proj:wkt2", "Parquet", "EPSG:4326", "EPSG:4326"),
      m_pcType(pcType)
{
    // Add STAC-specific fields
    m_fields.try_emplace("proj:projjson", OFTString, OFSTJSON);
    m_fields.emplace("datetime", OFTDateTime);
    m_fields.try_emplace("links", OFTString, OFSTJSON);
    m_fields.emplace("id", OFTString);
    m_fields.emplace("stac_extensions", OFTStringList);
    m_fields.emplace("stac_version", OFTString);
    m_fields.emplace("pc:count", OFTInteger);
    m_fields.emplace("pc:encoding", OFTString);
    m_fields.emplace("pc:type", OFTString);
    m_fields.try_emplace("pc:schemas", OFTString, OFSTJSON);
    m_fields.try_emplace("pc:statistics", OFTString, OFSTJSON);

    m_extensions = { "https://stac-extensions.github.io/projection/v1.1.0/",
        "https://stac-extensions.github.io/pointcloud/v1.0.0/" };
}

StacIndex::~StacIndex()
{}

std::unique_ptr<FileInfo> StacIndex::makeFileInfo(const std::string& filename)
{
    auto info = std::make_unique<StacFileInfo>();
    info->m_filename = filename;
    info->m_stacInfo.init(filename);
    return info;
}

void StacIndex::getFileInfo(std::unique_ptr<FileInfo>& fileInfo)
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
        stacFileInfo.m_stacInfo.addMetadata(statsMeta, readerMeta, infoMeta, m_pcType);
    }
}

void StacIndex::createExtraFields(const std::unique_ptr<FileInfo>& fileInfo, TIndexFeature& feature)
{
    // removing newlines to get rid of dead space in the parquet file. Not strictly necessary
    auto stripNewline = [](std::string& s) {
        s.erase(std::remove_if(s.begin(), s.end(), 
            [](char c) { return c == '\n' || c == '\r'; }), s.end());
        return s;
    };

    StacFileInfo& stacFileInfo = static_cast<StacFileInfo&>(*fileInfo);
    StacInfo& stacInfo = stacFileInfo.m_stacInfo;

    std::string linksJson = Utils::toJSON(stacInfo.rootChildren("links"));
    feature.setField(m_fields.at("links"), stripNewline(linksJson));

    feature.setField(m_fields.at("id"), FileUtils::getFilename(stacFileInfo.m_filename));

    feature.setField(m_fields.at("stac_version"), STAC_VERSION);

    feature.setField(m_fields.at("stac_extensions"), m_extensions);

    // Not added: proj:bbox, proj:geometry

    std::string projJson = SpatialReference(stacFileInfo.m_srs).getPROJJSON();
    feature.setField(m_fields.at("proj:projjson"), projJson);
 
    int pointCount = stacInfo.propertiesChild("pc:count").value<int>();
    feature.setField(m_fields.at("pc:count"), pointCount);

    std::string encoding = stacInfo.propertiesChild("pc:encoding").value();
    feature.setField(m_fields.at("pc:encoding"), encoding);

    feature.setField(m_fields.at("pc:type"), m_pcType);

    // Not sure if schema and statistics need to be native parquet lists or if json is ok
    std::string schema = Utils::toJSON(stacInfo.propertiesChildren("pc:schemas"));
    feature.setField(m_fields.at("pc:schemas"), stripNewline(schema));

    std::string statistics = Utils::toJSON(stacInfo.propertiesChildren("pc:statistics"));
    feature.setField(m_fields.at("pc:statistics"), stripNewline(statistics));
}

} // namespace pdal
} // namespace tindex