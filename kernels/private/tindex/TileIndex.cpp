
#include "TileIndex.hpp"

#include <ogr_api.h>

// just copied from TIndexKernel
namespace
{

void setDate(OGRFeatureH feature, const tm& tyme, int fieldNumber)
{
    OGR_F_SetFieldDateTime(feature, fieldNumber,
        tyme.tm_year + 1900, tyme.tm_mon + 1, tyme.tm_mday, tyme.tm_hour,
        tyme.tm_min, tyme.tm_sec, 100);
}

} // anonymous namespace

namespace pdal
{
namespace tindex
{

TileIndex::TileIndex(const Args& args, const std::string& tileIndexColumnName,
        const std::string& srsColumnName, const std::string& driverName, const std::string& tgtSrs,
        const std::string& assignSrs)
    : TIndexBuilder(args, tileIndexColumnName, srsColumnName, driverName, tgtSrs, assignSrs) 
{
    m_fields.emplace("modified", OFTDateTime);
    m_fields.emplace("created", OFTDateTime);
}

TileIndex::~TileIndex()
{
    if (m_dataset)
        OGR_DS_Destroy(m_dataset);
}

std::unique_ptr<FileInfo> TileIndex::makeFileInfo(const std::string& filename)
{
    auto info = std::make_unique<FileInfo>();
    info->m_filename = filename;
    return info;
}

void TileIndex::getFileInfo(std::unique_ptr<FileInfo>& fileInfo)
{
    PipelineManager manager;
    manager.commonOptions() = m_commonOptions;
    manager.stageOptions() = m_stageOptions;

    Stage& reader = manager.makeReader(fileInfo->m_filename, "");
    runBoundary(reader, *fileInfo, manager);  
}

void TileIndex::createExtraFields(const std::unique_ptr<FileInfo>& fileInfo, OGRFeatureH hFeature)
{
    setDate(hFeature, fileInfo->m_ctime, m_fields.at("created"));
    setDate(hFeature, fileInfo->m_mtime, m_fields.at("modified"));
}

} // namespace tindex
} // namespace pdal