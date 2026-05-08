#include "TileIndex.hpp"
#include "Dataset.hpp"

#include <ogr_api.h>

namespace pdal
{
namespace tindex
{

TileIndexBuilder::TileIndexBuilder(const Args& args, const std::string& tileIndexColumnName,
        const std::string& srsColumnName, const std::string& driverName, const std::string& tgtSrs,
        const std::string& assignSrs)
    : TIndexBuilder(args, tileIndexColumnName, srsColumnName, driverName, tgtSrs, assignSrs)
{
    m_mtimeField = m_dataset->defineField("modified", OFTDateTime);
    m_ctimeField = m_dataset->defineField("created", OFTDateTime);
}

FileInfoPtr TileIndexBuilder::makeFileInfo(const std::string& filename)
{
    return std::make_unique<FileInfo>(filename);
}

void TileIndexBuilder::getFileInfo(FileInfoPtr& fileInfo)
{
    PipelineManager manager;
    manager.commonOptions() = m_commonOptions;
    manager.stageOptions() = m_stageOptions;

    Stage& reader = manager.makeReader(fileInfo->m_filename, "");
    runBoundary(reader, *fileInfo, manager);
}

void TileIndexBuilder::createExtraFields(const FileInfoPtr& fileInfo,
    Feature& feature)
{
    feature.setField(m_mtimeField, fileInfo->m_ctime);
    feature.setField(m_ctimeField, fileInfo->m_mtime);
}

} // namespace tindex
} // namespace pdal
