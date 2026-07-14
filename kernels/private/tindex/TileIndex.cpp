#include "TileIndex.hpp"
#include "Dataset.hpp"

namespace pdal
{
namespace tindex
{

TileIndexBuilder::TileIndexBuilder(const Args& args, const std::string& tileIndexColumnName,
        const std::string& srsColumnName, const std::string& driverName, const std::string& tgtSrs,
        const std::string& assignSrs)
    : TIndexProcessor(args, tileIndexColumnName, srsColumnName, driverName, tgtSrs, assignSrs)
{
    m_mtimeField = m_dataset->defineField("modified", OFTDateTime);
    m_ctimeField = m_dataset->defineField("created", OFTDateTime);
}

FileInfoPtr TileIndexBuilder::makeFileInfo(const std::string& filename)
{
    return std::make_unique<FileInfo>(filename);
}

void TileIndexBuilder::fillFileInfo(FileInfoPtr& fileInfo)
{
    PipelineManager manager;
    manager.commonOptions() = m_commonOptions;
    manager.stageOptions() = m_stageOptions;

    manager.makeReader(fileInfo->m_filename, "");
    runBoundary(fileInfo, manager);
}

bool TileIndexBuilder::fastBoundary(PipelineManager& manager, FileInfoPtr& fileInfo)
{
    Stage* reader = manager.stages().front();
    QuickInfo qi = reader->preview();
    if (!qi.valid())
        return false;

    fileInfo->m_boundary = qi.m_bounds.to2d().toWKT();
    if (!qi.m_srs.empty())
        fileInfo->m_srs = qi.m_srs.getWKT();
    fileInfo->m_gridHeight = 0.0;
    return true;
}

void TileIndexBuilder::createExtraFields(const FileInfoPtr& fileInfo,
    Feature& feature)
{
    feature.setField(m_mtimeField, fileInfo->m_ctime);
    feature.setField(m_ctimeField, fileInfo->m_mtime);
}

} // namespace tindex
} // namespace pdal
