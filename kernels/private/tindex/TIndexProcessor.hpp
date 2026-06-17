#pragma once

#include <pdal/util/FileUtils.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>
#include <pdal/Options.hpp>

#include "TIndexBoundary.hpp"

namespace pdal
{
    class Polygon;

namespace gdal
{
    class SpatialRef;
}

class StageFactory;

namespace tindex
{

struct Field;
class Dataset;
class Feature;

// Only includes some args because we need to have defaults for stac
// (column names, SRS) & don't want to override them if they come from here
struct Args
{
    std::string idxFilename;
    std::string layerName;
    std::string wkt;
    StringList lcOptions;
    std::string pathPrefix;
    int numThreads;
    bool absPath;
    bool overrideASrs;
    bool skipMultiSrs;

    // boundary options
    bool fastBoundary;
    bool doSmooth;
    int32_t density;
    double edgeLength;
    uint32_t sampleSize;
    std::string boundaryExpr;
};

struct FileInfo
{
    std::string m_filename;
    std::string m_srs;
    std::string m_boundary;
    struct tm m_ctime;
    struct tm m_mtime;
    double m_gridHeight;
    bool m_isRemote = false;

    FileInfo(const std::string& filename) : m_filename(filename)
    {
        m_isRemote = Utils::isRemote(filename);
        if (!m_isRemote)
            FileUtils::fileTimes(m_filename, &m_ctime, &m_mtime);
    }
};
using FileInfoPtr = std::unique_ptr<FileInfo>;

class TIndexProcessor
{
public:
    TIndexProcessor(const Args& args, const std::string& tileIndexColumnName,
        const std::string& srsColumnName, const std::string& driverName,
        const std::string& tgtSrs, const std::string& assignSrs);
    virtual ~TIndexProcessor();

    void create(const StringList& files, PipelineManager& mgr);
    std::vector<FileInfo> readIndex();

protected:
    bool runBoundary(FileInfoPtr& fileInfo, PipelineManager& manager);

    std::vector<FileInfoPtr> m_infos;
    std::unique_ptr<Dataset> m_dataset;
    Options m_commonOptions;
    OptionsMap m_stageOptions;

private:
    virtual FileInfoPtr makeFileInfo(const std::string& filename) 
        { return nullptr; }
    virtual void fillFileInfo(FileInfoPtr& fileInfo) {}
    virtual void createExtraFields(const FileInfoPtr& fileInfo,
        Feature& feature) {}
    virtual bool fastBoundary(PipelineManager& manager, FileInfoPtr& fileInfo) 
        { return false; }

    bool createFeature(const FileInfoPtr& fileInfo);
    bool isFileIndexed(const FileInfoPtr& fileInfo);
    Polygon prepareGeometry(const FileInfo& fileInfo);

    const Args& m_args;
    std::string m_tileIndexColumnName;
    std::string m_srsColumnName;
    std::string m_driverName;
    std::string m_tgtSrsString;
    std::string m_assignSrsString;
    std::string m_layerName;
    std::string m_originalSrs;

    Field *m_tindexColumnNameField;
    Field *m_srsColumnNameField;

    LogPtr m_log;
};

} // namespace tindex
} // namespace pdal
