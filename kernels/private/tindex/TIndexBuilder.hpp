#pragma once

#include <ogr_api.h>

#include <pdal/Polygon.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>
#include <pdal/Options.hpp>

#include "TIndexBoundary.hpp"

// Get GDAL's forward decls if available
// otherwise make our own
#if __has_include(<gdal_fwd.h>)
#include <gdal_fwd.h>
#else
using OGRDataSourceH = void *;
using OGRLayerH = void *;
using OGRFeatureH = void *;
using OGRFieldType = int;
using OGRFieldSubType = int;
#endif

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

struct FieldInfo;
using FieldMap = std::map<std::string, FieldInfo>;

struct FieldInfo
{
    FieldInfo(const OGRFieldType fieldType) : m_fieldType(fieldType) 
    {}
    FieldInfo(const OGRFieldType fieldType, const OGRFieldSubType subtype) 
        : m_fieldType(fieldType), m_subtype(subtype) 
    {}
    ~FieldInfo() {}
    void setIdx(int idx) { m_fieldIdx = idx; }
    operator int() const { return m_fieldIdx; }

    OGRFieldType m_fieldType;
    OGRFieldSubType m_subtype = OFSTNone;
    int m_fieldIdx = -1;
};

// messy - only includes some args because we need to have defaults for stac
// (column names, SRS) & don't want to override them if they come from here
struct Args
{
    std::string idxFilename;
    std::string layerName;
    //std::string tileIndexColumnName;
    //std::string srsColumnName;
    //std::string driverName;
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

    // stac-specific
    //std::string pcType;
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
};

class TIndexBuilder
{
public:
    virtual ~TIndexBuilder();

    void getFieldIndexes(const OGRFeatureDefnH layerDefn);
    //std::vector<FileInfo> merge(const StringList& files);
    void create(const StringList& files, PipelineManager& mgr);
    const FieldMap& getFields() { return m_fields; }

protected:
    TIndexBuilder(const Args& args, const std::string& tileIndexColumnName,
        const std::string& srsColumnName, const std::string& driverName, 
        const std::string& tgtSrs, const std::string& assignSrs);

    bool runBoundary(Stage& stage, FileInfo& fileInfo,
        PipelineManager& manager);

    std::vector<std::unique_ptr<FileInfo>> m_infos;
    FieldMap m_fields;
    Options m_commonOptions;
    OptionsMap m_stageOptions;
    OGRDataSourceH m_dataset;

private:
    virtual std::unique_ptr<FileInfo> makeFileInfo(const std::string& filename) = 0;
    virtual void getFileInfo(std::unique_ptr<FileInfo>& fileInfo) = 0;
    virtual void createExtraFields(const std::unique_ptr<FileInfo>& fileInfo, 
        OGRFeatureH feature) = 0;

    bool fastBoundary(Stage& reader, FileInfo& fileInfo);
    bool createFeature(const std::unique_ptr<FileInfo>& fileInfo);
    bool isFileIndexed(const std::unique_ptr<FileInfo>& fileInfo);
    void setStringField(OGRFeatureH hFeature, int idx, const char* value);
    pdal::Polygon prepareGeometry(const std::unique_ptr<FileInfo>& fileInfo);
    std::string makeMultiPolygon(const std::string& wkt);
    bool openDataset();
    bool createDataset();
    bool openLayer();
    bool createLayer();
    void createFields();

    const Args& m_args;
    std::string m_tileIndexColumnName;
    std::string m_srsColumnName;
    std::string m_driverName;
    std::string m_tgtSrsString;
    std::string m_assignSrsString;
    std::string m_layerName;
    std::string m_originalSrs;
    size_t m_maxFieldSize;

    OGRLayerH m_layer;

    //
    // Only used in create()
    //
    LogPtr m_log;
    //StringList m_files;
};

class TIndexError : public std::runtime_error
{
public:
    TIndexError(const std::string txt) : std::runtime_error(txt)
    {}
};

} // namespace tindex
} // namespace pdal