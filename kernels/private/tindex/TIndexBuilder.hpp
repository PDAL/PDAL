#pragma once

#include <ogr_api.h>

#include <pdal/Polygon.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>
#include <pdal/Options.hpp>

#include "../stac/StacInfo.hpp"
#include "TIndexBoundary.hpp"

#define STAC_VERSION "1.1.0"

// Get GDAL's forward decls if available
// otherwise make our own
#if __has_include(<gdal_fwd.h>)
#include <gdal_fwd.h>
#else
using OGRDataSourceH = void *;
using OGRLayerH = void *;
using OGRFeatureH = void *;
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

class StacInfo
{
public:
    StacInfo() {}

    void init(std::string const& filename)
    {
        // stacProjection & stacPointCloud need these set in the root metadata 
        // for it to work. Should refactor to be less metadata dependent
        m_root.add("filename", filename);
        MetadataNode self = m_root.addList("links");
        self.add("rel", "derived_from");
        self.add("href", filename);
        m_properties = m_root.add("properties");
        m_extensions = { "https://stac-extensions.github.io/projection/v1.1.0/",
            "https://stac-extensions.github.io/pointcloud/v1.0.0/" };
    }

    void addMetadata(MetadataNode& statsMeta, MetadataNode& readerMeta,
        MetadataNode& infoMeta, std::string pcType)
    {
        stacPointcloud(m_root, statsMeta, infoMeta, m_properties, pcType);
    }

    MetadataNode propertiesChild(std::string key) 
    {
        return getChild(m_properties, key); 
    }

    MetadataNodeList propertiesChildren(std::string key) 
    {
        return m_properties.children(key); 
    }

    MetadataNode rootChild(std::string key) 
    {
        return getChild(m_root, key);
    }

    MetadataNodeList rootChildren(std::string key) 
    {
        return m_root.children(key);
    }

    StringList extensions() const { return m_extensions; }

private:
    MetadataNode m_root;
    MetadataNode m_properties;
    StringList m_extensions;
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

struct StacFileInfo : FileInfo
{
    StacInfo m_stacInfo;
};

class TIndexBuilder
{
public:
    void getFieldIndexes(const OGRFeatureDefnH layerDefn);
    //std::vector<FileInfo> merge(const StringList& files);
    void create(const StringList& files, PipelineManager& mgr);
    const FieldMap& getFields() { return m_fields; }

protected:
    TIndexBuilder(const Args& args, const std::string& tileIndexColumnName,
        const std::string& srsColumnName, const std::string& driverName, 
        const std::string& tgtSrs, const std::string& assignSrs);
    ~TIndexBuilder();

    bool runBoundary(Stage& stage, FileInfo& fileInfo,
        PipelineManager& manager);

    std::vector<std::unique_ptr<FileInfo>> m_infos;
    FieldMap m_fields;
    Options m_commonOptions;
    OptionsMap m_stageOptions;

private:
    virtual void processFile(const std::string& filename) = 0;
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

    OGRDataSourceH m_dataset;
    OGRLayerH m_layer;

    //
    // Only used in create()
    //
    LogPtr m_log;
    //StringList m_files;
};

class TileIndex : public TIndexBuilder
{
public:
    TileIndex(const Args& args, const std::string& tileIndexColumnName,
        const std::string& srsColumnName, const std::string& driverName, const std::string& tgtSrs,
        const std::string& assignSrs);
    //void create(const StringList& files) override;
private:
    std::unique_ptr<FileInfo> makeFileInfo(const std::string& filename) override;
    void getFileInfo(std::unique_ptr<FileInfo>& fileInfo) override;
    void createExtraFields(const std::unique_ptr<FileInfo>& fileInfo, OGRFeatureH hFeature) override;
};

class StacIndex : public TIndexBuilder
{
public:
    StacIndex(const Args& args, const std::string& pcType);
    //void create(const StringList& files) override;
private:
    std::unique_ptr<FileInfo> makeFileInfo(const std::string& filename) override;
    void getFileInfo(std::unique_ptr<FileInfo>& fileInfo) override;
    void createExtraFields(const std::unique_ptr<FileInfo>& fileInfo, OGRFeatureH hFeature) override;

    StringList m_extensions;
    std::string m_pcType;
    //std::vector<StacInfo> m_stacInfos;
};

class TIndexError : public std::runtime_error
{
public:
    TIndexError(const std::string txt) : std::runtime_error(txt)
    {}
};

} // namespace tindex
} // namespace pdal