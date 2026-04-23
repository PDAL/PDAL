#pragma once

#include <ogr_api.h>

#include <pdal/Polygon.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>

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

class FieldInfo;
using FieldMap = std::map<std::string, FieldInfo>;

struct FileInfo
{
    std::string m_filename;
    std::string m_srs;
    std::string m_boundary;
    double m_gridHeight;
};

class TIndexBuilder
{
public:
    void createOrOpenFile(const StringList& lcOptions);
    void execute(const StringList& files, int numThreads);

protected:
    TIndexBuilder(const std::string& idxFilename, const std::string& layerName,
        const std::string& driverName, const std::string& tileIndexColumnName,
        const std::string& srsColumnName);

private:
    bool openDataset(const std::string& filename);
    virtual bool createDataset(const std::string& filename);
    bool openLayer(const std::string& layerName);
    virtual bool createLayer(const std::string& layerName);
    void createFields();

    OGRDataSourceH m_dataset;
    OGRLayerH m_layer;
    StringList m_files;
    std::string m_layerName;
    std::string m_idxFilename;
    std::string m_driverName;
    std::string m_tileIndexColumnName;
    std::string m_srsColumnName;

    FieldMap m_fields;
};

class TileIndex : public TIndexBuilder
{
public:
    TileIndex(const std::string& idxFilename, const std::string& layerName,
        const std::string& driverName, const std::string& tileIndexColumnName,
        const std::string& srsColumnName);
private:
};

class StacIndex : public TIndexBuilder
{
    StacIndex(const std::string& idxFilename, const std::string& layerName, 
        const std::string& tileIndexColumnName, const std::string& srsColumnName);
};

} // namespace tindex
} // namespace pdal