
#include <pdal/private/gdal/GDALUtils.hpp>
#include <pdal/private/gdal/SpatialRef.hpp>

#if __has_include(<gdal_fwd.h>)
#include <gdal_fwd.h>
#else
using OGRDataSourceH = void *;
using OGRLayerH = void *;
using OGRFeatureH = void *;
using OGRFeatureDefnH = void *;
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

namespace tindex
{

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
using FieldMap = std::map<std::string, FieldInfo>;

class TIndexDataset
{
public:
    TIndexDataset(const std::string& idxFilename, const std::string& driverName);
    ~TIndexDataset();
/*
    void defineField(const std::string& name, const OGRFieldType fieldType)
    {
        
    }
    void defineField(const std::string& name, const OGRFieldType fieldType, 
        const OGRFieldSubType subtype)
    {
        m_fields.try_emplace(name, fieldType, subtype);
    }*/
    bool openDataset();
    bool createDataset();
    bool openLayer(const std::string& layerName);
    bool createLayer(const std::string& layerName, const std::string& srsString, 
        const StringList& lcOptions);
    void createField(const std::string& name, const OGRFieldType fieldType, 
        const OGRFieldSubType subtype = OFSTNone);

private:
    OGRLayerH m_layer;
    OGRDataSourceH m_dataset;
    OGRFeatureDefnH m_layerDefn;

    std::string m_idxFilename;
    std::string m_driverName;
    int m_maxFieldSize;
    FieldMap m_fields;
};

//!! define this elsewhere, or use pdal_error or gdal::GDALError or something
class TIndexError : public std::runtime_error
{
public:
    TIndexError(const std::string txt) : std::runtime_error(txt)
    {}
};

} // namespace tindex
} // namespace pdal