
#include <pdal/private/gdal/GDALUtils.hpp>
#include <pdal/private/gdal/SpatialRef.hpp>

#include <deque>

#include <ogr_api.h>

#if __has_include(<gdal_fwd.h>)
#include <gdal_fwd.h>
#else
using OGRDataSourceH = void *;
using OGRLayerH = void *;
using OGRFeatureH = void *;
using OGRFeatureDefnH = void *;
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

/**
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
**/

struct Field
{
    Field(const std::string& name, OGRFieldType fieldType, OGRFieldSubType subtype = OFSTNone) :
        m_name(name), m_fieldType(fieldType), m_subtype(subtype)
    {}

    std::string m_name;
    OGRFieldType m_fieldType;
    OGRFieldSubType m_subtype = OFSTNone;
    int m_index = -1;  // Integer value proxy for field.
};

class TIndexFeature
{
public:
    TIndexFeature(OGRFeatureDefnH layerDefn, int maxFieldSize);
    ~TIndexFeature();

    OGRFeatureH getFeature() { return m_feature; }

    void setField(Field *field, const std::string& value);
    void setField(Field *field, const StringList& values);
    void setField(Field *field, const int value);
    void setField(Field *field, const tm& tyme);
    bool setGeometry(const Polygon& polygon);

private:
    OGRFeatureH m_feature;
    int m_maxFieldSize;
};

class TIndexDataset
{
public:
    TIndexDataset(const std::string& idxFilename, const std::string& driverName);
    ~TIndexDataset();

    bool openDataset();
    bool createDataset();
    bool openLayer(const std::string& layerName);
    bool createLayer(const std::string& layerName, const std::string& srsString,
        const StringList& lcOptions);
    void createFields();
    TIndexFeature buildFeature();
    bool createFeature(TIndexFeature& feature);
    int getFieldIdx(const std::string& fieldName);
    bool queryLayer(const std::string& query);
    Field *defineField(const std::string& name, const OGRFieldType fieldType);
    Field *defineField(const std::string& name, const OGRFieldType fieldType,
        const OGRFieldSubType subtype);

private:
    OGRLayerH m_layer;
    OGRDataSourceH m_dataset;
    OGRFeatureDefnH m_layerDefn;

    std::string m_idxFilename;
    std::string m_driverName;
    size_t m_maxFieldSize;
    std::deque<Field> m_fields;
};

} // namespace tindex
} // namespace pdal
