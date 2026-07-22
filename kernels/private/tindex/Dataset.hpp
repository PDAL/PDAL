
#include <pdal/private/gdal/GDALUtils.hpp>
#include <pdal/private/gdal/SpatialRef.hpp>

#include <deque>

#include <ogr_api.h>
#include <nlohmann/json.hpp>

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

struct Field
{
    Field(const std::string& name, OGRFieldType fieldType, OGRFieldSubType subtype) :
        m_name(name), m_type(fieldType), m_subtype(subtype)
    {}
    virtual ~Field() {}

    bool valid() const { return !m_name.empty(); }

    std::string m_name;
    OGRFieldType m_type;
    OGRFieldSubType m_subtype;
    int m_index = -1;  // Integer value proxy for field.
};

struct StaticField : public Field
{
    StaticField(const std::string& name, const NL::json& value) :
        Field(name, OFTString, OFSTNone), m_value(value)
    {
        if (m_value.is_number_integer())
            m_type = OFTInteger64;
        else if (m_value.is_number_float())
            m_type = OFTReal;
        else if (m_value.is_array())
            m_type = OFTStringList;
        else if (!m_value.is_string())
            m_subtype = OFSTJSON; // Dump the value when writing
    }

    NL::json m_value;
};

class Feature
{
public:
    Feature(OGRFeatureDefnH layerDefn, int maxFieldSize);
    Feature(OGRFeatureH feature);
    ~Feature();

    OGRFeatureH getFeature() { return m_feature; }

    void setField(StaticField *field);
    void setField(Field *field, const std::string& value);
    void setField(Field *field, const StringList& values);
    void setField(Field *field, const int64_t value);
    void setField(Field *field, const uint64_t value);
    void setField(Field *field, const double value);
    void setField(Field *field, const tm& tyme);
    void setField(Field *field, const std::vector<double>& values);
    std::string getField(Field *field);
    bool setGeometry(const Polygon& polygon);

private:
    OGRFeatureH m_feature;
    int m_maxFieldSize;
};

class Dataset
{
public:
    Dataset(const std::string& idxFilename, const std::string& driverName);
    ~Dataset() {}

    bool openDataset();
    bool createDataset();
    bool openLayer(const std::string& layerName);
    bool createLayer(const std::string& layerName, const std::string& srsString,
        const StringList& lcOptions);
    void createFields();
    Feature buildFeature();
    Feature getNextFeature();
    void resetReading();
    bool createFeature(Feature& feature);
    void getFieldIndexes();
    bool queryLayer(const std::string& query);
    void setSpatialFilter(const Polygon& polygon);
    Field *defineField(const std::string& name, const OGRFieldType fieldType,
        const OGRFieldSubType subtype = OFSTNone);
    StaticField *defineField(const std::string& name, const NL::json& value);
    void sortFields();

private:
    OGRLayerH m_layer;
    OGRDataSourceH m_dataset;
    OGRFeatureDefnH m_layerDefn;

    std::string m_idxFilename;
    std::string m_driverName;
    size_t m_maxFieldSize;
    std::vector<Field *> m_fields;
    std::deque<Field> m_dynamicFields;
    std::deque<StaticField> m_staticFields;
};

} // namespace tindex
} // namespace pdal
