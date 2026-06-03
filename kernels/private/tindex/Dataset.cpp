
#include "Dataset.hpp"
#include "TIndexError.hpp"

#include <pdal/Polygon.hpp>

#include <ogr_api.h>
#include <cpl_string.h>

namespace pdal
{
namespace tindex
{
///
/// Feature
///

Feature::Feature(OGRFeatureDefnH layerDefn, int maxFieldSize)
    : m_maxFieldSize(maxFieldSize)
{
    m_feature = OGR_F_Create(layerDefn);
}

//!! Max field size doesn't matter since this is for reading
Feature::Feature(OGRFeatureH feature) 
    : m_feature(feature), m_maxFieldSize(0)
{}

Feature::~Feature()
{
    OGR_F_Destroy(m_feature);
}

std::string Feature::getField(Field *field)
{
    return OGR_F_GetFieldAsString(m_feature, field->m_index);
}

void Feature::setField(Field *field, const std::string& value)
{
    if (m_maxFieldSize == 0 || value.size() <= (size_t)m_maxFieldSize)
    {
        OGR_F_SetFieldString(m_feature, field->m_index, value.c_str());
    }
    else
    {
        OGRFieldDefnH hFieldDefn = OGR_F_GetFieldDefnRef(m_feature, field->m_index);

        throw TIndexError("Value for field'" + std::string(OGR_Fld_GetNameRef(hFieldDefn)) +
            "' exceeds supported length of " + std::to_string(m_maxFieldSize) + ".");
    }
}

void Feature::setField(Field *field, const StringList& values)
{
    std::vector<char *> sl;
    for (const std::string& s : values)
        sl.push_back(const_cast<char *>(s.data()));
    sl.push_back(nullptr);
    OGR_F_SetFieldStringList(m_feature, field->m_index, sl.data());
}

void Feature::setField(Field *field, const int value)
{
    OGR_F_SetFieldInteger(m_feature, field->m_index, value);
}

void Feature::setField(Field *field, const tm& tyme)
{
    OGR_F_SetFieldDateTime(m_feature, field->m_index,
        tyme.tm_year + 1900, tyme.tm_mon + 1, tyme.tm_mday, tyme.tm_hour,
        tyme.tm_min, tyme.tm_sec, 100);
}

bool Feature::setGeometry(const Polygon& polygon)
{
    return OGR_F_SetGeometry(m_feature, const_cast<Polygon &>(polygon).getOGRHandle()) ==
        OGRERR_NONE;
}

///
/// Dataset
///

Dataset::Dataset(const std::string& idxFilename, const std::string& driverName)
    : m_layer(nullptr),
      m_dataset(nullptr),
      m_layerDefn(nullptr),
      m_idxFilename(idxFilename),
      m_driverName(driverName),
      m_maxFieldSize(0)
{
    gdal::registerDrivers();

    if (m_driverName == "ESRI Shapefile")
        m_maxFieldSize = 254;
}

Field *Dataset::defineField(const std::string& name, const OGRFieldType fieldType)
{
    return &m_fields.emplace_back(name, fieldType);
}

Field *Dataset::defineField(const std::string& name, const OGRFieldType fieldType,
        const OGRFieldSubType subtype)
{
    return &m_fields.emplace_back(name, fieldType, subtype);
}

bool Dataset::openDataset()
{
    m_dataset = OGROpen(m_idxFilename.c_str(), TRUE, NULL);
    return (bool)m_dataset;
}

bool Dataset::openLayer(const std::string& layerName)
{
    if (layerName.size())
        m_layer = OGR_DS_GetLayerByName(m_dataset, layerName.c_str());
    else if (OGR_DS_GetLayerCount(m_dataset) == 1)
        m_layer = OGR_DS_GetLayer(m_dataset, 0);

    return (bool)m_layer;
}

bool Dataset::createDataset()
{
    OGRSFDriverH hDriver = OGRGetDriverByName(m_driverName.c_str());
    if (!hDriver)
    {
        std::ostringstream oss;

        oss << "Can't create dataset using driver '" << m_driverName <<
            "'. Driver is not available.";
        throw TIndexError(oss.str());
    }

    m_dataset = OGR_Dr_CreateDataSource(hDriver, m_idxFilename.c_str(), NULL);
    return (bool)m_dataset;
}

bool Dataset::createLayer(const std::string& layerName, const std::string& srsString,
    const StringList& lcOptions)
{
    gdal::SpatialRef srs(srsString);
    if (!srs)
        throw TIndexError("Unable to import srs for layer creation");

    std::vector<char *> opts;
    for (const std::string& s : lcOptions)
        opts.push_back(const_cast<char *>(s.data()));
    opts.push_back(nullptr);

    m_layer = OGR_DS_CreateLayer(m_dataset, layerName.c_str(),
        srs.get(), wkbMultiPolygon, opts.data());

    return (bool)m_layer;
}

void Dataset::createFields()
{
    for (Field& field : m_fields)
    {
        OGRFieldDefnH hFieldDefn = OGR_Fld_Create(field.m_name.c_str(), field.m_fieldType);
        OGR_Fld_SetSubType(hFieldDefn, field.m_subtype);
        OGR_L_CreateField(m_layer, hFieldDefn, TRUE);
        OGR_Fld_Destroy(hFieldDefn);
    }
    // Once all the fields have been created, the final layerDefn is ready
    // & we can get the field indices
    getFieldIndexes();
}

void Dataset::getFieldIndexes()
{
    if (!m_layerDefn)
        m_layerDefn = OGR_L_GetLayerDefn(m_layer);
    for (Field& field : m_fields)
    {
        field.m_index = OGR_FD_GetFieldIndex(m_layerDefn, field.m_name.c_str());
    }
}

Feature Dataset::buildFeature()
{
    return Feature(m_layerDefn, m_maxFieldSize);
}

Feature Dataset::getNextFeature()
{
    return Feature(OGR_L_GetNextFeature(m_layer));
}

void Dataset::resetReading()
{
    OGR_L_ResetReading(m_layer);
}

bool Dataset::createFeature(Feature& feature)
{
    return (OGR_L_CreateFeature(m_layer, feature.getFeature()) == OGRERR_NONE);
}

void Dataset::setSpatialFilter(const Polygon& polygon)
{
    OGR_L_SetSpatialFilter(m_layer, const_cast<Polygon &>(polygon).getOGRHandle());
}

bool Dataset::queryLayer(const std::string& query)
{
    OGRErr err = OGR_L_SetAttributeFilter(m_layer, query.c_str());
    if (err != OGRERR_NONE)
    {
        std::ostringstream oss;
        oss << "Unable to set attribute filter with OGR error '" <<
            CPLGetLastErrorMsg() << "'";
        throw TIndexError(oss.str());
    }

    OGR_L_ResetReading(m_layer);
    auto hFeat = OGR_L_GetNextFeature(m_layer);
    bool output = (bool)hFeat;
    if(hFeat)
        OGR_F_Destroy(hFeat);

    OGR_L_ResetReading(m_layer);
    OGR_L_SetAttributeFilter(m_layer, NULL);
    return output;
}

} // namespace tindex
} // namespace pdal
