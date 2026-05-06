
#include "TIndexDataset.hpp"

#include <pdal/Polygon.hpp>

#include <ogr_api.h>
#include <cpl_string.h>

namespace pdal
{
namespace tindex
{
///
/// TIndexFeature
///

TIndexFeature::TIndexFeature(OGRFeatureDefnH layerDefn, int maxFieldSize) 
    : m_maxFieldSize(maxFieldSize)
{
    m_feature = OGR_F_Create(layerDefn);
}

TIndexFeature::~TIndexFeature() 
{
    OGR_F_Destroy(m_feature);
}

void TIndexFeature::setField(int fieldIdx, const std::string& value)
{
    if (m_maxFieldSize == 0 || strlen(value) <= m_maxFieldSize)
    {
        OGR_F_SetFieldString(m_feature, fieldIdx, value.c_str());
    }
    else
    {
        std::ostringstream oss;
        OGRFieldDefnH hFieldDefn = OGR_F_GetFieldDefnRef(m_feature, fieldIdx);

        oss << "value for field'" << OGR_Fld_GetNameRef(hFieldDefn) << "' has " << strlen(value) <<
            " characters; ESRI Shapefile driver supports a maximum of 254.";

        throw TIndexError(oss.str());
    }
}

void TIndexFeature::setField(int fieldIdx, const StringList& values)
{
    char** csl = NULL;
    for (const std::string& s : values)
        csl = CSLAddString(csl, s.c_str());
    OGR_F_SetFieldStringList(m_feature, fieldIdx, csl);
}

void TIndexFeature::setField(int fieldIdx, const int value)
{
    OGR_F_SetFieldInteger(m_feature, fieldIdx, value);
}

void TIndexFeature::setField(int fieldIdx, const tm& tyme)
{
    OGR_F_SetFieldDateTime(m_feature, fieldIdx,
        tyme.tm_year + 1900, tyme.tm_mon + 1, tyme.tm_mday, tyme.tm_hour,
        tyme.tm_min, tyme.tm_sec, 100);
}

bool TIndexFeature::setGeometry(pdal::Polygon& polygon)
{
    OGRGeometryH geometry = polygon.getOGRHandle();
    return (OGR_F_SetGeometry(m_feature, geometry) == OGRERR_NONE);
}

///
/// TIndexDataset
///

TIndexDataset::TIndexDataset(const std::string& idxFilename, const std::string& driverName) 
    : m_layer(nullptr), 
      m_dataset(nullptr), 
      m_layerDefn(nullptr), 
      m_idxFilename(idxFilename),
      m_driverName(driverName),
      m_maxFieldSize(0)
{
    if (m_driverName == "ESRI Shapefile")
        m_maxFieldSize = 254;
}

TIndexDataset::~TIndexDataset() 
{
    if (m_dataset)
        OGR_DS_Destroy(m_dataset);
}

bool TIndexDataset::openDataset()
{
    m_dataset = OGROpen(m_idxFilename.c_str(), TRUE, NULL);
    return (bool)m_dataset;   
}

bool TIndexDataset::openLayer(const std::string& layerName)
{
    if (OGR_DS_GetLayerCount(m_dataset) == 1)
        m_layer = OGR_DS_GetLayer(m_dataset, 0);
    else if (layerName.size())
        m_layer = OGR_DS_GetLayerByName(m_dataset, layerName.c_str());

    return (bool)m_layer;
}

bool TIndexDataset::createDataset()
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

bool TIndexDataset::createLayer(const std::string& layerName, const std::string& srsString,
    const StringList& lcOptions)
{
    gdal::SpatialRef srs(srsString);
    // This used to just write to log w/ LogLevel::Error
    if (!srs)
        throw TIndexError("Unable to import srs for layer creation");

    char** papszOptions = NULL;
    for (const std::string& s : lcOptions)
        papszOptions = CSLAddString(papszOptions, s.c_str());

    m_layer = OGR_DS_CreateLayer(m_dataset, layerName.c_str(),
        srs.get(), wkbMultiPolygon, papszOptions);

    CSLDestroy(papszOptions);

    return (bool)m_layer;
}

void TIndexDataset::createField(const std::string& name, const OGRFieldType fieldType,
    const OGRFieldSubType subtype) 
{
    OGRFieldDefnH hFieldDefn = 
        OGR_Fld_Create(name.c_str(), fieldType);
    if (subtype != OFSTNone)
        OGR_Fld_SetSubType(hFieldDefn, subtype);
    OGR_L_CreateField(m_layer, hFieldDefn, TRUE);
    OGR_Fld_Destroy(hFieldDefn);
}

int TIndexDataset::getFieldIdx(const std::string& fieldName) 
{
    if (!m_layerDefn)
        m_layerDefn = OGR_L_GetLayerDefn(m_layer);
    return OGR_L_GetFieldIndex(m_layerDefn, name.c_str());
}

TIndexFeature TIndexDataset::buildFeature() 
{
    if (!m_layerDefn)
        m_layerDefn = OGR_L_GetLayerDefn(m_layer);
    return TIndexFeature(m_layerDefn, m_maxFieldSize);
}

bool TIndexDataset::createFeature(TIndexFeature& feature) 
{
    return (OGR_L_CreateFeature(m_layer, feature.getFeature()) == OGRERR_NONE);
}

bool TIndexDataset::queryLayer(const std::string& query) 
{
    OGRErr err = OGR_L_SetAttributeFilter(m_layer, query.c_str());
    if (err != OGRERR_NONE)
    {
        std::ostringstream oss;
        oss << "Unable to set attribute filter with OGR error '" <<
            CPLGetLastErrorMsg() << "'";
        throw TIndexError(oss.str());
    }

    bool output(false);
    OGR_L_ResetReading(m_layer);
    auto hFeat = OGR_L_GetNextFeature(m_layer);
    if( hFeat )
    {
        OGR_F_Destroy(hFeat);
        output = true;
    }
    OGR_L_ResetReading(m_layer);
    OGR_L_SetAttributeFilter(m_layer, NULL);
    return output;
}

} // namespace tindex
} // namespace pdal
