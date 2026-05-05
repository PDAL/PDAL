
#include "TIndexDataset.hpp"

#include <ogr_api.h>
#include <cpl_string.h>

namespace pdal
{
namespace tindex
{

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

TIndexDataset::openDataset()
{
    m_dataset = OGROpen(m_idxFilename.c_str(), TRUE, NULL);
    return (bool)m_dataset;   
}

TIndexDataset::openLayer(const std::string& layerName)
{
    if (OGR_DS_GetLayerCount(m_dataset) == 1)
        m_layer = OGR_DS_GetLayer(m_dataset, 0);
    else if (m_layerName.size())
        m_layer = OGR_DS_GetLayerByName(m_dataset, layerName.c_str());

    return (bool)m_layer;
}

TIndexDataset::createDataset()
{
    OGRSFDriverH hDriver = OGRGetDriverByName(m_driverName.c_str());
    if (!hDriver)
    {
        std::ostringstream oss;

        oss << "Can't create dataset using driver '" << m_driverName <<
            "'. Driver is not available.";
        throw TIndexError(oss.str());
    }

    m_dataset = OGR_Dr_CreateDataSource(hDriver, m_args.idxFilename.c_str(), NULL);
    return (bool)m_dataset;
}

TIndexDataset::createLayer(const std::string& layerName, const std::string& srsString,
    const StringList& lcOptions)
{
    gdal::SpatialRef srs(srsString);
    // This used to just write to log w/ LogLevel::Error
    if (!srs)
        throw TIndexError("Unable to import srs for layer creation");

    char** papszOptions = NULL;
    for (const std::string& s : m_args.lcOptions)
        papszOptions = CSLAddString(papszOptions, s.c_str());

    m_layer = OGR_DS_CreateLayer(m_dataset, m_args.layerName.c_str(),
        srs.get(), wkbMultiPolygon, papszOptions);

    CSLDestroy(papszOptions);
}

TIndexDataset::createField(const std::string& name, const OGRFieldType fieldType,
    const OGRFieldSubType subtype) 
{
    OGRFieldDefnH hFieldDefn = 
        OGR_Fld_Create(name.c_str(), fieldType);
    if (subtype != OFSTNone)
        OGR_Fld_SetSubType(hFieldDefn, subtype);
    OGR_L_CreateField(m_layer, hFieldDefn, TRUE);
    OGR_Fld_Destroy(hFieldDefn);
}

} // namespace tindex
} // namespace pdal
