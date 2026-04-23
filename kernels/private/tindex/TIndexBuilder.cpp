#include "TIndexBuilder.hpp"

#include <filters/private/hexer/HexGrid.hpp>

namespace pdal
{
namespace tindex
{

struct FieldInfo
{
    FieldInfo(const OGRFieldType fieldType) : m_fieldType(fieldType) 
    {}
    FieldInfo(const OGRFieldType fieldType, OGRFieldSubType subtype) 
        : m_fieldType(fieldType), m_subtype(subtype) 
    {}
    void setIdx(int idx) { m_fieldIdx = idx; }

    OGRFieldType m_fieldType;
    OGRFieldSubType m_subtype = OFSTNone;
    int m_fieldIdx = -1;
};

class TindexBoundary : public Filter, public Streamable
{
public:
    TindexBoundary(int32_t density, double edgeLength, uint32_t sampleSize)
        : m_density(density), m_edgeLength(edgeLength),
        m_sampleSize(sampleSize)
    {}
    ~TindexBoundary()
    {}

    std::string getName() const
    { return "tindex-boundary"; }
    double height()
    { return m_grid->height(); }
    std::string toWKT()
    {
        std::ostringstream out;
        out.setf(std::ios_base::fixed, std::ios_base::floatfield);
        out.precision(10);
        m_grid->toWKT(out);
        return out.str();
    }
private:
    std::unique_ptr<hexer::HexGrid> m_grid;
    int32_t m_density;
    double m_edgeLength;
    uint32_t m_sampleSize;

    virtual void ready(PointTableRef table)
    {
        if (m_edgeLength == 0.0)
        {
            m_grid.reset(new hexer::HexGrid(m_density));
            m_grid->setSampleSize(m_sampleSize);
        }
        else
            m_grid.reset(new hexer::HexGrid(m_edgeLength * sqrt(3), m_density));
    }
    virtual void filter(PointView& view)
    {
        PointRef p(view, 0);

        for (PointId idx = 0; idx < view.size(); ++idx)
        {
            p.setPointId(idx);
            processOne(p);
        }
    }
    virtual bool processOne(PointRef& point)
    {
        double x = point.getFieldAs<double>(Dimension::Id::X);
        double y = point.getFieldAs<double>(Dimension::Id::Y);
        m_grid->addXY(x, y);
        return true;
    }
    virtual void spatialReferenceChanged(const SpatialReference& srs)
    { setSpatialReference(srs); }
    virtual void done(PointTableRef table)
    {
        try
        {
            m_grid->findShapes();
            m_grid->findParentPaths();
        }
        catch (hexer::hexer_error& e)
        {
            throwError(e.what());
            m_grid.reset(new hexer::HexGrid(m_density));
        }
    }
};

//
// Base class
//

TIndexBuilder::TIndexBuilder(const std::string& idxFilename, const std::string& layerName, 
    const std::string& driverName, const std::string& tileIndexColumnName, 
    const std::string& srsColumnName)
    : m_idxFilename(idxFilename), m_layerName(layerName), m_driverName(driverName),
        m_tileIndexColumnName(tileIndexColumnName), m_srsColumnName(srsColumnName) 
{
    m_fields.emplace(m_tileIndexColumnName, OFTString);
    m_fields.emplace(m_srsColumnName, OFTString);
}

TIndexBuilder::~TIndexBuilder() {}

void TIndexBuilder::createOrOpenFile(const StringList& lcOptions)
{
    // Open or create the dataset.
    if (!openDataset(m_idxFilename))
        if (!createDataset(m_idxFilename))
        {
            std::ostringstream out;
            out << "Couldn't open or create index dataset file '" <<
                m_idxFilename << "'.";
            throw pdal_error(out.str());
        }

    // Open or create a layer
    if (!openLayer(m_layerName))
    {
        if (createLayer(m_layerName))
            createFields();
        else
        {
            std::ostringstream out;
            out << "Couldn't open or create layer '" << m_layerName <<
                "' in output file '" << m_idxFilename << "'.";
            throw pdal_error(out.str());
        }
    }
}

bool TIndexBuilder::openDataset(const std::string& filename)
{ return false; }

bool TIndexBuilder::createDataset(const std::string& filename) 
{ return false; }

bool TIndexBuilder::openLayer(const std::string& layerName)
{     
    if (OGR_DS_GetLayerCount(m_dataset) == 1)
        m_layer = OGR_DS_GetLayer(m_dataset, 0);
    else if (layerName.size())
        m_layer = OGR_DS_GetLayerByName(m_dataset, m_layerName.c_str());

    return (bool)m_layer;
}

bool TIndexBuilder::createLayer(const std::string& layerName)
{ return false; }

void TIndexBuilder::createFields()
{
    for (auto& field : m_fields)
    {
        OGRFieldDefnH hFieldDefn = OGR_Fld_Create(
            field.first.c_str(), field.second.m_fieldType);
        if (field.second.m_subType != OFSTNone)
            OGR_Fld_SetSubType(hFieldDefn, field.second.m_subType);
        OGR_L_CreateField(m_layer, hFieldDefn, TRUE);
        OGR_Fld_Destroy(hFieldDefn);
    }
}

//
// Standard class
//

TileIndex::TileIndex(const std::string& idxFilename, const std::string& layerName,
    const std::string& driverName, const std::string& tileIndexColumnName,
    const std::string& srsColumnName)
    : TIndexBuilder(idxFilename, layerName, driverName, tileIndexColumnName, srsColumnName) 
{
    m_fields.emplace("modified", OFTDateTime);
    m_fields.emplace("created", OFTDateTime);
}

//
// STAC class
//

StacIndex::StacIndex(const std::string& idxFilename,
    const std::string& layerName, const std::string& tileIndexColumnName,
    const std::string& srsColumnName)
    : TIndexBuilder(idxFilename, layerName, "Parquet", tileIndexColumnName, srsColumnName) 
{
    m_fields.emplace("proj:projjson", OFTString, OFSTJSON);
    m_fields.emplace("datetime", OFTDateTime);
    m_fields.emplace("links", OFTString, OFSTJSON);
    m_fields.emplace("id", OFTString);
    m_fields.emplace("stac_extensions", OFTStringList);
    m_fields.emplace("stac_version", OFTString);
    m_fields.emplace("pc:count", OFTInteger);
    m_fields.emplace("pc:encoding", OFTString);
    m_fields.emplace("pc:type", OFTString);
    m_fields.emplace("pc:schemas", OFTString, OFSTJSON);
    m_fields.emplace("pc:statistics", OFTString, OFSTJSON);
}

} // namespace tindex
} // namespace pdal
