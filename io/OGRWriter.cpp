/******************************************************************************
* Copyright (c) 2017, Hobu Inc. <info@hobu.co>
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following
* conditions are met:
*
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in
*       the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of Hobu, Inc. nor the names of its contributors
*       may be used to endorse or promote products derived from this
*       software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
* OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
* OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
* OF SUCH DAMAGE.
****************************************************************************/

#include "OGRWriter.hpp"

#include <sstream>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wredundant-decls"
#include <pdal/PointView.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/private/gdal/ErrorHandler.hpp>

#include <ogr_core.h>
#include <ogrsf_frmts.h>
#pragma GCC diagnostic pop

namespace pdal
{

static StaticPluginInfo const s_info
{
    "writers.ogr",
    "Write a point cloud as a set of OGR points/multipoints",
    "http://pdal.io/stages/writers.ogr.html",
    { "shp", "geojson" }
};

CREATE_STATIC_STAGE(OGRWriter, s_info)

OGRWriter::OGRWriter() : m_driver(nullptr), m_ds(nullptr), m_layer(nullptr),
    m_feature(nullptr), m_curCount(0), m_measureDim(Dimension::Id::Unknown),
    m_inTransaction(false)
{}


std::string OGRWriter::getName() const
{
    return s_info.name;
}


void OGRWriter::addArgs(ProgramArgs& args)
{
    args.add("filename", "Output filename", m_filename).setPositional();
    args.add("multicount", "Group 'multicount' points into a structure",
        m_multiCount, (size_t)1);
    args.add("measure_dim", "Use dimensions as a measure value",
        m_measureDimName);
    args.add("ogrdriver", "OGR writer driver name", m_driverName, m_driverName);
    args.add("ogr_options", "OGR layer creation options", m_ogrOptions);
    args.add("attr_dims", "Dimension to use as attributes, 'all' for all. Incompatible with multicount>1", m_attrDimNames);
}


void OGRWriter::initialize()
{
    gdal::registerDrivers();
    if (m_multiCount < 1)
        throwError("multicount must be greater than 0.");
    else if (m_multiCount > 1 && m_attrDimNames.size() > 0) {
        throwError("multicount > 1 incompatible with attr_dims");
    }
}


void OGRWriter::prepared(PointTableRef table)
{
    if (m_measureDimName.size())
    {
        m_measureDim = table.layout()->findDim(m_measureDimName);
        if (m_measureDim == Dimension::Id::Unknown)
            throwError("Dimension '" + m_measureDimName + "' (measure_dim) not "
                "found.");
    }

    if (m_driverName.empty())
    {
        if (FileUtils::extension(m_filename) == ".geojson")
            m_driverName = "GeoJSON";
        else
            m_driverName = "ESRI Shapefile";
    }

    // Build the attr dims list, replacing special keywords with the proper
    // field names.
    for (auto& name : m_attrDimNames)
    {
        if (name == "all")
        {
            m_attrDimNames.clear();
            for (auto& dim : table.layout()->dims())
            {
                switch(dim)
                {
                    // we don't need geometry attributes repeated as fields
                    case Dimension::Id::X:
                    case Dimension::Id::Y:
                    case Dimension::Id::Z:
                        break;

                    default:
                        if (dim != m_measureDim) {
                            m_attrDimNames.push_back(table.layout()->dimName(dim));
                        }
                }
            }
            break;
        }
        else
        {
            auto dim = table.layout()->findDim(name);
            if (dim == Dimension::Id::Unknown)
                throwError("Dimension '" + name + "' (attr_dims) not found.");
        }
    }
}


void OGRWriter::readyTable(PointTableRef table)
{
    m_driver = GetGDALDriverManager()->GetDriverByName(m_driverName.data());
    m_geomType = (m_multiCount == 1) ? wkbPointZM : wkbMultiPointZM;

    const auto& layout = table.layout();
    for(auto& name : m_attrDimNames)
    {
        auto dim = layout->findDim(name);
        auto dimType = layout->dimType(dim);
        OGRFieldType ogrType;

        switch(dimType)
        {
            case Dimension::Type::Signed8:
            case Dimension::Type::Unsigned8:
            case Dimension::Type::Signed16:
            case Dimension::Type::Unsigned16:
            case Dimension::Type::Signed32:
                ogrType = OFTInteger;
                break;
            case Dimension::Type::Unsigned32:
            case Dimension::Type::Signed64:
            case Dimension::Type::Unsigned64:  // error here?
                ogrType = OFTInteger64;
                break;
            case Dimension::Type::Float:
            case Dimension::Type::Double:
                ogrType = OFTReal;
                break;
            case Dimension::Type::None:
            default:
                throwError("Unknown type for dimension '" + name + "' (attr_dims).");
                continue;
        }
        auto ogrField = new OGRFieldDefn(name.c_str(), ogrType);
        m_attrs.emplace_back(dim, dimType, ogrField);
    }

}


void OGRWriter::readyFile(const std::string& filename,
    const SpatialReference& srs)
{
    m_curCount = 0;
    m_outputFilename = filename;

    // Dataset
    m_ds = m_driver->Create(filename.data(), 0, 0, 0, GDT_Unknown, nullptr);
    if (!m_ds)
        throwError("Unable to open OGR datasource '" + filename + "': " + CPLGetLastErrorMsg());

    // CRS
    if (!srs.empty())
    {
        if (m_srs.importFromWkt(srs.getWKT().data()) != OGRERR_NONE)
            throwError(std::string("Can't initialise OGR SRS: ") + CPLGetLastErrorMsg());
    }

    // Creation options
    std::vector<const char*> ogr_create_options;
    for(auto&& o:m_ogrOptions)
        ogr_create_options.push_back(o.c_str());
    ogr_create_options.push_back(nullptr);

    // Layer
    m_layer = m_ds->CreateLayer("points", &m_srs, m_geomType, const_cast<char**>(ogr_create_options.data()));
    if (!m_layer)
        throwError(std::string("Can't create OGR layer: ") + CPLGetLastErrorMsg());

    // Fields
    for(auto& attr : m_attrs)
    {
        auto& ogrField = std::get<2>(attr);
        if (m_layer->CreateField(&ogrField) != OGRERR_NONE)
            throwError(std::string("Can't create OGR field: ") + ogrField.GetNameRef());
    }

    // Reusable template feature
    m_feature = OGRFeature::CreateFeature(m_layer->GetLayerDefn());
    if (!m_feature)
        throwError(std::string("Can't create template OGR feature: ") + CPLGetLastErrorMsg());

    // Try to use a transaction for data sources that support it (e.g. GPKG),
    // otherwise new points may get auto-committed after each insert (very slow)
    if (m_ds->TestCapability( ODsCTransactions ) && m_ds->StartTransaction() == OGRERR_NONE)
        m_inTransaction = true;
}

void OGRWriter::writeView(const PointViewPtr view)
{
    m_curCount = 0;
    PointRef point(*view, 0);
    for (PointId idx = 0; idx < view->size(); ++idx)
    {
        point.setPointId(idx);
        processOne(point);
    }
}


bool OGRWriter::processOne(PointRef& point)
{
    double x = point.getFieldAs<double>(Dimension::Id::X);
    double y = point.getFieldAs<double>(Dimension::Id::Y);
    double z = point.getFieldAs<double>(Dimension::Id::Z);
    double m = point.getFieldAs<double>(m_measureDim);

    OGRPoint pt(x, y, z);
    if (m_measureDim != Dimension::Id::Unknown)
        pt.setM(m);

    m_curCount++;

    if (m_multiCount > 1)
        m_multiPoint.addGeometry(&pt);

    if (m_curCount % m_multiCount == 0)
    {
        if (m_multiCount > 1)
        {
            m_feature->SetGeometry(&m_multiPoint);
            m_multiPoint.empty();
        }
        else
        {
            m_feature->SetGeometry(&pt);

            for (auto it = std::begin(m_attrs); it != std::end(m_attrs); ++it)
            {
                const auto &dim = std::get<0>(*it);
                const auto &dimType = std::get<1>(*it);
                const auto &ogrField = std::get<2>(*it);
                size_t ogr_field_idx = std::distance(std::begin(m_attrs), it);

                switch(dimType)
                {
                    case Dimension::Type::Signed8:
                    case Dimension::Type::Unsigned8:
                    case Dimension::Type::Signed16:
                    case Dimension::Type::Unsigned16:
                    case Dimension::Type::Signed32:
                        m_feature->SetField(ogr_field_idx, point.getFieldAs<int>(dim));
                        break;

                    case Dimension::Type::Unsigned32:
                    case Dimension::Type::Unsigned64:
                    case Dimension::Type::Signed64:
                        m_feature->SetField(ogr_field_idx, point.getFieldAs<GIntBig>(dim));
                        break;

                    case Dimension::Type::Float:
                    case Dimension::Type::Double:
                        m_feature->SetField(ogr_field_idx, point.getFieldAs<double>(dim));
                        break;

                    default:
                        break;
                }
            }
        }

        if (m_layer->CreateFeature(m_feature))
            throwError(std::string("Can't create OGR feature: ") + CPLGetLastErrorMsg());

#if GDAL_VERSION_NUM >= GDAL_COMPUTE_VERSION(3,5,0)
        m_feature->Reset();
#else
        m_feature->SetFID(OGRNullFID);
#endif
    }
    return true;
}


void OGRWriter::doneFile()
{
    if (m_curCount % m_multiCount > 0) {
#if GDAL_VERSION_NUM >= GDAL_COMPUTE_VERSION(3,5,0)
        m_feature->Reset();
#else
        m_feature->SetFID(OGRNullFID);
#endif

        m_feature->SetGeometry(&m_multiPoint);

        if (m_layer->CreateFeature(m_feature))
            throwError(std::string("Can't create OGR feature: ") + CPLGetLastErrorMsg());
    }
    OGRFeature::DestroyFeature(m_feature);

    if (m_inTransaction && m_ds->CommitTransaction() != OGRERR_NONE)
        throwError(std::string("Failed to commit transaction in OGR: ") + CPLGetLastErrorMsg());
    m_inTransaction = false;

    GDALClose(m_ds);
    m_layer = nullptr;
    m_ds = nullptr;
}

} // namespace pdal

