/******************************************************************************
* Copyright (c) 2015, Howard Butler (howard@hobu.co)
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
*     * Neither the name of Hobu, Inc. or Flaxen Geo Consulting nor the
*       names of its contributors may be used to endorse or promote
*       products derived from this software without specific prior
*       written permission.
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

#include "TIndexReader.hpp"
#include <pdal/GDALUtils.hpp>
#include <pdal/pdal_macros.hpp>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "readers.tindex",
    "TileIndex Reader",
    "http://pdal.io/stages/readers.tindex.html" );

CREATE_STATIC_PLUGIN(1, 0, TIndexReader, Reader, s_info)

std::string TIndexReader::getName() const { return s_info.name; }

Options TIndexReader::getDefaultOptions()
{
    Options options;
    Option sql("sql", "", "OGR-compatible SQL statement for querying tile index layer");
    options.add(sql);
    Option polygon("polygon", "", "WKT Polygon or MultiPolygon describing spatial extent to query");
    options.add(polygon);
    Option bounds("bounds", "", "PDAL-style bounds to limit query window (exclusive of --polygon)");
    options.add(bounds);
    Option lyr_name("lyr_name", "", "OGR layer name from which to read tile index layer");
    options.add(lyr_name);
    Option tindex_name("tindex_name", "", "OGR column name from which to read tile index location");
    options.add(tindex_name);
    Option a_srs("a_srs", "", "Override SRS of geometry in the tile index");
    options.add(a_srs);
    Option t_srs("t_srs", "", "Transform SRS of tile index geometry");
    options.add(t_srs);
    Option srs_column("srs_column", "", "Column to use for SRS");
    options.add(srs_column);
    return options;
}


TIndexReader::FieldIndexes TIndexReader::getFields()
{
    FieldIndexes indexes;

    void *fDefn = OGR_L_GetLayerDefn(m_layer);

    indexes.m_filename = OGR_FD_GetFieldIndex(fDefn,
        m_tileIndexColumnName.c_str());
    if (indexes.m_filename < 0)
    {
        std::ostringstream out;

        out << "Unable to find field '" << m_tileIndexColumnName <<
            "' in file '" << m_filename << "'.";
        throw pdal_error(out.str());
    }
    indexes.m_srs = OGR_FD_GetFieldIndex(fDefn, m_srsColumnName.c_str());
    if (indexes.m_srs < 0)
    {
        std::ostringstream out;

        out << "Unable to find field '" << m_srsColumnName << "' in file '" <<
            m_filename << "'.";
        throw pdal_error(out.str());
    }

    indexes.m_ctime = OGR_FD_GetFieldIndex(fDefn, "created");
    indexes.m_mtime = OGR_FD_GetFieldIndex(fDefn, "modified");

    return indexes;
}


std::vector<TIndexReader::FileInfo> TIndexReader::getFiles()
{
    std::vector<TIndexReader::FileInfo> output;

    OGR_L_ResetReading(m_layer);
    FieldIndexes indexes = getFields();

    while (true)
    {
        OGRFeatureH feature = OGR_L_GetNextFeature(m_layer);
        if (!feature)
            break;

        FileInfo fileInfo;
        fileInfo.m_filename =
            OGR_F_GetFieldAsString(feature, indexes.m_filename);
        fileInfo.m_srs =
            OGR_F_GetFieldAsString(feature, indexes.m_srs);
        output.push_back(fileInfo);

        OGR_F_Destroy(feature);
    }

    return output;
}


void TIndexReader::processOptions(const Options& options)
{
    m_layerName = options.getValueOrDefault<std::string>("lyr_name", "pdal");
    m_srsColumnName = options.getValueOrDefault<std::string>("srs_column",
        "srs");
    m_tileIndexColumnName =
        options.getValueOrDefault<std::string>("tindex_name", "location");
    m_sql = options.getValueOrDefault<std::string>("sql", "");

    BOX2D boundary = options.getValueOrDefault<BOX2D>("boundary", BOX2D());
    m_wkt = boundary.toWKT();
    if (m_wkt.empty())
        m_wkt = options.getValueOrDefault<std::string>("wkt");

    m_tgtSrsString = options.getValueOrDefault<std::string>("t_srs",
        "EPSG:4326");
    m_filterSRS = options.getValueOrDefault<std::string>("filter_srs");
    m_attributeFilter = options.getValueOrDefault<std::string>("where");
    m_dialect = options.getValueOrDefault<std::string>("dialect", "OGRSQL");

    m_out_ref.reset(new gdal::SpatialRef());
}


void TIndexReader::addDimensions(PointLayoutPtr layout)
{
    using namespace pdal::Dimension::Type;
    layout->registerDim(pdal::Dimension::Id::X);
    layout->registerDim(pdal::Dimension::Id::Y);
    layout->registerDim(pdal::Dimension::Id::Z);
}


pdal::Dimension::IdList TIndexReader::getDefaultDimensions()
{
    return Dimension::IdList();
}


void TIndexReader::initialize()
{
    log()->get(LogLevel::Debug) << "Opening file " << m_filename <<
        std::endl;

    GlobalEnvironment::get().wakeGDALDrivers();

    m_dataset = OGROpen(m_filename.c_str(), FALSE, NULL);
    if (!m_dataset)
    {
        std::stringstream oss;
        oss << "unable to datasource '" << m_filename << "'";
        throw pdal::pdal_error(oss.str());
    }

    OGRGeometryH geometry(0);
    if (m_sql.size())
    {
        m_layer = OGR_DS_ExecuteSQL(m_dataset, m_sql.c_str(), geometry,
            m_dialect.c_str());
    }
    else
    {
        m_layer = OGR_DS_GetLayerByName(m_dataset, m_layerName.c_str());
    }
    if (!m_layer)
    {
        std::stringstream oss;
        oss << getName() << ": Unable to open layer '" << m_layerName <<
            "' from OGR datasource '" << m_filename << "'";
        throw pdal::pdal_error(oss.str());
    }

    m_out_ref->setFromLayer(m_layer);

    // Override the SRS if the user set one, otherwise, take it
    // from the layer
    if (m_tgtSrsString.size())
        m_out_ref.reset(new gdal::SpatialRef(m_tgtSrsString));
    else
        m_out_ref.reset(new gdal::SpatialRef(m_out_ref->wkt()));

    setSpatialReference(SpatialReference(m_out_ref->wkt()));

    std::unique_ptr<gdal::Geometry> wkt_g;

    // If the user set either explicit 'polygon' or 'boundary' options
    // we will filter by that geometry. The user can set a 'filter_srs'
    // option to override the SRS of the input geometry and we will
    // reproject to the output projection as needed.
    if (m_wkt.size())
    {
        // Reproject the given wkt to the output SRS so
        // filtering/cropping works
        gdal::SpatialRef assign(m_filterSRS);
        gdal::Geometry before(m_wkt, assign);
        before.transform(*m_out_ref);

        wkt_g.reset (new gdal::Geometry(before.wkt(), *m_out_ref));

        geometry = wkt_g->get();
        m_wkt = wkt_g->wkt();
        OGR_L_SetSpatialFilter(m_layer, geometry);
    }

    if (m_attributeFilter.size())
    {
        OGRErr err = OGR_L_SetAttributeFilter(m_layer,
            m_attributeFilter.c_str());
        if (err != OGRERR_NONE)
        {
            std::stringstream oss;
            oss << getName() << ": Unable to set attribute filter '"
                << m_attributeFilter << "' for OGR datasource '"
                << m_filename << "'";
            throw pdal::pdal_error(oss.str());
        }
    }

    Options cropOptions;
    if (m_wkt.size())
        cropOptions.add("polygon", m_wkt);

    for (auto f : getFiles())
    {
        log()->get(LogLevel::Debug) << "Adding file "
                                    << f.m_filename
                                    << " to merge filter" <<std::endl;

        std::string driver = m_factory.inferReaderDriver(f.m_filename);
        Stage *reader = m_factory.createStage(driver);
        if (!reader)
        {
            std::stringstream out;
            out << "Unable to create reader for file '"
                << f.m_filename << "'.";
            throw pdal_error(out.str());
        }
        Options readerOptions;
        readerOptions.add("filename", f.m_filename);
        reader->setOptions(readerOptions);
        Stage *premerge = reader;

        if (m_tgtSrsString != f.m_srs &&
            (m_tgtSrsString.size() && f.m_srs.size()))
        {
            Stage *repro = m_factory.createStage("filters.reprojection");
            repro->setInput(*reader);
            Options reproOptions;
            reproOptions.add("out_srs", m_tgtSrsString);
            reproOptions.add("in_srs", f.m_srs);
            log()->get(LogLevel::Debug2) << "Repro = "
                                         << m_tgtSrsString << "/"
                                         << f.m_srs << "!\n";
            repro->setOptions(reproOptions);
            premerge = repro;
        }

        // WKT is set even if we're using a bounding box for filtering, so
        // can be used as a test here.
        if (!m_wkt.empty())
        {
            Stage *crop = m_factory.createStage("filters.crop");
            crop->setOptions(cropOptions);
            crop->setInput(*premerge);
            log()->get(LogLevel::Debug3) << "Cropping data with wkt '"
                                         << m_wkt << "'" << std::endl;
            premerge = crop;
        }

        m_merge.setInput(*premerge);
    }

    if (m_sql.size())
    {
        // We were created with OGR_DS_ExecuteSQL which needs to have
        // its layer explicitly released
        OGR_DS_ReleaseResultSet(m_dataset, m_layer);
    }
    else
    {
        OGR_DS_Destroy(m_dataset);
    }
    m_layer = 0;
    m_dataset = 0;
}


void TIndexReader::ready(PointTableRef table)
{
    m_merge.prepare(table);
    m_pvSet = m_merge.execute(table);
}


PointViewSet TIndexReader::run(PointViewPtr)
{
    return m_pvSet;
}

} // namespace pdal

