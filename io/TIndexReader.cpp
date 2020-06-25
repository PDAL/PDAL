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

#include <ogr_api.h>

#include <pdal/Polygon.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>
#include <pdal/private/gdal/SpatialRef.hpp>

namespace pdal
{

static StaticPluginInfo const s_info
{
    "readers.tindex",
    "TileIndex Reader",
    "http://pdal.io/stages/readers.tindex.html",
    { "tindex" }
};

CREATE_STATIC_STAGE(TIndexReader, s_info)

std::string TIndexReader::getName() const { return s_info.name; }

TIndexReader::FieldIndexes TIndexReader::getFields()
{
    FieldIndexes indexes;

    void *fDefn = OGR_L_GetLayerDefn(m_layer);

    indexes.m_filename = OGR_FD_GetFieldIndex(fDefn,
        m_tileIndexColumnName.c_str());
    if (indexes.m_filename < 0)
        throwError("Unable to find field '" + m_tileIndexColumnName +
            "' in file '" + m_filename + "'.");
    if (m_srsColumnName.size())
        indexes.m_srs = OGR_FD_GetFieldIndex(fDefn, m_srsColumnName.c_str());

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

        if (m_srsColumnName.size())
        {
            fileInfo.m_srs =
                OGR_F_GetFieldAsString(feature, indexes.m_srs);
        }
        output.push_back(fileInfo);

        OGR_F_Destroy(feature);
    }

    return output;
}


void TIndexReader::addArgs(ProgramArgs& args)
{
    args.add("lyr_name", "OGR layer name from which to read tile index layer",
        m_layerName, "pdal");
    args.add("srs_column", "Column to use to override a file's SRS", m_srsColumnName, "");
    args.add("tindex_name", "OGR column name from which to read tile "
        "index location", m_tileIndexColumnName, "location");
    args.add("sql", "OGR-compatible SQL statement for querying tile "
        "index layer", m_sql);
    args.add("bounds", "Bounds box to limit query window. "
       "Format: '([xmin,xmax],[ymin,ymax])'", m_bounds);
    args.add("polygon", "Well-known text description of bounds to limit query",
        m_wkt);
    args.addSynonym("polygon", "wkt");
    args.add("t_srs", "Transform SRS of tile index geometry", m_tgtSrsString,
        "EPSG:4326");
    args.add("filter_srs", "Transforms any wkt or boundary option to "
        "this coordinate system before filtering or reading data.",
        m_filterSRS, "EPSG:4326");
    args.add("where", "OGR SQL filter clause to use on the layer. It only "
        "works in combination with tile index layers that are defined "
        "with lyr_name", m_attributeFilter);
    args.add("dialect", "OGR SQL dialect to use when querying tile "
        "index layer", m_dialect, "OGRSQL");
}


void TIndexReader::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(pdal::Dimension::Id::X);
    layout->registerDim(pdal::Dimension::Id::Y);
    layout->registerDim(pdal::Dimension::Id::Z);
}


void TIndexReader::initialize()
{
    if (!m_bounds.empty())
        m_wkt = m_bounds.toWKT();
    m_out_ref.reset(new gdal::SpatialRef());

    log()->get(LogLevel::Debug) << "Opening file " << m_filename <<
        std::endl;

    gdal::registerDrivers();
    m_dataset = OGROpen(m_filename.c_str(), FALSE, NULL);
    if (!m_dataset)
        throwError("Unable to datasource '" + m_filename + "'");

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
        throwError("Unable to open layer '" + m_layerName +
            "' from OGR datasource '" + m_filename + "'");

    m_out_ref->setFromLayer(m_layer);

    // Override the SRS if the user set one, otherwise, take it
    // from the layer
    if (m_tgtSrsString.size())
        m_out_ref.reset(new gdal::SpatialRef(m_tgtSrsString));
    else
        m_out_ref.reset(new gdal::SpatialRef(m_out_ref->wkt()));

    // Set SRS if not overridden.
    if (getSpatialReference().empty())
        setSpatialReference(SpatialReference(m_out_ref->wkt()));

    // If the user set either explicit 'polygon' or 'boundary' options
    // we will filter by that geometry. The user can set a 'filter_srs'
    // option to override the SRS of the input geometry and we will
    // reproject to the output projection as needed.
    Polygon poly;
    if (m_wkt.size())
    {
        // Reproject the given wkt to the output SRS so
        // filtering/cropping works
        poly = Polygon(m_wkt, m_filterSRS);
        poly.transform(m_out_ref->wkt());

        m_wkt = poly.wkt();
        OGR_L_SetSpatialFilter(m_layer, poly.getOGRHandle());
    }

    if (m_attributeFilter.size())
    {
        OGRErr err = OGR_L_SetAttributeFilter(m_layer,
            m_attributeFilter.c_str());
        if (err != OGRERR_NONE)
            throwError("Unable to set attribute filter '" + m_attributeFilter +
                "' for OGR datasource '" + m_filename + "'");
    }

    Options cropOptions;
    if (m_wkt.size())
        cropOptions.add("polygon", m_wkt);

    for (auto f : getFiles())
    {
        log()->get(LogLevel::Debug) << "Adding file " << f.m_filename <<
            " to merge filter" << std::endl;

        std::string driver = m_factory.inferReaderDriver(f.m_filename);
        Stage *reader = m_factory.createStage(driver);
        if (!reader)
            throwError("Unable to create reader for file '" + f.m_filename +
                "'.");
        Options readerOptions;
        readerOptions.add("filename", f.m_filename);
        reader->setOptions(readerOptions);
        Stage *premerge = reader;

        if (m_tgtSrsString.size() )
        {
            Stage *repro = m_factory.createStage("filters.reprojection");
            repro->setInput(*reader);
            Options reproOptions;
            reproOptions.add("out_srs", m_tgtSrsString);
            if (m_srsColumnName.size())
            {
                reproOptions.add("in_srs", f.m_srs);
                log()->get(LogLevel::Debug2) << "Repro = "
                                             << m_tgtSrsString << "/"
                                             << f.m_srs << "!\n";
            }
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

void TIndexReader::prepared(PointTableRef table)
{
    m_merge.prepare(table);
}

void TIndexReader::ready(PointTableRef table)
{
    m_pvSet = m_merge.execute(table);
}


PointViewSet TIndexReader::run(PointViewPtr)
{
    return m_pvSet;
}

} // namespace pdal

