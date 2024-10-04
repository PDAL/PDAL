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

#include "TIndexKernel.hpp"

#include <memory>
#include <vector>

#include <ogr_api.h>

#include <pdal/PDALUtils.hpp>
#include <pdal/Polygon.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>
#include <pdal/private/gdal/SpatialRef.hpp>

#include "../io/LasWriter.hpp"

#include <cpl_string.h>

namespace
{

void setDate(OGRFeatureH feature, const tm& tyme, int fieldNumber)
{
    OGR_F_SetFieldDateTime(feature, fieldNumber,
        tyme.tm_year + 1900, tyme.tm_mon + 1, tyme.tm_mday, tyme.tm_hour,
        tyme.tm_min, tyme.tm_sec, 100);
}


} // anonymous namespace


namespace pdal
{

static StaticPluginInfo const s_info
{
    "kernels.tindex",
    "TIndex Kernel",
    "http://pdal.io/apps/tindex.html"
};

CREATE_STATIC_KERNEL(TIndexKernel, s_info)

std::string TIndexKernel::getName() const { return s_info.name; }

TIndexKernel::TIndexKernel() : SubcommandKernel()
//ABELL - need to option this.
    , m_srsColumnName("srs")
    , m_dataset(NULL)
    , m_layer(NULL)
    , m_overrideASrs(false)
{}


StringList TIndexKernel::subcommands() const
{
    return { "create", "merge" };
}


void TIndexKernel::addSubSwitches(ProgramArgs& args,
    const std::string& subcommand)
{
    if (subcommand == "create")
    {
        args.add("tindex", "OGR-readable/writeable tile index output",
            m_idxFilename).setPositional();
        args.add("filespec", "Pattern of files to index",
            m_filespec).setOptionalPositional();
        args.add("fast_boundary", "Use extent instead of exact boundary",
            m_fastBoundary);
        args.add("lyr_name", "OGR layer name to write into datasource",
            m_layerName);
        args.add("tindex_name", "Tile index column name", m_tileIndexColumnName,
            "location");
        args.add("ogrdriver,f", "OGR driver name to use ", m_driverName,
            "ESRI Shapefile");
        args.add("t_srs", "Target SRS of tile index", m_tgtSrsString,
            "EPSG:4326");
        args.add("a_srs", "Assign SRS of tile with no SRS to this value",
            m_assignSrsString, "EPSG:4326");
        args.add("write_absolute_path",
            "Write absolute rather than relative file paths", m_absPath);
        args.add("stdin,s", "Read filespec pattern from standard input",
            m_usestdin);
    }
    else if (subcommand == "merge")
    {
        args.add("tindex", "OGR-readable/writeable tile index output",
            m_idxFilename).setPositional();
        args.add("filespec", "Output filename",
            m_filespec).setPositional();
        args.add("lyr_name", "OGR layer name to write into datasource",
            m_layerName);
        args.add("tindex_name", "Tile index column name", m_tileIndexColumnName,
            "location");
        args.add("ogrdriver,f", "OGR driver name to use ", m_driverName,
            "ESRI Shapefile");
        args.add("bounds", "Extent (in XYZ) to clip output to", m_bounds);
        args.add("polygon", "Well-known text of polygon to clip output", m_wkt);
        args.add("t_srs", "Spatial reference of the clipping geometry",
            m_tgtSrsString, "EPSG:4326");
    }
}


void TIndexKernel::validateSwitches(ProgramArgs& args)
{
    if (m_subcommand == "merge")
    {
        if (!m_wkt.empty() && !m_bounds.empty())
            throw pdal_error("Can't specify both 'polygon' and "
                "'bounds' options.");
        if (!m_bounds.empty())
            m_wkt = m_bounds.toWKT();
    }
    else
    {
        if (m_filespec.empty() && !m_usestdin)
            throw pdal_error("Must specify either --filespec or --stdin.");
        if (m_filespec.size() && m_usestdin)
            throw pdal_error("Can't specify both --filespec and --stdin "
                "options.");
        if (args.set("a_srs"))
            m_overrideASrs = true;
    }
}


int TIndexKernel::execute()
{
    gdal::registerDrivers();

    if (m_subcommand == "merge")
        mergeFile();
    else
    {
        try
        {
            createFile();
        }
        catch (pdal_error&)
        {
            if (m_dataset)
                OGR_DS_Destroy(m_dataset);
            throw;
        }
    }
    return 0;
}


StringList readSTDIN()
{
    std::string line;
    StringList output;
    while (std::getline(std::cin, line))
    {
        output.push_back(line);
    }
    return output;
}


bool TIndexKernel::isFileIndexed(const FieldIndexes& indexes,
    const FileInfo& fileInfo)
{
    std::ostringstream qstring;

    qstring << Utils::toupper(m_tileIndexColumnName) << "=" <<
        "'" << fileInfo.m_filename << "'";
    std::string query = qstring.str();
    OGRErr err = OGR_L_SetAttributeFilter(m_layer, query.c_str());
    if (err != OGRERR_NONE)
    {
        std::ostringstream oss;
        oss << "Unable to set attribute filter for file '" <<
             fileInfo.m_filename << "'";
        throw pdal_error(oss.str());
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


void TIndexKernel::createFile()
{
    if (!m_usestdin)
        m_files = FileUtils::glob(m_filespec);
    else
        m_files = readSTDIN();

    if (m_absPath)
        for (auto& s : m_files)
            s = FileUtils::toAbsolutePath(s);

    if (m_files.empty())
    {
        std::ostringstream out;
        out << "Couldn't find files to index: " << m_filespec << ".";
        throw pdal_error(out.str());
    }

//ABELL - Remove CPLGetBasename use.
    const std::string filename = m_files.front();
    if (m_layerName.empty())
       m_layerName = CPLGetBasename(filename.c_str());

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
        if (!createLayer(m_layerName))
        {
            std::ostringstream out;
            out << "Couldn't open or create layer '" << m_layerName <<
                "' in output file '" << m_idxFilename << "'.";
            throw pdal_error(out.str());
        }

    FieldIndexes indexes = getFields();

    size_t filecount(0);
    StageFactory factory(false);
    for (auto f : m_files)
    {
        const std::chrono::time_point<std::chrono::steady_clock> start =
            std::chrono::steady_clock::now();
        //ABELL - Not sure why we need to get absolute path here.
        f = FileUtils::toAbsolutePath(f);
        FileInfo info;
        if (getFileInfo(factory, f, info))
        {
            filecount++;
            if (!isFileIndexed(indexes, info))
            {
                if (createFeature(indexes, info))
                    m_log->get(LogLevel::Info) << "Indexed file " << f <<
                    std::endl;
                else
                    m_log->get(LogLevel::Error) << "Failed to create feature "
                        "for file '" << f << "'" << std::endl;
            }
        }
        const std::chrono::time_point<std::chrono::steady_clock> end =
            std::chrono::steady_clock::now();
        std::cout << "file processing took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << " milliseconds\n";
    }
    if (!filecount)
        throw pdal_error("Couldn't index any files.");
    OGR_DS_Destroy(m_dataset);
    m_dataset = nullptr;
    m_layer = nullptr;
}


void TIndexKernel::mergeFile()
{
    using namespace gdal;

    std::ostringstream out;

    if (!openDataset(m_idxFilename))
    {
        std::ostringstream out;
        out << "Couldn't open index dataset file '" << m_idxFilename << "'.";
        throw pdal_error(out.str());
    }
    if (!openLayer(m_layerName))
    {
        std::ostringstream out;
        out << "Couldn't open layer '" << m_layerName <<
            "' in output file '" << m_idxFilename << "'.";
        throw pdal_error(out.str());
    }

    FieldIndexes indexes = getFields();

    if (!m_wkt.empty())
    {
        pdal::Polygon g(m_wkt, m_tgtSrsString);
        OGR_L_SetSpatialFilter(m_layer, g.getOGRHandle());
    }

    std::vector<FileInfo> files;

    // Docs are bad here.  You need this call even if you haven't read anything
    // or nothing happens.
    OGR_L_ResetReading(m_layer);
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
        files.push_back(fileInfo);

        OGR_F_Destroy(feature);
    }

    OGR_DS_Destroy(m_dataset);
    m_dataset = nullptr;
    m_layer = nullptr;

    m_log->get(LogLevel::Info) << "Merge filecount: " <<
        files.size() << std::endl;

    Options cropOptions;
    if (!m_bounds.empty())
        cropOptions.add("bounds", m_bounds);
    else
        cropOptions.add("polygon", m_wkt);

    Stage& merge = makeFilter("filters.merge");
    size_t filecount(0);
    for (auto f : files)
    {
        Stage& reader = makeReader(f.m_filename, m_driverOverride);
        Stage *premerge = &reader;

        if (m_tgtSrsString != f.m_srs)
        {
            Options reproOptions;
            reproOptions.add("out_srs", m_tgtSrsString);
            reproOptions.add("in_srs", f.m_srs);
            Stage& repro = makeFilter("filters.reprojection", reader,
                reproOptions);
            premerge = &repro;
        }

        // WKT is set, even if we're using a bounding box for fitering, so
        // can be used as a test here.
        if (!m_wkt.empty())
        {
            Stage& crop = makeFilter("filters.crop", *premerge, cropOptions);
            premerge = &crop;
        }
        merge.setInput(*premerge);
    }

    Stage& writer = makeWriter(m_filespec, merge, "");
    try
    {
        (void)dynamic_cast<LasWriter &>(writer);
        Options options;
        options.add("offset_x", "auto");
        options.add("offset_y", "auto");
        options.add("offset_z", "auto");
        writer.addOptions(options);
    }
    catch (std::bad_cast&)
    {}

    ColumnPointTable table;
    writer.prepare(table);
    writer.execute(table);
}


bool TIndexKernel::createFeature(const FieldIndexes& indexes,
    FileInfo& fileInfo)
{
    using namespace gdal;

    OGRFeatureH hFeature = OGR_F_Create(OGR_L_GetLayerDefn(m_layer));

    // Set the creation time into the feature.
    setDate(hFeature, fileInfo.m_ctime, indexes.m_ctime);

    // Set the file mod time into the feature.
    setDate(hFeature, fileInfo.m_mtime, indexes.m_mtime);

    // Set the filename into the feature.
    OGR_F_SetFieldString(hFeature, indexes.m_filename,
        fileInfo.m_filename.c_str());

    // Set the SRS into the feature.
    // We override if m_assignSrsString is set
    if (fileInfo.m_srs.empty() || m_overrideASrs)
        fileInfo.m_srs = m_assignSrsString;

    if (fileInfo.m_srs.empty())
    {
        std::ostringstream oss;

        oss << "Unable to import source spatial reference '" <<
            fileInfo.m_srs << "' for file '" <<
            fileInfo.m_filename << "'.";
        OGR_F_Destroy(hFeature);
        throw pdal_error(oss.str());
    }

    std::string wkt =
        SpatialReference(fileInfo.m_srs).getWKT();
    OGR_F_SetFieldString(hFeature, indexes.m_srs, wkt.data());

    // Set the geometry in the feature
    Polygon g = prepareGeometry(fileInfo);
    OGR_F_SetGeometry(hFeature, g.getOGRHandle());

    const bool bRet = (OGR_L_CreateFeature(m_layer, hFeature) == OGRERR_NONE);
    OGR_F_Destroy(hFeature);
    return bRet;
}


bool TIndexKernel::fastBoundary(Stage& reader, FileInfo& fileInfo)
{
    QuickInfo qi = reader.preview();
    if (!qi.valid())
        return false;

    fileInfo.m_boundary = qi.m_bounds.to2d().toWKT();
    if (!qi.m_srs.empty())
        fileInfo.m_srs = qi.m_srs.getWKT();
    return true;
}


bool TIndexKernel::slowBoundary(PipelineManager& manager, FileInfo& fileInfo)
{
    std::cout << "running slow bounds\n";
    manager.prepare();
    manager.execute(ExecMode::Stream);
    PointViewSet set = manager.views();
    MetadataNode root = manager.getMetadata();

    // If we had an error set, bail out
    MetadataNode e = root.findChild("filters.hexbin:error");
    if (e.valid())
        return false;

    MetadataNode m = root.findChild("filters.hexbin:boundary");
    fileInfo.m_boundary = m.value();

    PointViewPtr v = *set.begin();
    if (!v->spatialReference().empty())
        fileInfo.m_srs = v->spatialReference().getWKT();
    return true;
}


bool TIndexKernel::getFileInfo(StageFactory& factory,
    const std::string& filename, FileInfo& fileInfo)
{
    PipelineManager manager;
    manager.commonOptions() = m_manager.commonOptions();
    manager.stageOptions() = m_manager.stageOptions();

    // Need to make sure options get set.
    Stage& reader = manager.makeReader(filename, "");

    // If we aren't able to make a hexbin filter, we
    // will just do a simple fast_boundary.
    bool fast(m_fastBoundary);
    try
    {
        if (!fast)
        {
            Stage& hexer = manager.makeFilter("filters.hexbin", reader);
            fast = !slowBoundary(manager, fileInfo);
        }
    }
    catch (pdal_error&)
    {
        fast = true;
    }
    if (fast && !fastBoundary(reader, fileInfo))
    {
        m_log->get(LogLevel::Error) << "Skipping file '" << filename <<
            "': can't compute boundary." << std::endl;
        return false;
    }
    FileUtils::fileTimes(filename, &fileInfo.m_ctime, &fileInfo.m_mtime);
    fileInfo.m_filename = filename;

    return true;
}


bool TIndexKernel::openDataset(const std::string& filename)
{
    m_dataset = OGROpen(filename.c_str(), TRUE, NULL);
    return (bool)m_dataset;
}


bool TIndexKernel::createDataset(const std::string& filename)
{
    OGRSFDriverH hDriver = OGRGetDriverByName(m_driverName.c_str());
    if (!hDriver)
    {
        std::ostringstream oss;

        oss << "Can't create dataset using driver '" << m_driverName <<
            "'. Driver is not available.";
        throw pdal_error(oss.str());
    }

    std::string dsname = FileUtils::toAbsolutePath(filename);
    m_dataset = OGR_Dr_CreateDataSource(hDriver, dsname.c_str(), NULL);
    return (bool)m_dataset;
}


bool TIndexKernel::openLayer(const std::string& layerName)
{
    if (OGR_DS_GetLayerCount(m_dataset) == 1)
        m_layer = OGR_DS_GetLayer(m_dataset, 0);
    else if (layerName.size())
        m_layer = OGR_DS_GetLayerByName(m_dataset, m_layerName.c_str());

    return (bool)m_layer;
}


bool TIndexKernel::createLayer(std::string const& layername)
{
    gdal::SpatialRef srs(m_tgtSrsString);
    if (!srs)
        m_log->get(LogLevel::Error) << "Unable to import srs for layer "
           "creation" << std::endl;

    m_layer = OGR_DS_CreateLayer(m_dataset, m_layerName.c_str(),
        srs.get(), wkbPolygon, NULL);

    if (m_layer)
        createFields();

    //ABELL - At this point we should essentially "sync" things so that
    //  index file gets created with the proper fields.  If this doesn't
    //  and a failure occurs, the file may be left with a layer that doesn't
    //  have the requisite fields.  Note that OGR_DS_SyncToDisk doesn't seem
    //  to work reliably enough to warrant use.
    return (bool)m_layer;
}


void TIndexKernel::createFields()
{
    OGRFieldDefnH hFieldDefn = OGR_Fld_Create(
        m_tileIndexColumnName.c_str(), OFTString);
    OGR_Fld_SetWidth(hFieldDefn, 254);
    OGR_L_CreateField(m_layer, hFieldDefn, TRUE);
    OGR_Fld_Destroy(hFieldDefn);

    hFieldDefn = OGR_Fld_Create(m_srsColumnName.c_str(), OFTString);
    OGR_L_CreateField(m_layer, hFieldDefn, TRUE );
    OGR_Fld_Destroy(hFieldDefn);

    hFieldDefn = OGR_Fld_Create("modified", OFTDateTime);
    OGR_L_CreateField(m_layer, hFieldDefn, TRUE);
    OGR_Fld_Destroy(hFieldDefn);

    hFieldDefn = OGR_Fld_Create("created", OFTDateTime);
    OGR_L_CreateField(m_layer, hFieldDefn, TRUE);
    OGR_Fld_Destroy(hFieldDefn);
}


TIndexKernel::FieldIndexes TIndexKernel::getFields()
{
    FieldIndexes indexes;

    void *fDefn = OGR_L_GetLayerDefn(m_layer);

    indexes.m_filename = OGR_FD_GetFieldIndex(fDefn,
        m_tileIndexColumnName.c_str());
    if (indexes.m_filename < 0)
    {
        std::ostringstream out;

        out << "Unable to find field '" << m_tileIndexColumnName <<
            "' in file '" << m_idxFilename << "'.";
        throw pdal_error(out.str());
    }
    indexes.m_srs = OGR_FD_GetFieldIndex(fDefn, m_srsColumnName.c_str());
    if (indexes.m_srs < 0)
    {
        std::ostringstream out;

        out << "Unable to find field '" << m_srsColumnName << "' in file '" <<
            m_idxFilename << "'.";
        throw pdal_error(out.str());
    }

    indexes.m_ctime = OGR_FD_GetFieldIndex(fDefn, "created");
    indexes.m_mtime = OGR_FD_GetFieldIndex(fDefn, "modified");

    return indexes;
}


pdal::Polygon TIndexKernel::prepareGeometry(const FileInfo& fileInfo)
{
    using namespace gdal;

    Polygon g(fileInfo.m_boundary, fileInfo.m_srs);
    if (m_tgtSrsString.size())
    {
        SpatialReference out(m_tgtSrsString);
        g.transform(out);
    }
    return g;
}

} // namespace pdal
