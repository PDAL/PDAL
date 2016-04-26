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

#ifndef WIN32
#include <glob.h>
#include <time.h>
#endif

#include <memory>
#include <vector>

#include <pdal/KernelFactory.hpp>
#include <pdal/util/FileUtils.hpp>
#include <merge/MergeFilter.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/pdal_macros.hpp>

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

static PluginInfo const s_info = PluginInfo(
    "kernels.tindex",
    "TIndex Kernel",
    "http://pdal.io/kernels/kernels.tindex.html" );

CREATE_STATIC_PLUGIN(1, 0, TIndexKernel, Kernel, s_info)

std::string TIndexKernel::getName() const { return s_info.name; }

TIndexKernel::TIndexKernel()
    : Kernel()
//ABELL - need to option this.
    , m_srsColumnName("srs")
    , m_merge(false)
    , m_dataset(NULL)
    , m_layer(NULL)
    , m_fastBoundary(false)

{
    m_log.setLeader("pdal tindex");
}


void TIndexKernel::addSwitches(ProgramArgs& args)
{
    args.add("tindex", "OGR-readable/writeable tile index output",
        m_idxFilename).setPositional();
    args.add("filespec", "Build: Pattern of files to index. "
        "Merge: Output filename", m_filespec).setPositional();
    args.add("fast_boundary", "Use extent instead of exact boundary",
        m_fastBoundary);
    args.add("lyr_name", "OGR layer name to write into datasource",
        m_layerName);
    args.add("tindex_name", "Tile index column name", m_tileIndexColumnName,
        "location");
    args.add("driver,f", "OGR driver name to use ", m_driverName,
        "ESRI Shapefile");
    args.add("t_srs", "Target SRS of tile index", m_tgtSrsString,
        "EPSG:4326");
    args.add("a_srs", "Assign SRS of tile with no SRS to this value",
        m_assignSrsString, "EPSG:4326");
    args.add("bounds", "Extent (in XYZ) to clip output to", m_bounds);
    args.add("polygon", "Well-known text of polygon to clip output", m_wkt);
    args.add("write_absolute_path",
        "Write absolute rather than relative file paths", m_absPath);
    args.add("merge", "Whether we're merging the entries in a tindex file.",
        m_merge);
}


void TIndexKernel::validateSwitches(ProgramArgs& args)
{
    if (m_merge)
    {
        if (!m_wkt.empty() && !m_bounds.empty())
            throw pdal_error("Can't specify both 'polygon' and "
                "'bounds' options.");
        if (!m_bounds.empty())
            m_wkt = m_bounds.toWKT();
        if (m_filespec.empty())
            throw pdal_error("No output filename provided.");
        StringList invalidArgs;
        invalidArgs.push_back("a_srs");
        invalidArgs.push_back("src_srs_name");
        for (auto arg : invalidArgs)
            if (args.set(arg))
            {
                std::ostringstream out;

                out << "option '" << arg << "' not supported during merge.";
                throw pdal_error(out.str());
            }
    }
    else
    {
        if (m_filespec.empty() && !m_usestdin)
            throw pdal_error("No input pattern specified");
        if (args.set("polygon"))
            throw pdal_error("'polygon' option not supported when building "
                "index.");
        if (args.set("bounds"))
            throw pdal_error("'bounds' option not supported when building "
                "index.");
    }
}


int TIndexKernel::execute()
{
    gdal::registerDrivers();

    if (m_merge)
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


StringList TIndexKernel::glob(std::string& path)
{
    StringList filenames;

#ifndef WIN32
    glob_t glob_result;

    ::glob(path.c_str(), GLOB_TILDE, NULL, &glob_result);
    for (unsigned int i = 0; i < glob_result.gl_pathc; ++i)
    {
        std::string filename = glob_result.gl_pathv[i];
        if (m_absPath)
            filename = FileUtils::toAbsolutePath(filename);
        filenames.push_back(filename);
    }
    globfree(&glob_result);
#endif

    return filenames;
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
    if (OGR_L_GetNextFeature(m_layer))
        output = true;
    OGR_L_ResetReading(m_layer);
    OGR_L_SetAttributeFilter(m_layer, NULL);
    return output;
}


void TIndexKernel::createFile()
{
    if (!m_usestdin)
        m_files = glob(m_filespec);
    else
        m_files = readSTDIN();

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

    KernelFactory factory(false);
    for (auto f : m_files)
    {
        //ABELL - Not sure why we need to get absolute path here.
        f = FileUtils::toAbsolutePath(f);
        FileInfo info = getFileInfo(factory, f);
        if (!isFileIndexed(indexes, info))
        {
            if (createFeature(indexes, info))
                m_log.get(LogLevel::Info) << "Indexed file " << f << std::endl;
            else
                m_log.get(LogLevel::Error) << "Failed to create feature for "
                    "file '" << f << "'" << std::endl;

        }
    }
    OGR_DS_Destroy(m_dataset);
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

    SpatialRef outSrs(m_tgtSrsString);
    if (!outSrs)
        throw pdal_error("Couldn't interpret target SRS string.");

    if (!m_wkt.empty())
    {
        Geometry g(m_wkt, outSrs);

        if (!g)
            throw pdal_error("Couldn't interpret geometry filter string.");
        OGR_L_SetSpatialFilter(m_layer, g.get());
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

    Options cropOptions;
    if (!m_bounds.empty())
        cropOptions.add("bounds", m_bounds);
    else
        cropOptions.add("polygon", m_wkt);

    Stage& merge = makeFilter("filters.merge");
    for (auto f : files)
    {
        Stage& reader = makeReader(f.m_filename, "");
        Stage *premerge = &reader;

        if (m_tgtSrsString != f.m_srs)
        {
            Stage& repro = makeFilter("filters.reprojection", reader);
            Options reproOptions;
            reproOptions.add("out_srs", m_tgtSrsString);
            reproOptions.add("in_srs", f.m_srs);
            repro.addOptions(reproOptions);
            premerge = &repro;
        }

        // WKT is set, even if we're using a bounding box for fitering, so
        // can be used as a test here.
        if (!m_wkt.empty())
        {
            Stage& crop = makeFilter("filters.crop", *premerge);
            crop.addOptions(cropOptions);
            premerge = &crop;
        }
        merge.setInput(*premerge);
    }

    Stage& writer = makeWriter(m_filespec, merge, "");

    Options writerOptions;
    writerOptions.add("offset_x", "auto");
    writerOptions.add("offset_y", "auto");
    writerOptions.add("offset_z", "auto");
    writer.addConditionalOptions(writerOptions);

    PointTable table;
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
    if (fileInfo.m_srs.empty() || m_assignSrsString.size())
        fileInfo.m_srs = m_assignSrsString;

    SpatialRef srcSrs(fileInfo.m_srs);
    if (srcSrs.empty())
    {
        std::ostringstream oss;

        oss << "Unable to import source spatial reference '" <<
            fileInfo.m_srs << "' for file '" <<
            fileInfo.m_filename << "'.";
        throw pdal_error(oss.str());
    }

    // We have a limit of like 254 characters in some formats (notably
    // shapefile), so try to get the condensed version of the SRS.

    // Failing that, get the proj.4 version.  Not sure what's supposed to
    // happen if we overflow 254 with proj.4.

    const char* pszAuthorityCode = OSRGetAuthorityCode(srcSrs.get(), NULL);
    const char* pszAuthorityName = OSRGetAuthorityName(srcSrs.get(), NULL);
    if (pszAuthorityName && pszAuthorityCode)
    {
        std::string auth = std::string(pszAuthorityName) + ":" +
            pszAuthorityCode;
        OGR_F_SetFieldString(hFeature, indexes.m_srs, auth.data());
    }
    else
    {
        char* pszProj4 = NULL;
        int err = -1;
        try
        {
            err = OSRExportToProj4(srcSrs.get(), &pszProj4);
        }
        catch (pdal_error)
        {}
        if (err != OGRERR_NONE)
        {
            m_log.get(LogLevel::Warning) << "Unable to convert SRS to "
                "proj.4 format for file '" << fileInfo.m_filename << "'" <<
                std::endl;
            return false;
        }
        std::string srs = std::string(pszProj4);
        OGR_F_SetFieldString(hFeature, indexes.m_srs, srs.c_str());
        CPLFree(pszProj4);
    }

    // Set the geometry in the feature
    Geometry g = prepareGeometry(fileInfo);
    char *pgeom;
    OGR_G_ExportToWkt(g.get(), &pgeom);
    OGR_F_SetGeometry(hFeature, g.get());

    bool ok = (OGR_L_CreateFeature(m_layer, hFeature) == OGRERR_NONE);
    OGR_F_Destroy(hFeature);
    return ok;
}


TIndexKernel::FileInfo TIndexKernel::getFileInfo(KernelFactory& factory,
    const std::string& filename)
{
    FileInfo fileInfo;

    PipelineManager manager;
    manager.commonOptions() = m_manager.commonOptions();
    manager.stageOptions() = m_manager.stageOptions();

    // Need to make sure options get set.
    Stage& reader = manager.makeReader(filename, "");

    if (m_fastBoundary)
    {
        QuickInfo qi = reader.preview();

        std::stringstream polygon;
        polygon << "POLYGON ((";

        polygon <<         qi.m_bounds.minx << " " << qi.m_bounds.miny;
        polygon << ", " << qi.m_bounds.maxx << " " << qi.m_bounds.miny;
        polygon << ", " << qi.m_bounds.maxx << " " << qi.m_bounds.maxy;
        polygon << ", " << qi.m_bounds.minx << " " << qi.m_bounds.maxy;
        polygon << ", " << qi.m_bounds.minx << " " << qi.m_bounds.miny;
        polygon << "))";
        fileInfo.m_boundary = polygon.str();
        if (!qi.m_srs.empty())
            fileInfo.m_srs = qi.m_srs.getWKT();
    }
    else
    {
        Stage& hexer = manager.makeFilter("filters.hexbin", reader);

        PointTable table;
        hexer.prepare(table);
        PointViewSet set = hexer.execute(table);

        MetadataNode m = table.metadata();
        m = m.findChild("filters.hexbin:boundary");
        fileInfo.m_boundary = m.value();

        PointViewPtr v = *set.begin();
        if (!v->spatialReference().empty())
            fileInfo.m_srs = v->spatialReference().getWKT();
    }

    FileUtils::fileTimes(filename, &fileInfo.m_ctime, &fileInfo.m_mtime);
    fileInfo.m_filename = filename;

    return fileInfo;
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
    using namespace gdal;

    SpatialRef srs(m_tgtSrsString);
    if (!srs)
        m_log.get(LogLevel::Error) << "Unable to import srs for layer "
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
    OGR_Fld_SetWidth(hFieldDefn, 254);
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

//     /* Load in memory existing file names in SHP */
//     int nExistingFiles = (int)OGR_L_GetFeatureCount(m_layer, FALSE);
//     for (auto i = 0; i < nExistingFiles; i++)
//     {
//         OGRFeatureH hFeature = OGR_L_GetNextFeature(m_layer);
//         m_files.push_back(OGR_F_GetFieldAsString(hFeature, indexes.m_filename));
//         OGR_F_Destroy(hFeature);
//     }
    return indexes;
}


gdal::Geometry TIndexKernel::prepareGeometry(const FileInfo& fileInfo)
{
    using namespace gdal;

    std::ostringstream oss;


    SpatialRef srcSrs(fileInfo.m_srs);
    SpatialRef tgtSrs(m_tgtSrsString);
    if (!tgtSrs)
        throw pdal_error("Unable to import target SRS.");

    Geometry g;
    try
    {
       g = prepareGeometry(fileInfo.m_boundary, srcSrs, tgtSrs);
    }
    catch (pdal_error& e)
    {
        oss << "Unable to transform geometry from source to target SRS for " <<
            fileInfo.m_filename << "'. Message is '" << e.what() << "'";
        throw pdal_error(oss.str());
    }
    if (!g)
    {
        oss << "Update to create geometry from WKT for '" <<
            fileInfo.m_filename << "'.";
        throw pdal_error(oss.str());
    }
    return g;
}


gdal::Geometry TIndexKernel::prepareGeometry(const std::string& wkt,
   const gdal::SpatialRef& inSrs, const gdal::SpatialRef& outSrs)
{
    // Create OGR geometry from text.

    gdal::Geometry g(wkt, inSrs);

    if (g)
        if (OGR_G_TransformTo(g.get(), outSrs.get()) != OGRERR_NONE)
            throw pdal_error("Unable to transform geometry.");

    return g;
}

} // namespace pdal

