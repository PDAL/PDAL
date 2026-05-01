#include "TIndexBuilder.hpp"
#include "TIndexBoundary.hpp"

#include <pdal/util/ThreadPool.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>
#include <pdal/private/gdal/SpatialRef.hpp>

#include <ogr_api.h>
#include <cpl_string.h>

// just copied from TIndexKernel
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
    operator int() const { return m_fieldIdx; }

    OGRFieldType m_fieldType;
    OGRFieldSubType m_subtype = OFSTNone;
    int m_fieldIdx = -1;
};

//
// Base class
//

TIndexBuilder::TIndexBuilder(const Args& args, const std::string& tileIndexColumnName,
        const std::string& srsColumnName, const std::string& driverName, const std::string& tgtSrs,
        const std::string& assignSrs)
    : m_args(std::move(args)), m_tileIndexColumnName(tileIndexColumnName), m_srsColumnName(srsColumnName), 
      m_driverName(driverName), m_tgtSrsString(tgtSrs), m_assignSrsString(assignSrs), m_layerName(m_args.layerName),
      m_maxFieldSize(0)
{
    m_fields.emplace(m_tileIndexColumnName, OFTString);
    m_fields.emplace(m_srsColumnName, OFTString);
}

TIndexBuilder::~TIndexBuilder() 
{
    if (m_dataset)
        OGR_DS_Destroy(m_dataset);
}

bool TIndexBuilder::openDataset()
{
    m_dataset = OGROpen(m_args.idxFilename.c_str(), TRUE, NULL);
    return (bool)m_dataset;
}

bool TIndexBuilder::createDataset()
{
    OGRSFDriverH hDriver = OGRGetDriverByName(m_args.driverName.c_str());
    if (!hDriver)
    {
        std::ostringstream oss;

        oss << "Can't create dataset using driver '" << m_args.driverName <<
            "'. Driver is not available.";
        throw TIndexError(oss.str());
    }

    m_dataset = OGR_Dr_CreateDataSource(hDriver, m_args.filename.c_str(), NULL);
    return (bool)m_dataset;
}


bool TIndexBuilder::openLayer()
{
    if (OGR_DS_GetLayerCount(m_dataset) == 1)
        m_layer = OGR_DS_GetLayer(m_dataset, 0);
    else if (layerName.size())
        m_layer = OGR_DS_GetLayerByName(m_dataset, m_args.layerName.c_str());

    return (bool)m_layer;
}


bool TIndexBuilder::createLayer()
{
    gdal::SpatialRef srs(m_tgtSrsString);
    if (!srs)
        m_log->get(LogLevel::Error) << "Unable to import srs for layer "
           "creation" << std::endl;

    char** papszOptions = NULL;
    for (const std::string& s : m_args.lcOptions)
        papszOptions = CSLAddString(papszOptions, s.c_str());

    m_layer = OGR_DS_CreateLayer(m_dataset, m_args.layerName.c_str(),
        srs.get(), wkbMultiPolygon, papszOptions);

    CSLDestroy(papszOptions);

    if (m_layer)
        createFields();

    //ABELL - At this point we should essentially "sync" things so that
    //  index file gets created with the proper fields.  If this doesn't
    //  and a failure occurs, the file may be left with a layer that doesn't
    //  have the requisite fields.  Note that OGR_DS_SyncToDisk doesn't seem
    //  to work reliably enough to warrant use.
    return (bool)m_layer;
}

void TIndexBuilder::getFieldIndexes(const OGRFeatureDefnH layerDefn)
{
    for (auto& [name, info]: m_fields)
    {
        info.m_fieldIdx =  OGR_FD_GetFieldIndex(layerDefn, name.c_str());
        //!! expand this to all fields?
        if ((name == m_tileIndexColumnName || name == m_srsColumnName)
            && info.m_fieldIdx < 0)
        {
            std::ostringstream out;
            out << "Unable to find field '" << name << "' in file '" 
                << m_args.idxFilename << "'.";
            throw TIndexError(out.str());
        }
    }
}

void TIndexBuilder::create(const StringList& files, PipelineManager& mgr) 
{
    const std::string filename = files.front();
    m_infos.reserve(files.size());

    //!! could just pass these in directly
    m_commonOptions = mgr.commonOptions();
    m_stageOptions = mgr.stageOptions();
    //!! Remove log use if possible
    m_log = mgr.log();

    if (m_layerName.empty())
       m_layerName = CPLGetBasename(filename.c_str());

    // Open or create the dataset.
    if (!openDataset())
        if (!createDataset())
        {
            std::ostringstream out;
            out << "Couldn't open or create index dataset file '" <<
                m_args.idxFilename << "'.";
            throw TIndexError(out.str());
        }

    // Open or create a layer
    if (!openLayer())
        if (!createLayer())
        {
            std::ostringstream out;
            out << "Couldn't open or create layer '" << m_layerName <<
                "' in output file '" << m_args.idxFilename << "'.";
            throw TIndexError(out.str());
        }
    for (auto& file : files)
    {
        // Sets filename, initializes STAC metadata
        auto& info = makeFileInfo(file);
        info->m_isRemote = Utils::isRemote(file);
        if (!info.m_isRemote)
            FileUtils::fileTimes(info->m_filename, &info->m_ctime, &info->m_mtime);
        m_infos.push_back(std::move(info));
    }
    ThreadPool pool(m_args.numThreads);

    for (auto &info : m_infos)
    {
        pool.add([this, info = std::move(info)]()
        {
            getFileInfo(info);
        });
    }
    pool.await();

    m_originalSrs = infos[0].m_srs;
    if (m_originalSrs.empty() || m_args.overrideASrs)
        m_originalSrs = m_assignSrsString;
    bool indexedFile(false);
    for (auto& info : m_infos)
    {
        if (m_args.pathPrefix.size() && !info->m_isRemote)
            info->m_filename = m_args.pathPrefix + FileUtils::getFilename(info->m_filename);
        else if (m_args.absPath && !info->m_isRemote)
            info->m_filename = FileUtils::toAbsolutePath(info->m_filename);
        if (!info->m_boundary.empty() && !isFileIndexed(info))
            indexedFile |= createFeature(info);
    }
    if (!indexedFile)
        throw TIndexError("Couldn't index any files.");
    OGR_DS_Destroy(m_dataset);
    m_dataset = nullptr;
    m_layer = nullptr;
}

bool TIndexBuilder::isFileIndexed(const std::unique_ptr<FileInfo>& fileInfo)
{
    std::ostringstream qstring;

    qstring << "\"" <<  Utils::toupper(m_tileIndexColumnName) << "\"=" <<
        "'" << fileInfo->m_filename << "'";
    std::string query = qstring.str();
    OGRErr err = OGR_L_SetAttributeFilter(m_layer, query.c_str());
    if (err != OGRERR_NONE)
    {
        std::ostringstream oss;
        oss << "Unable to set attribute filter for file '" <<
             fileInfo->m_filename << "'";
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

bool TIndexBuilder::createFeature(const std::unique_ptr<FileInfo>& fileInfo)
{
    using namespace gdal;

    OGRFeatureH hFeature = OGR_F_Create(OGR_L_GetLayerDefn(m_layer));

    setStringField(hFeature, m_fields.at(m_tileIndexColumnName), 
        fileInfo->m_filename.c_str());

    if (fileInfo->m_srs.empty() || m_args.overrideASrs)
        fileInfo->m_srs = m_assignSrsString;

    if (fileInfo->m_srs.empty())
    {
        std::ostringstream oss;

        oss << "Unable to import source spatial reference '" <<
            fileInfo->m_srs << "' for file '" <<
            fileInfo->m_filename << "'.";
        OGR_F_Destroy(hFeature);
        throw TIndexError(oss.str());
    }
    if (fileInfo->m_srs != m_originalSrs)
    {
        m_log->get(LogLevel::Warning) << "SRS value for " << fileInfo->m_filename <<
            " does not match the SRS of other files in the tileindex." <<
            (m_args.skipMultiSrs ? " Skipping this file" : "") << std::endl;
        if (m_args.skipMultiSrs)
        {
            OGR_F_Destroy(hFeature);
            return false;
        }
    }

    std::string wkt = SpatialReference(fileInfo->m_srs).getWKT();
    setStringField(hFeature, m_fields.at(m_srsColumnName), wkt.c_str());

    createExtraFields(fileInfo, hFeature);

    Polygon g = prepareGeometry(fileInfo);
    OGR_F_SetGeometry(hFeature, g.getOGRHandle());

    const bool bRet = (OGR_L_CreateFeature(m_layer, hFeature) == OGRERR_NONE);
    OGR_F_Destroy(hFeature);

    if (bRet)
        m_log->get(LogLevel::Info) << "Indexed file " << fileInfo.m_filename <<
            std::endl;
    else
        m_log->get(LogLevel::Error) << "Failed to create feature "
            "for file '" << fileInfo.m_filename << "'" << std::endl;

    return bRet;
}

void TIndexBuilder::setStringField(OGRFeatureH hFeature, int idx,
    const char* value)
{
    if (m_maxFieldSize == 0 || strlen(value) <= m_maxFieldSize)
    {
        OGR_F_SetFieldString(hFeature, idx, value);
    }
    else
    {
        std::ostringstream oss;
        OGRFieldDefnH hFieldDefn = OGR_F_GetFieldDefnRef(hFeature, idx);

        oss << "value for field'" << OGR_Fld_GetNameRef(hFieldDefn) << "' has " << strlen(value) <<
            " characters; ESRI Shapefile driver supports a maximum of 254.";

        OGR_F_Destroy(hFeature);
        throw pdal_error(oss.str());
    }
}

bool TIndexBuilder::runBoundary(Stage& stage, FileInfo& fileInfo,
    PipelineManager& manager)
{
    // If we aren't able to make a hexbin filter, we
    // will just do a simple fast_boundary.
    bool fast(m_args.fastBoundary); 
    if (!fast)
    {
        TindexBoundary hexer{m_args.density, m_args.edgeLength, m_args.sampleSize};
        if (m_args.boundaryExpr.size())
        {
            Options opts;
            opts.add("where", m_args.boundaryExpr);
            hexer.addOptions(opts);
        }
        hexer.setInput(stage);
        manager.addStage(&hexer);
        try
        {
            manager.execute(ExecMode::PreferStream);

            fileInfo.m_boundary = hexer.toWKT();
            fileInfo.m_srs = hexer.getSpatialReference().getWKT();
            fileInfo.m_gridHeight = hexer.height();
        }
        catch(pdal_error& e)
        {
            fast = true;
            m_log->get(LogLevel::Warning) << "Unable to create exact boundary for tile " << 
                fileInfo.m_filename << " with error: '" << e.what() << std::endl;
        }
    }

    if (fast)
        return fastBoundary(stage, fileInfo);
    return true;
}

bool TIndexBuilder::fastBoundary(Stage& reader, FileInfo& fileInfo)
{
    QuickInfo qi = reader.preview();
    if (!qi.valid())
        return false;

    fileInfo.m_boundary = makeMultiPolygon(qi.m_bounds.to2d().toWKT());
    if (!qi.m_srs.empty())
        fileInfo.m_srs = qi.m_srs.getWKT();
    fileInfo.m_gridHeight = 0.0;
    return true;
}

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

pdal::Polygon TIndexBuilder::prepareGeometry(const std::unique_ptr<FileInfo>& fileInfo)
{
    using namespace gdal;

    Polygon g(fileInfo->m_boundary, fileInfo->m_srs);
    if (fileInfo->m_gridHeight && m_args.doSmooth)
    {
        double tolerance = 1.1 * fileInfo->m_gridHeight / 2;
        double cull = (6 * tolerance * tolerance);
        g.simplify(tolerance, cull);
        if (g.wkt()[0] == 'P')
        {
            std::string multi = makeMultiPolygon(g.wkt());
            g = Polygon(multi, fileInfo->m_srs);
        }
    }
    if (m_tgtSrsString.size())
    {
        SpatialReference out(m_tgtSrsString);
        g.transform(out);
    }

    return g;
}
//
// Standard class
//

TileIndex::TileIndex(const Args& args, const std::string& tileIndexColumnName,
        const std::string& srsColumnName, const std::string& driverName, const std::string& tgtSrs,
        const std::string& assignSrs)
    : TIndexBuilder(args, tileIndexColumnName, srsColumnName, driverName, tgtSrs, assignSrs) 
{
    m_fields.emplace("modified", OFTDateTime);
    m_fields.emplace("created", OFTDateTime);

    if (m_driverName == "ESRI Shapefile")
        m_maxFieldSize = 254;
}

std::unique_ptr<FileInfo> TileIndex::makeFileInfo(const std::string& filename)
{
    auto info = std::make_unique<FileInfo>();
    info->m_filename = filename;
    return info;
}

void TileIndex::getFileInfo(std::unique_ptr<FileInfo>& fileInfo)
{
    PipelineManager manager;
    manager.commonOptions() = m_commonOptions;
    manager.stageOptions() = m_stageOptions;

    Stage& reader = manager.makeReader(fileInfo->m_filename, "");
    runBoundary(reader, fileInfo, manager);  
}

void TileIndex::createExtraFields(std::unique_ptr<FileInfo>& fileInfo, OGRFeatureH hFeature)
{
    setDate(hFeature, fileInfo->m_ctime, "created");
    setDate(hFeature, fileInfo->m_mtime, "modified");
}

//
// STAC class
//

StacIndex::StacIndex(const Args& args, const std::string& pcType)
    : TIndexBuilder(args, "assets.data.href", "proj:wkt2", "Parquet", "EPSG:4326", "EPSG:4326"),
      m_pcType(pcType)
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

    m_extensions = { "https://stac-extensions.github.io/projection/v1.1.0/",
        "https://stac-extensions.github.io/pointcloud/v1.0.0/" };
}

std::unique_ptr<FileInfo> StacIndex::makeFileInfo(const std::string& filename)
{
    auto info = std::make_unique<StacFileInfo>();
    info->m_filename = filename;
    info->m_stacInfo.init(filename);
    return info;
}

void StacIndex::getFileInfo(std::unique_ptr<FileInfo>& fileInfo)
{
    StacFileInfo& stacFileInfo = static_cast<StacFileInfo&>(*fileInfo);

    PipelineManager manager;
    manager.commonOptions() = m_commonOptions;
    manager.stageOptions() = m_stageOptions;

    Stage& reader = manager.makeReader(stacFileInfo->m_filename, "");
    Stage& info = manager.makeFilter("filters.info", reader);
    Stage& stats = manager.makeFilter("filters.stats", info);

    if (runBoundary(stats, stacFileInfo, manager))
    {
        MetadataNode readerMeta = reader.getMetadata();
        MetadataNode statsMeta = stats.getMetadata();
        MetadataNode infoMeta = info.getMetadata();
        stacFileInfo.m_stacInfo.addMetadata(statsMeta, readerMeta, infoMeta);
    }
}

void StacIndex::createExtraFields(std::unique_ptr<FileInfo>& fileInfo, OGRFeatureH hFeature)
{
    // removing newlines to get rid of dead space in the parquet file. Not strictly necessary
    auto stripNewline = [](std::string& s) {
        s.erase(std::remove_if(s.begin(), s.end(), 
            [](char c) { return c == '\n' || c == '\r'; }), s.end());
        return s;
    };

    StacFileInfo& stacFileInfo = static_cast<StacFileInfo&>(*fileInfo);
    StacInfo& stacInfo = stacFileInfo.m_stacInfo;

    std::string linksJson = Utils::toJSON(stacInfo.rootChildren("links"));
    OGR_F_SetFieldString(hFeature, m_fields.at("links"), stripNewline(linksJson).c_str());

    OGR_F_SetFieldString(hFeature, m_fields.at("id"),
        FileUtils::getFilename(stacFileInfo.m_filename).c_str());

    OGR_F_SetFieldString(hFeature, m_fields.at("stac_version"), (const char*)STAC_VERSION);

    char** cslExtensions = NULL;
    for (auto& s : m_extensions)
        cslExtensions = CSLAddString(cslExtensions, s.c_str());
    OGR_F_SetFieldStringList(hFeature, m_fields.at("stac_extensions"), cslExtensions);
    CSLDestroy(cslExtensions);

    // Not added: proj:bbox, proj:geometry

    std::string projJson = SpatialReference(stacFileInfo.m_srs).getPROJJSON();
    OGR_F_SetFieldString(hFeature, m_fields.at("proj:projjson"), projJson.c_str());

    int pointCount = stacInfo.propertiesChild("pc:count").value<int>();
    OGR_F_SetFieldInteger(hFeature, m_fields.at("pc:count"), pointCount);

    std::string encoding = stacInfo.propertiesChild("pc:encoding").value();
    OGR_F_SetFieldString(hFeature, m_fields.at("pc:encoding"), encoding.c_str());

    OGR_F_SetFieldString(hFeature, m_fields.at("pc:type"), m_pcType.c_str());

    // Not sure if schema and statistics need to be native parquet lists or if json is ok
    std::string schema = Utils::toJSON(fileInfo.m_stacInfo.propertiesChildren("pc:schemas"));
    OGR_F_SetFieldString(hFeature, m_fields.at("pc:schemas"), stripNewline(schema).c_str());

    std::string statistics = 
        Utils::toJSON(fileInfo.m_stacInfo.propertiesChildren("pc:statistics"));
    OGR_F_SetFieldString(hFeature, m_fields.at("pc:statistics"), stripNewline(statistics).c_str());
}

} // namespace tindex
} // namespace pdal
