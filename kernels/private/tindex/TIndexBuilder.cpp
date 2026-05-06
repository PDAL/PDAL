#include "TIndexBuilder.hpp"
#include "TIndexBoundary.hpp"

#include <pdal/util/ThreadPool.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>
#include <pdal/private/gdal/SpatialRef.hpp>

#include <ogr_api.h>
#include <cpl_string.h>

namespace pdal
{
namespace tindex
{

//
// Base class
//

TIndexBuilder::TIndexBuilder(const Args& args, const std::string& tileIndexColumnName,
        const std::string& srsColumnName, const std::string& driverName, const std::string& tgtSrs,
        const std::string& assignSrs)
    : m_dataset(new TIndexDataset(args.idxFilename, driverName)),
      m_args(std::move(args)), 
      m_tileIndexColumnName(tileIndexColumnName),
      m_srsColumnName(srsColumnName), 
      m_driverName(driverName),
      m_tgtSrsString(tgtSrs),
      m_assignSrsString(assignSrs), 
      m_layerName(m_args.layerName),
      m_maxFieldSize(0)
{
    // Add necessary fields
    m_fields.emplace(m_tileIndexColumnName, OFTString);
    m_fields.emplace(m_srsColumnName, OFTString);

    if (m_driverName == "ESRI Shapefile")
        m_maxFieldSize = 254;
}

TIndexBuilder::~TIndexBuilder() {}

void TIndexBuilder::getFieldIndexes()
{
    for (auto& [name, info]: m_fields)
    {
        info.m_fieldIdx = m_dataset->getFieldIndex(name);
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
    if (!m_dataset->openDataset())
        if (!m_dataset->createDataset())
        {
            std::ostringstream out;
            out << "Couldn't open or create index dataset file '" <<
                m_args.idxFilename << "'.";
            throw TIndexError(out.str());
        }

    // Open or create a layer
    if (!m_dataset->openLayer(m_layerName))
        if (!m_dataset->createLayer(m_layerName, m_tgtSrsString, m_args.lcOptions))
        {
            std::ostringstream out;
            out << "Couldn't open or create layer '" << m_layerName <<
                "' in output file '" << m_args.idxFilename << "'.";
            throw TIndexError(out.str());
        }

    for (auto& [name, info]: m_fields)
        m_dataset->createField(name, info.m_fieldType, info.m_fieldSubType);

    for (auto& file : files)
    {
        // Sets filename, initializes STAC metadata
        auto info = makeFileInfo(file);
        info->m_isRemote = Utils::isRemote(file);
        if (!info->m_isRemote)
            FileUtils::fileTimes(info->m_filename, &info->m_ctime, &info->m_mtime);
        m_infos.push_back(std::move(info));
    }
    ThreadPool pool(m_args.numThreads);

    for (auto &info : m_infos)
    {
        pool.add([this, &info]()
        {
            getFileInfo(info);
        });
    }
    pool.await();

    m_originalSrs = m_infos[0]->m_srs;
    if (m_originalSrs.empty() || m_args.overrideASrs)
        m_originalSrs = m_assignSrsString;

    bool indexedFile(false);
    getFieldIndexes();

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
}

bool TIndexBuilder::createFeature(const std::unique_ptr<FileInfo>& fileInfo)
{
    using namespace gdal;

    TIndexFeature feature = m_dataset->buildFeature();

    feature.setField(m_fields.at(m_tileIndexColumnName), fileInfo->m_filename);

    if (fileInfo->m_srs.empty() || m_args.overrideASrs)
        fileInfo->m_srs = m_assignSrsString;

    if (fileInfo->m_srs.empty())
    {
        std::ostringstream oss;

        oss << "Unable to import source spatial reference '" <<
            fileInfo->m_srs << "' for file '" <<
            fileInfo->m_filename << "'.";
        throw TIndexError(oss.str());
    }
    if (fileInfo->m_srs != m_originalSrs)
    {
        m_log->get(LogLevel::Warning) << "SRS value for " << fileInfo->m_filename <<
            " does not match the SRS of other files in the tileindex." <<
            (m_args.skipMultiSrs ? " Skipping this file" : "") << std::endl;
        if (m_args.skipMultiSrs)
            return false;
    }

    std::string wkt = SpatialReference(fileInfo->m_srs).getWKT();
    feature.setField(m_fields.at(m_srsColumnName), wkt);

    createExtraFields(fileInfo, feature);

    Polygon g = prepareGeometry(fileInfo);
    feature.setGeometry(g);

    const bool bRet = m_dataset->createFeature(feature);

    if (bRet)
        m_log->get(LogLevel::Info) << "Indexed file " << fileInfo->m_filename <<
            std::endl;
    else
        m_log->get(LogLevel::Error) << "Failed to create feature "
            "for file '" << fileInfo->m_filename << "'" << std::endl;

    return bRet;
}


bool TIndexBuilder::isFileIndexed(const std::unique_ptr<FileInfo>& fileInfo)
{
    std::ostringstream qstring;

    qstring << "\"" <<  Utils::toupper(m_tileIndexColumnName) << "\"=" <<
        "'" << fileInfo->m_filename << "'";
    std::string query = qstring.str();

    bool output(false);
    try
    {
        output = m_dataset->queryLayer(query);
    }
    catch(TIndexError& e)
    {
        std::ostringstream out;
        out << "Unable to query layer for file '" << fileInfo->m_filename << "': " 
            << e.what();
        throw TIndexError(out.str());
    }
    return output;
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

std::string TIndexBuilder::makeMultiPolygon(const std::string& wkt)
{
    std::string multi = wkt + ')';
    multi.insert(8, "(");
    multi.insert(0, "MULTI");
    return multi;
}

} // namespace tindex
} // namespace pdal
