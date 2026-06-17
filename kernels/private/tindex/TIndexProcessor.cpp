#include "TIndexProcessor.hpp"
#include "TIndexBoundary.hpp"
#include "Dataset.hpp"
#include "TIndexError.hpp"

#include <pdal/Polygon.hpp>
#include <pdal/util/ThreadPool.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>
#include <pdal/private/gdal/SpatialRef.hpp>
#include <pdal/Polygon.hpp>

namespace pdal
{
namespace tindex
{

//
// Base class
//

TIndexProcessor::TIndexProcessor(const Args& args, const std::string& tileIndexColumnName,
        const std::string& srsColumnName, const std::string& driverName, const std::string& tgtSrs,
        const std::string& assignSrs)
    : m_dataset(new Dataset(args.idxFilename, driverName)),
      m_args(args),
      m_tileIndexColumnName(tileIndexColumnName),
      m_srsColumnName(srsColumnName),
      m_driverName(driverName),
      m_tgtSrsString(tgtSrs),
      m_assignSrsString(assignSrs),
      m_layerName(m_args.layerName)
{
    m_tindexColumnNameField = m_dataset->defineField(m_tileIndexColumnName, OFTString);
    m_srsColumnNameField = m_dataset->defineField(m_srsColumnName, OFTString);
}

TIndexProcessor::~TIndexProcessor()
{}

void TIndexProcessor::create(const StringList& files, PipelineManager& mgr)
{
    const std::string filename = files.front();
    m_infos.reserve(files.size());

    m_commonOptions = mgr.commonOptions();
    m_stageOptions = mgr.stageOptions();
    m_log = mgr.log();

    if (m_layerName.empty())
       m_layerName = FileUtils::stem(filename);

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
    m_dataset->createFields();

    for (const std::string& file : files)
        m_infos.push_back(makeFileInfo(file));

    ThreadPool pool(m_args.numThreads);

    for (auto &info : m_infos)
    {
        pool.add([this, &info]()
        {
            fillFileInfo(info);
        });
    }
    pool.await();

    m_originalSrs = m_infos[0]->m_srs;
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
}

std::vector<FileInfo> TIndexProcessor::readIndex()
{
    if (!m_dataset->openDataset())
    {
        std::ostringstream out;
        out << "Couldn't open index dataset file '" << m_args.idxFilename << "'.";
        throw TIndexError(out.str());
    }
    if (!m_dataset->openLayer(m_args.layerName))
    {
        std::ostringstream out;
        out << "Couldn't open layer '" << m_args.layerName <<
            "' in output file '" << m_args.idxFilename << "'.";
        throw TIndexError(out.str());
    }

    m_dataset->getFieldIndexes();

    if (!m_args.wkt.empty())
    {
        pdal::Polygon g(m_args.wkt, m_tgtSrsString);
        m_dataset->setSpatialFilter(g);
    }

    std::vector<FileInfo> files;

    // Docs are bad here.  You need this call even if you haven't read anything
    // or nothing happens.
    m_dataset->resetReading();
    while (true)
    {
        Feature feature = m_dataset->getNextFeature();
        if (!feature.getFeature())
            break;

        FileInfo fileInfo(feature.getField(m_tindexColumnNameField));
        fileInfo.m_srs =
            feature.getField(m_srsColumnNameField);
        files.push_back(fileInfo);
    }

    return files;
}

bool TIndexProcessor::createFeature(const FileInfoPtr& fileInfo)
{

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

    Feature feature = m_dataset->buildFeature();
    feature.setField(m_tindexColumnNameField, fileInfo->m_filename);
    feature.setField(m_srsColumnNameField, SpatialReference(fileInfo->m_srs).getWKT());
    createExtraFields(fileInfo, feature);
    feature.setGeometry(prepareGeometry(*fileInfo));

    bool bRet = m_dataset->createFeature(feature);
    if (bRet)
        m_log->get(LogLevel::Info) << "Indexed file " << fileInfo->m_filename <<
            std::endl;
    else
        m_log->get(LogLevel::Error) << "Failed to create feature "
            "for file '" << fileInfo->m_filename << "'" << std::endl;

    return bRet;
}


bool TIndexProcessor::isFileIndexed(const FileInfoPtr& fileInfo)
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
        out << "Unable to execute OGR attribute query: " << e.what();
        throw TIndexError(out.str());
    }
    return output;
}

bool TIndexProcessor::runBoundary(FileInfo& fileInfo, PipelineManager& manager)
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

        // Could be a reader or a stats filter
        Stage *stage = manager.stages().back();
        hexer.setInput(*stage);
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
            // Destroy the hexbin filter -- the manager gets executed again for STAC
            manager.destroyStage(&hexer);
            m_log->get(LogLevel::Warning) << "Unable to create exact boundary for tile " <<
                fileInfo.m_filename << " with error: '" << e.what() << std::endl;
        }
    }

    if (fast)
        return fastBoundary(manager, fileInfo);

    return true;
}

pdal::Polygon TIndexProcessor::prepareGeometry(const FileInfo& fileInfo)
{
    auto makeMultiPolygon = [](std::string poly) -> std::string
    {
        // Erase "POLYGON" and wrap with "MULTIPOLYGON(..)"
        if (Utils::startsWith(poly, "POLYGON"))
            return poly.replace(0, 7, "MULTIPOLYGON(") + ")";
        return poly;
    };

    Polygon g(fileInfo.m_boundary, fileInfo.m_srs);
    if (fileInfo.m_gridHeight && m_args.doSmooth)
    {
        double tolerance = 1.1 * fileInfo.m_gridHeight / 2;
        double cull = (6 * tolerance * tolerance);
        g.simplify(tolerance, cull);
    }
    g = Polygon(makeMultiPolygon(g.wkt()), fileInfo.m_srs);
    g.transform(SpatialReference(m_tgtSrsString));

    return g;
}

} // namespace tindex
} // namespace pdal
