/******************************************************************************
* Copyright (c) 2012, Howard Butler, hobu.inc@gmail.com
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

#include "SQLiteWriter.hpp"
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/pdal_internal.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <iomanip>
#include <sstream>

#include <gdal.h>
#include <ogr_api.h>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "writers.sqlite",
    "Write data to SQLite3 database files.",
    "" );

CREATE_SHARED_PLUGIN(1, 0, SQLiteWriter, Writer, s_info)

std::string SQLiteWriter::getName() const { return s_info.name; }

SQLiteWriter::SQLiteWriter() :
    m_doCreateIndex(false)
    , m_sdo_pc_is_initialized(false)
    , m_obj_id(0)
    , m_block_id(0)
{}


void SQLiteWriter::addArgs(ProgramArgs& args)
{
    DbWriter::doAddArgs(args);
    args.add("block_table_name", "Block table name",
        m_block_table).setPositional();
    args.add("cloud_table_name", "Cloud table name",
        m_cloud_table).setPositional();
    args.add("connection", "SQL connection string",
        m_connection).setPositional();
    args.addSynonym("connection", "filename");
    args.add("cloud_column_name", "Cloud column name", m_cloud_column, "id");
    args.add("module", "Module name", m_modulename);
    args.add("srid", "SRID", m_srid, 4326U);
    args.add("pcid", "PCID", m_pcId);
    args.add("is3d", "Whether a Z dimension should be written", m_is3d);
    args.add("compression", "Whether blocks should be compressed",
        m_doCompression);
    args.add("overwrite", "Whether existing data should be overwritten",
        m_overwrite);
    args.add("pre_sql", "SQL to be executed before running the translation, "
        "or the name of a file containing such SQL", m_preSql);
    args.add("post_sql", "SQL to be executed after running the translation, "
        "or the name of a file containing such SQL", m_postSql);
    args.add("clound_boundary_wkt", "Boundary of points to be written",
        m_cloudBoundary);
}


void SQLiteWriter::initialize()
{
    try
    {
        log()->get(LogLevel::Debug) << "Connection: '" << m_connection <<
            "'" << std::endl;
        m_session = std::unique_ptr<SQLite>(new SQLite(m_connection, log()));
        m_session->connect(true);
        log()->get(LogLevel::Debug) << "Connected to database" << std::endl;

        bool bHaveSpatialite = m_session->haveSpatialite();
        log()->get(LogLevel::Debug) << "Have spatialite?: " <<
            bHaveSpatialite << std::endl;
        m_session->loadSpatialite(m_modulename);

        if (!bHaveSpatialite)
        {
            m_session->initSpatialiteMetadata();
        }

    }
    catch (pdal_error const& e)
    {
        throwError("Unable to connect to database with error '" +
            std::string(e.what()));
    }

    m_patch = PatchPtr(new Patch());
}


void SQLiteWriter::write(const PointViewPtr view)
{
    writeInit();
    writeTile(view);
}

void SQLiteWriter::writeInit()
{
    if (m_sdo_pc_is_initialized)
        return;

    m_block_insert_query << "INSERT INTO " <<
        Utils::tolower(m_block_table) << " ("<<
        Utils::tolower(m_cloud_column) <<
        ", block_id, num_points, points, extent, bbox) VALUES (" <<
        " ?, ?, ?, ?, "
        "ST_GeometryFromText(?,?), ?)";

    // m_block_insert_query << "INSERT INTO " <<
    //     Utils::tolower(m_block_table) << " ("<<
    //     Utils::tolower(m_cloud_column) <<
    //     ", block_id, num_points, points, extent, bbox) VALUES (" <<
    //     " :obj_id, :block_id, :num_points, decode(:hex, 'hex'), "
    //     "ST_Force_2D(ST_GeometryFromText(:extent,:srid)), :bbox)";

    m_session->begin();

    bool bHaveBlockTable = m_session->doesTableExist(m_block_table);
    bool bHaveCloudTable = m_session->doesTableExist(m_cloud_table);

    log()->get(LogLevel::Debug) << "bHaveBlockTable '"
                         << bHaveBlockTable
                         <<"'"<< std::endl;
    log()->get(LogLevel::Debug) << "bHaveCloudTable '"
                         << bHaveCloudTable
                         <<"'"<< std::endl;

    if (m_overwrite)
    {
        if (bHaveBlockTable)
        {
            DeleteBlockTable();
            bHaveBlockTable = false;
        }
        if (bHaveCloudTable)
        {
            DeleteCloudTable();
            bHaveCloudTable = false;
        }
    }

    if (m_preSql.size())
    {
        std::string sql = FileUtils::readFileIntoString(m_preSql);
        if (!sql.size())
        {
            // if there was no file to read because the data in pre_sql was
            // actually the sql code the user wanted to run instead of the
            // filename to open, we'll use that instead.
            sql = m_preSql;
        }
        m_session->execute(sql);
    }

    if (!bHaveCloudTable)
    {
        CreateCloudTable();
    }

    if (!bHaveBlockTable)
    {
        m_doCreateIndex = true;
        CreateBlockTable();
    }
    CreateCloud();
    m_sdo_pc_is_initialized = true;
}


void SQLiteWriter::CreateBlockTable()
{
    std::ostringstream oss;

    oss << "CREATE TABLE " << Utils::tolower(m_block_table)
        << "(" << Utils::tolower(m_cloud_column)  <<
        " INTEGER REFERENCES " << Utils::tolower(m_cloud_column)  <<
        "," << " block_id INTEGER," << " num_points INTEGER," <<
        " points BLOB," << " bbox box3d " << ")";

    m_session->execute(oss.str());
    log()->get(LogLevel::Debug) << "Created block table '"
        << Utils::tolower(m_block_table) << "'" <<std::endl;

    {
        std::ostringstream oss;
        oss << "SELECT AddGeometryColumn('" <<
            Utils::tolower(m_block_table) << "'," << "'extent'" << ","
            << m_srid << ", 'POLYGON', 'XY')";
        m_session->execute(oss.str());
        log()->get(LogLevel::Debug) <<
            "Added geometry column for block table '" <<
            Utils::tolower(m_block_table) <<"'"<< std::endl;
    }
}

void SQLiteWriter::DeleteBlockTable()
{
    std::ostringstream oss;

    // Delete all the items from the table first
    oss << "DELETE FROM " << m_block_table;
    m_session->execute(oss.str());
    oss.str("");
    log()->get(LogLevel::Debug) << "Deleted rows from block table '" <<
        Utils::tolower(m_block_table) << "'" <<std::endl;

    // Drop the table's dependencies
    // We need to clean up the geometry column before dropping the table
    oss << "SELECT DiscardGeometryColumn('" <<
        Utils::tolower(m_block_table) << "', 'extent')";
    m_session->execute(oss.str());
    log()->get(LogLevel::Debug) << "Dropped geometry column for block table"
                         << std::endl;
    oss.str("");

    oss << "DROP TABLE " << Utils::tolower(m_block_table);
    m_session->execute(oss.str());
    log()->get(LogLevel::Debug) << "Dropped block table '" <<
        Utils::tolower(m_block_table) << "'" <<std::endl;
}


void SQLiteWriter::CreateCloudTable()
{
    std::ostringstream oss;

    oss << "CREATE TABLE " << Utils::tolower(m_cloud_table) << " (" <<
        Utils::tolower(m_cloud_column) <<
        " INTEGER PRIMARY KEY AUTOINCREMENT," << " schema TEXT," <<
        " block_table varchar(64)" << ")";
    m_session->execute(oss.str());
    log()->get(LogLevel::Debug) << "Created cloud table '"
        << Utils::tolower(m_cloud_table) << "'" << std::endl;

    uint32_t nDim = 2;

    oss.str("");
    oss << "SELECT AddGeometryColumn('"
        << Utils::tolower(m_cloud_table)
        << "'," << "'extent'" << "," << m_srid << ", 'POLYGON', 'XY')";
    m_session->execute(oss.str());
    log()->get(LogLevel::Debug) << "Added geometry column to cloud table '" <<
        Utils::tolower(m_cloud_table) << "'" <<std::endl;
}


void SQLiteWriter::DeleteCloudTable()
{
    std::ostringstream oss;

    // Delete all the items from the table first
    oss << "DELETE FROM " << m_cloud_table;
    m_session->execute(oss.str());
    oss.str("");
    log()->get(LogLevel::Debug) << "Deleted records from cloud table '"
                         << Utils::tolower(m_cloud_table)
                         << "'" <<std::endl;

    // Go drop the table
    // We need to clean up the geometry column before dropping the table
    oss << "SELECT DiscardGeometryColumn('" <<
        Utils::tolower(m_cloud_table) << "', 'extent')";
    m_session->execute(oss.str());
    oss.str("");
    log()->get(LogLevel::Debug) << "Dropped geometry column from cloud table '"
                         << Utils::tolower(m_cloud_table)
                         << "'" <<std::endl;

    oss << "DROP TABLE " << Utils::tolower(m_cloud_table);
    m_session->execute(oss.str());
    oss.str("");
    log()->get(LogLevel::Debug) << "Dropped cloud table '"
                         << Utils::tolower(m_cloud_table)
                         << "'" <<std::endl;
}


void SQLiteWriter::CreateIndexes(std::string const& table_name,
    std::string const& spatial_column_name, bool is3d)
{
    std::ostringstream oss;
    std::ostringstream index_name_ss;

    index_name_ss << table_name << "_cloud_idx";
    std::string index_name = index_name_ss.str().substr(0,29);

    // Spatial indexes
    oss << "SELECT CreateSpatialIndex('"<< Utils::tolower(table_name) <<
        "', 'extent')";
    m_session->execute(oss.str());
    log()->get(LogLevel::Debug) << "Created spatial index for'" <<
        table_name << "'" << std::endl;
}


std::string
SQLiteWriter::loadGeometryWKT(std::string const& filename_or_wkt) const
{
    std::ostringstream wkt_s;

    if (filename_or_wkt.empty())
        return std::string();

    if (!FileUtils::fileExists(filename_or_wkt))
    {
        if (!IsValidGeometryWKT(filename_or_wkt))
            throwError("WKT for not valid and '" + filename_or_wkt +
                "' doesn't exist as a file");
        wkt_s << filename_or_wkt;
    }
    else
    {
        std::string wkt = FileUtils::readFileIntoString(filename_or_wkt);
        if (!IsValidGeometryWKT(wkt))
            throwError("WKT for was from file '" + filename_or_wkt +
                "' is not valid");
        wkt_s << wkt;
    }
    return wkt_s.str();
}


bool SQLiteWriter::IsValidGeometryWKT(std::string const& input) const
{
    OGRGeometryH g;

    char* wkt = const_cast<char*>(input.c_str());
    OGRErr e = OGR_G_CreateFromWkt(&wkt, NULL, &g);
    OGR_G_DestroyGeometry(g);
    return (!e);
}

void SQLiteWriter::done(PointTableRef table)
{
    if (m_doCreateIndex)
    {
        CreateIndexes(m_block_table, "extent", m_is3d);
    }

    if (m_postSql.size())
    {
        std::string sql = FileUtils::readFileIntoString(m_postSql);
        if (!sql.size())
        {
            // if there was no file to read because the data in m_postSql was
            // actually the sql code the user wanted to run instead of the
            // filename to open, we'll use that instead.
            sql = m_postSql;
        }
        m_session->execute(sql);
    }

    m_session->commit();

}


void SQLiteWriter::CreateCloud()
{
    using namespace std;

    ostringstream oss;

    if (m_cloudBoundary.size())
        m_cloudBoundary = loadGeometryWKT(m_cloudBoundary);

    oss << "INSERT INTO " << Utils::tolower(m_cloud_table) << "(" <<
        " block_table, schema) VALUES ('" <<
        Utils::tolower(m_block_table) << "',?) ";

    MetadataNode m;
    if (m_doCompression)
    {
        Metadata metadata;
        m = metadata.getNode();
        m.add("compression", "lazperf");
        m.add("version", "1.0");
    }
    XMLSchema schema(dbDimTypes(), m);
    std::string xml = schema.xml();

    records rs;
    row r;
    r.push_back(column(xml));
    rs.push_back(r);
    m_session->insert(oss.str(), rs);
    oss.str("");

    long id = m_session->last_row_id();
    m_obj_id = id;

    log()->get(LogLevel::Debug) << "Point cloud id was " << id << std::endl;
    m_pcId = id;
    if (m_cloudBoundary.size())
    {
        records rs;
        row r;

        r.push_back(column(m_cloudBoundary));
        r.push_back(column(m_srid));
        r.push_back(column(id));
        rs.push_back(r);

        oss << "UPDATE " << Utils::tolower(m_cloud_table) <<
            " SET extent="
            "ST_GeometryFromText(?,?) where " <<
            Utils::tolower(m_cloud_column) <<"=?";

        m_session->insert(oss.str(), rs);
        log()->get(LogLevel::Debug) <<
            "Inserted boundary wkt into cloud table " << std::endl;
    }
}


void SQLiteWriter::writeTile(const PointViewPtr view)
{
    using namespace std;

    if (m_doCompression)
    {
#ifdef PDAL_HAVE_LAZPERF
        XMLDimList xmlDims = dbDimTypes();
        DimTypeList dimTypes;
        for (XMLDim& xmlDim : xmlDims)
            dimTypes.push_back(xmlDim.m_dimType);

        LazPerfCompressor<Patch> compressor(*m_patch, dimTypes);

        try
        {
            std::vector<char> outbuf(packedPointSize());
            for (PointId idx = 0; idx < view->size(); idx++)
            {
                size_t size = readPoint(*view.get(), idx, outbuf.data());
                // Read the data and write to the patch.
                compressor.compress(outbuf.data(), size);
            }
        }
        catch (pdal_error)
        {
            compressor.done();
            throw;
        }
        compressor.done();
#else
        throwError("Can't compress without LAZperf.");
#endif

        size_t viewSize = view->size() * view->pointSize();
        double percent = (double) m_patch->byte_size()/(double) viewSize;
        percent = percent * 100;
        log()->get(LogLevel::Debug3) << "Compressing tile by " <<
            std::setprecision(2) << (100 - percent) << "%" << std::endl;
    }
    else
    {
        std::vector<char> storage(packedPointSize());

        for (PointId idx = 0; idx < view->size(); idx++)
        {
            size_t size = readPoint(*view.get(), idx, storage.data());
            m_patch->putBytes((const unsigned char *)storage.data(), size);
        }
        log()->get(LogLevel::Debug3) << "uncompressed size: " <<
            m_patch->getBytes().size() << std::endl;
    }

    records rs;
    row r;

    uint32_t precision(9);
    BOX3D b;
    view->calculateBounds(b);
    std::string bounds = b.toWKT(precision); // polygons are only 2d, not cubes

    std::string box = b.toBox(precision);
    log()->get(LogLevel::Debug3) << "extent: " << bounds << std::endl;
    log()->get(LogLevel::Debug3) << "bbox: " << box << std::endl;

    r.push_back(column(m_obj_id));
    r.push_back(column(m_block_id));
    r.push_back(column(view->size()));
    r.push_back(blob((const char*)(&m_patch->getBytes()[0]),
        m_patch->getBytes().size()));
    r.push_back(column(bounds));
    r.push_back(column(m_srid));
    r.push_back(column(box));
    rs.push_back(r);
    m_session->insert(m_block_insert_query.str(), rs);
    m_block_id++;
    m_patch->clear();

}

} // namespaces
