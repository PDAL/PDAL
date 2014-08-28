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

#include <pdal/drivers/sqlite/SQLiteWriter.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/pdal_internal.hpp>
#include <pdal/FileUtils.hpp>


#include <sstream>

#ifdef PDAL_HAVE_GDAL
#include <gdal.h>
#include <ogr_api.h>
#endif

#ifdef USE_PDAL_PLUGIN_SQLITE
MAKE_WRITER_CREATOR(sqliteWriter, pdal::drivers::sqlite::Writer)
CREATE_WRITER_PLUGIN(sqlite, pdal::drivers::sqlite::Writer)
#endif


namespace pdal
{
namespace drivers
{
namespace sqlite
{


SQLiteWriter::SQLiteWriter(const Options& options)
    : pdal::Writer(options)
    , m_doCreateIndex(false)
    , m_bounds(Bounds<double>())
    , m_sdo_pc_is_initialized(false)
    , m_obj_id(0)
    , m_block_id(0)
    , m_srid(0)
    , m_num_points(0)
    , m_pointSize(0)
    , m_orientation(Orientation::PointMajor)
    , m_is3d(false)
{}

void SQLiteWriter::processOptions(const Options& options)
{
    m_connection =
        options.getValueOrDefault<std::string>("connection", "");
    if (!m_connection.size())
    {
        m_connection =
            options.getValueOrDefault<std::string>("filename", "");
        
        if (!m_connection.size())
        throw sqlite_driver_error("unable to connect to database, "
            "no connection string was given!");
    }
    m_block_table =
        options.getValueOrThrow<std::string>("block_table_name");
    m_cloud_table =
        options.getValueOrThrow<std::string>("cloud_table_name");
    m_cloud_column =
        options.getValueOrDefault<std::string>("cloud_column_name", "id");
    m_srid =
        m_options.getValueOrDefault<boost::uint32_t>("srid", 4326);
    m_is3d = m_options.getValueOrDefault<bool>("is3d", false);    
}


void SQLiteWriter::initialize()
{
    try
    {
        log()->get(LogLevel::Debug) << "Connection: '" << m_connection << "'" << std::endl;
        m_session = std::unique_ptr<SQLite>(new SQLite(m_connection, log()));
        m_session->connect(true);
        log()->get(LogLevel::Debug) << "Connected to database" << std::endl;
        bool bHaveSpatialite = m_session->doesTableExist("geometry_columns");
        log()->get(LogLevel::Debug) << "Have spatialite?: " << bHaveSpatialite << std::endl;
        m_session->spatialite();

        if (!bHaveSpatialite)
        {
            std::ostringstream oss;
            oss << "SELECT InitSpatialMetadata()";
            m_session->execute(oss.str());
        }
        
    }
    catch (sqlite::sqlite_driver_error const& e)
    {
        std::stringstream oss;
        oss << "Unable to connect to database with error '" << e.what() << "'";
        throw pdal_error(oss.str());
    }
}

void SQLiteWriter::ready(PointContext ctx)
{
    m_dims = ctx.dims();
    // Determine types for the dimensions.  We use the default types when
    // they exist, float otherwise.
    for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
    {
        Dimension::Type::Enum type = Dimension::defaultType(*di);
        if (type == Dimension::Type::None)
            type = Dimension::Type::Float;
        m_types.push_back(type);
        m_pointSize += Dimension::size(type);
    }
}

void SQLiteWriter::write(const PointBuffer& buffer)
{
    writeInit();
    writeTile(buffer);
}

void SQLiteWriter::writeInit()
{
    if (m_sdo_pc_is_initialized)
        return;
    
    // m_block_insert_query << "INSERT INTO " <<
    //     boost::to_lower_copy(m_block_table) << " ("<<
    //     boost::to_lower_copy(m_cloud_column) <<
    //     ", block_id, num_points, points, extent, bbox) VALUES (" <<
    //     " ?, ?, ?, decode(?, 'hex'), "
    //     "ST_Force_2D(ST_GeometryFromText(?,?)), ?)";
    m_block_insert_query << "INSERT INTO " <<
        boost::to_lower_copy(m_block_table) << " ("<<
        boost::to_lower_copy(m_cloud_column) <<
        ", block_id, num_points, points, extent, bbox) VALUES (" <<
        " ?, ?, ?, ?, "
        "ST_GeometryFromText(?,?), ?)";    
    
    // m_block_insert_query << "INSERT INTO " <<
    //     boost::to_lower_copy(m_block_table) << " ("<<
    //     boost::to_lower_copy(m_cloud_column) <<
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
        
    if (m_options.getValueOrDefault<bool>("overwrite", true))
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

    std::string pre_sql = m_options.getValueOrDefault<std::string>("pre_sql", "");
    if (pre_sql.size())
    {
        std::string sql = FileUtils::readFileAsString(pre_sql);
        if (!sql.size())
        {
            // if there was no file to read because the data in pre_sql was
            // actually the sql code the user wanted to run instead of the
            // filename to open, we'll use that instead.
            sql = pre_sql;
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

    oss << "CREATE TABLE " << boost::to_lower_copy(m_block_table)
        << "(" << boost::to_lower_copy(m_cloud_column)  <<
        " INTEGER REFERENCES " << boost::to_lower_copy(m_cloud_column)  <<
        "," << " block_id INTEGER," << " num_points INTEGER," <<
        " points BLOB," << " bbox box3d " << ")";

    m_session->execute(oss.str());
    log()->get(LogLevel::Debug) << "Created block table '" 
                         << boost::to_lower_copy(m_block_table) 
                         << "'" <<std::endl;        

    oss.str("");
    {
        oss << "SELECT AddGeometryColumn('" << boost::to_lower_copy(m_block_table)
            << "'," << "'extent'" << ","
            << m_srid << ", 'POLYGON', 'XY')";
        m_session->execute(oss.str());
        log()->get(LogLevel::Debug) << "Added geometry column for block table '" 
                             << boost::to_lower_copy(m_block_table) 
                             <<"'"<< std::endl;        
    }
}

void SQLiteWriter::DeleteBlockTable()
{
    std::ostringstream oss;

    // Delete all the items from the table first
    oss << "DELETE FROM " << m_block_table;
    m_session->execute(oss.str());
    oss.str("");
    log()->get(LogLevel::Debug) << "Deleted rows from block table '" 
                         << boost::to_lower_copy(m_block_table) 
                         << "'" <<std::endl;        
   
    // Drop the table's dependencies
    // We need to clean up the geometry column before dropping the table
    oss << "SELECT DiscardGeometryColumn('" <<
        boost::to_lower_copy(m_block_table) << "', 'extent')";
    m_session->execute(oss.str());
    log()->get(LogLevel::Debug) << "Dropped geometry column for block table" 
                         << std::endl;        
    oss.str("");

    oss << "DROP TABLE " << boost::to_lower_copy(m_block_table);
    m_session->execute(oss.str());
    log()->get(LogLevel::Debug) << "Dropped block table '" 
                         << boost::to_lower_copy(m_block_table) 
                         << "'" <<std::endl;        

}


void SQLiteWriter::CreateCloudTable()
{
    std::ostringstream oss;

    oss << "CREATE TABLE " << boost::to_lower_copy(m_cloud_table) << " (" <<
        boost::to_lower_copy(m_cloud_column) <<
        " INTEGER PRIMARY KEY AUTOINCREMENT," << " schema TEXT," <<
        " block_table varchar(64)" << ")";
    m_session->execute(oss.str());
    log()->get(LogLevel::Debug) << "Created cloud table '" 
                         << boost::to_lower_copy(m_cloud_table) 
                         << "'" <<std::endl;        

    boost::uint32_t nDim = 2;

    oss.str("");
    oss << "SELECT AddGeometryColumn('" 
        << boost::to_lower_copy(m_cloud_table) 
        << "'," << "'extent'" << "," << m_srid << ", 'POLYGON', 'XY')";
    m_session->execute(oss.str());
    log()->get(LogLevel::Debug) << "Added geometry column to cloud table '" 
                         << boost::to_lower_copy(m_cloud_table) << "'" <<std::endl;        
}

void SQLiteWriter::DeleteCloudTable()
{
    std::ostringstream oss;

    // Delete all the items from the table first
    oss << "DELETE FROM " << m_cloud_table;
    m_session->execute(oss.str());
    oss.str("");
    log()->get(LogLevel::Debug) << "Deleted records from cloud table '" 
                         << boost::to_lower_copy(m_cloud_table) 
                         << "'" <<std::endl;        

    // Go drop the table
    // We need to clean up the geometry column before dropping the table
    oss << "SELECT DiscardGeometryColumn('" <<
        boost::to_lower_copy(m_cloud_table) << "', 'extent')";
    m_session->execute(oss.str());
    oss.str("");
    log()->get(LogLevel::Debug) << "Dropped geometry column from cloud table '" 
                         << boost::to_lower_copy(m_cloud_table) 
                         << "'" <<std::endl;

    oss << "DROP TABLE " << boost::to_lower_copy(m_cloud_table);
    m_session->execute(oss.str());
    oss.str("");
    log()->get(LogLevel::Debug) << "Dropped cloud table '" 
                         << boost::to_lower_copy(m_cloud_table) 
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
    oss << "SELECT CreateSpatialIndex('"<< boost::to_lower_copy(table_name) <<
        "', 'extent')";
    m_session->execute(oss.str());
    log()->get(LogLevel::Debug) << "Created spatial index for'" << table_name << "'" <<std::endl;
}


std::string SQLiteWriter::loadGeometryWKT(std::string const& filename_or_wkt) const
{
    std::ostringstream wkt_s;

    if (filename_or_wkt.empty())
        return std::string();

    if (!FileUtils::fileExists(filename_or_wkt))
    {
        if (!IsValidGeometryWKT(filename_or_wkt))
        {
            std::ostringstream oss;
            oss << "WKT for not valid and '" << filename_or_wkt
                << "' doesn't exist as a file";
            throw pdal::pdal_error(oss.str());
        }
        wkt_s << filename_or_wkt;
    }
    else
    {
        std::string wkt = FileUtils::readFileAsString(filename_or_wkt);
        if (!IsValidGeometryWKT(wkt))
        {
            std::ostringstream oss;
            oss << "WKT for was from file '" << filename_or_wkt
                << "' is not valid";
            throw pdal::pdal_error(oss.str());
        }
        wkt_s << wkt;
    }
    return wkt_s.str();
}


bool SQLiteWriter::IsValidGeometryWKT(std::string const& input) const
{
#ifdef PDAL_HAVE_GDAL
    OGRGeometryH g;

    char* wkt = const_cast<char*>(input.c_str());
    OGRErr e = OGR_G_CreateFromWkt(&wkt, NULL, &g);
    OGR_G_DestroyGeometry(g);
    return (!e);
#else
    throw pdal_error("GDAL support not available for WKT validation");
#endif
}

void SQLiteWriter::done(PointContext ctx)
{
    if (m_doCreateIndex)
    {
        CreateIndexes(m_block_table, "extent", m_is3d);
    }
    
    std::string post_sql = m_options.getValueOrDefault<std::string>("post_sql", "");
    if (post_sql.size())
    {
        std::string sql = FileUtils::readFileAsString(post_sql);
        if (!sql.size())
        {
            // if there was no file to read because the data in post_sql was
            // actually the sql code the user wanted to run instead of the
            // filename to open, we'll use that instead.
            sql = post_sql;
        }
        m_session->execute(sql);
    }
    
    m_session->commit();
    
}


void SQLiteWriter::CreateCloud()
{
    using namespace std;

    ostringstream oss;

    bool pack =
        m_options.getValueOrDefault<bool>("pack_ignored_fields", true);

    string bounds = m_options.getValueOrDefault<string>(
        "cloud_boundary_wkt", "");
    if (bounds.size())
    {
        log()->get(LogLevel::Debug2) << "have cloud_boundary_wkt of size " <<
            bounds.size() << std::endl;
        bounds = loadGeometryWKT(bounds);
    }

    string cloud_column =
        m_options.getValueOrDefault<string>("cloud_column", "id");

    oss << "INSERT INTO " << boost::to_lower_copy(m_cloud_table) << "(" <<
        " block_table, schema) VALUES ('" <<
        boost::to_lower_copy(m_block_table) << "',?) ";

    schema::Writer writer(m_dims, m_types);
    std::string xml = writer.getXML();
    
    records rs;
    row r;
    r.push_back(column(xml)); 
    rs.push_back(r);
    m_session->insert(oss.str(), rs);
    oss.str("");

    long id = m_session->last_row_id();
    m_obj_id = id;

    log()->get(LogLevel::Debug) << "Point cloud id was " << id << std::endl;
    try
    {
        Option& pc_id = m_options.getOptionByRef("pc_id");
        pc_id.setValue(id);
    }
    catch (pdal::option_not_found&)
    {
        Option pc_id("pc_id", id, "Point Cloud Id");
        m_options.add(pc_id);
    }
    if (bounds.size())
    {
        records rs;
        row r;
          
        r.push_back(column(bounds)); 
        r.push_back(column(m_srid));
        r.push_back(column(id));
        rs.push_back(r);

        oss << "UPDATE " << boost::to_lower_copy(m_cloud_table) <<
            " SET extent="
            "ST_GeometryFromText(?,?) where " <<
            boost::to_lower_copy(m_cloud_column) <<"=?";

        m_session->insert(oss.str(), rs);
        log()->get(LogLevel::Debug) << "Inserted boundary wkt into cloud table " << std::endl;        
    }
}


namespace
{

void fillBuf(const PointBuffer& buf, char *pos, Dimension::Id::Enum d,
    Dimension::Type::Enum type, PointId id)
{
    union
    {
        float f;
        double d;
        int8_t s8;
        int16_t s16;
        int32_t s32;
        int64_t s64;
        uint8_t u8;
        uint16_t u16;
        uint32_t u32;
        uint64_t u64;
    } e;  // e - for Everything.

    switch (type)
    {
    case Dimension::Type::Float:
        e.f = buf.getFieldAs<float>(d, id);
        break;
    case Dimension::Type::Double:
        e.d = buf.getFieldAs<double>(d, id);
        break;
    case Dimension::Type::Signed8:
        e.s8 = buf.getFieldAs<int8_t>(d, id);
        break;
    case Dimension::Type::Signed16:
        e.s16 = buf.getFieldAs<int16_t>(d, id);
        break;
    case Dimension::Type::Signed32:
        e.s32 = buf.getFieldAs<int32_t>(d, id);
        break;
    case Dimension::Type::Signed64:
        e.s64 = buf.getFieldAs<int64_t>(d, id);
        break;
    case Dimension::Type::Unsigned8:
        e.u8 = buf.getFieldAs<uint8_t>(d, id);
        break;
    case Dimension::Type::Unsigned16:
        e.u16 = buf.getFieldAs<uint16_t>(d, id);
        break;
    case Dimension::Type::Unsigned32:
        e.u32 = buf.getFieldAs<uint32_t>(d, id);
        break;
    case Dimension::Type::Unsigned64:
        e.u64 = buf.getFieldAs<uint64_t>(d, id);
        break;
    case Dimension::Type::None:
        break;
    }
    memcpy(pos, &e, Dimension::size(type));
}

} // anonymous namespace.

void SQLiteWriter::writeTile(PointBuffer const& buffer)
{
    using namespace std;

    boost::uint8_t* point_data(0);
    boost::uint32_t point_data_length(0);
    boost::uint32_t schema_byte_size(0);

    size_t outbufSize = m_pointSize * buffer.size();
    std::unique_ptr<char> outbuf(new char[outbufSize]);
    char *pos = outbuf.get();
    
    for (PointId id = 0; id < buffer.size(); ++id)
    {
        auto ti = m_types.begin();
        for (auto di = m_dims.begin(); di != m_dims.end(); ++di, ++ti)
        {
            fillBuf(buffer, pos, *di, *ti, id);
            pos += Dimension::size(*ti);
        }
        if (id % 100 == 0)
            m_callback->invoke(id);
    }
    
    records rs;
    row r;

    boost::uint32_t precision(9);    
    pdal::Bounds<double> b = buffer.calculateBounds(true);
    std::string bounds = b.toWKT(precision); // polygons are only 2d, not cubes

    std::string box = b.toBox(precision, 3);
    log()->get(LogLevel::Debug3) << "extent: " << bounds << std::endl;
    log()->get(LogLevel::Debug3) << "bbox: " << box << std::endl;
  
    r.push_back(column(m_obj_id));
    r.push_back(column(m_block_id));
    r.push_back(column(buffer.size()));
    r.push_back(blob(outbuf.get(), outbufSize));
    r.push_back(column(bounds));
    r.push_back(column(m_srid));
    r.push_back(column(box));
    rs.push_back(r);
    m_session->insert(m_block_insert_query.str(), rs);
    m_block_id++;

}

}
}
} // namespaces
