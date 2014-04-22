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

#include <pdal/drivers/sqlite/Writer.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/FileUtils.hpp>

#include <boost/algorithm/string.hpp>

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


Writer::Writer(Stage& prevStage, const Options& options)
    : pdal::Writer(prevStage, options)
    , m_session(0)
    , m_block_statement(0)
    , m_doCreateIndex(false)
    , m_bounds(Bounds<double>())
    , m_sdo_pc_is_initialized(false)
{}


void Writer::initialize()
{
    pdal::Writer::initialize();

    std::string connection =
        getOptions().getValueOrDefault<std::string>("connection", "");
    if (!connection.size())
    {
        throw sqlite_driver_error("unable to connect to database, "
            "no connection string was given!");
    }

    try
    {
        m_session = new ::soci::session(::soci::sqlite3, connection);
        log()->get(logDEBUG) << "Connected to database" << std::endl;

    }
    catch (::soci::soci_error const& e)
    {
        std::stringstream oss;
        oss << "Unable to connect to database with error '" << e.what() << "'";
        throw pdal_error(oss.str());
    }
    m_session->set_log_stream(&(log()->get(logDEBUG2)));
}


Options Writer::getDefaultOptions()
{
    return Options();
}


void Writer::writeBegin(boost::uint64_t /*targetNumPointsToWrite*/)
{
    using namespace std;

    string block_table =
        getOptions().getValueOrThrow<string>("block_table");
    string cloud_table =
        getOptions().getValueOrThrow<string>("cloud_table");
    string cloud_column =
        getOptions().getValueOrDefault<string>("cloud_column", "id");

    m_block_insert_query << "INSERT INTO " <<
        boost::to_lower_copy(block_table) << " ("<<
        boost::to_lower_copy(cloud_column) <<
        ", block_id, num_points, points, extent, bbox) VALUES (" <<
        " :obj_id, :block_id, :num_points, decode(:hex, 'hex'), "
        "ST_Force_2D(ST_GeometryFromText(:extent,:srid)), :bbox)";

    m_session->begin();

    bool bHaveBlockTable = CheckTableExists(block_table);
    bool bHaveCloudTable = CheckTableExists(cloud_table);

    if (getOptions().getValueOrDefault<bool>("overwrite", true))
    {
        if (bHaveBlockTable)
        {
            DeleteBlockTable(cloud_table, cloud_column, block_table);
            bHaveBlockTable = false;
        }
        if (bHaveCloudTable)
        {
            DeleteCloudTable(cloud_table, cloud_column);
            bHaveCloudTable = false;
        }
    }

    string pre_sql = getOptions().getValueOrDefault<string>("pre_sql", "");
    if (pre_sql.size())
    {
        string sql = FileUtils::readFileAsString(pre_sql);
        if (!sql.size())
        {
            // if there was no file to read because the data in pre_sql was
            // actually the sql code the user wanted to run instead of the
            // filename to open, we'll use that instead.
            sql = pre_sql;
        }
        m_session->once << sql;
    }

    if (!bHaveCloudTable)
    {
        CreateCloudTable(cloud_table,
            getOptions().getValueOrDefault<uint32_t>("srid", 4326));
    }

    if (!bHaveBlockTable)
    {
        m_doCreateIndex = true;
        CreateBlockTable(block_table,
            getOptions().getValueOrDefault<uint32_t>("srid", 4326));
    }
}


bool Writer::CheckTableExists(std::string const& name)
{
    std::ostringstream oss;

    oss << "SELECT name FROM sqlite_master WHERE type = \"table\"";
    log()->get(logDEBUG) << "checking for " << name <<
        " existence ... " << std::endl;

    ::soci::rowset<std::string> rs = (m_session->prepare << oss.str());

    std::ostringstream debug;
    for (auto it = rs.begin(); it != rs.end(); ++it)
    {
        debug << ", " << *it;
        if (boost::iequals(*it, name))
        {
            log()->get(logDEBUG) << "it exists!" << std::endl;
            return true;
        }
    }
    log()->get(logDEBUG) << debug.str();
    log()->get(logDEBUG) << " -- '" << name << "' not found." << std::endl;
    return false;
}


void Writer::CreateBlockTable(std::string const& name, boost::uint32_t srid)
{
    std::ostringstream oss;

    std::string cloud_column =
        getOptions().getValueOrDefault<std::string>("cloud_column", "id");
    std::string cloud_table =
        getOptions().getValueOrThrow<std::string>("cloud_table");

    oss << "CREATE TABLE " << boost::to_lower_copy(name)
        << "(" << boost::to_lower_copy(cloud_column)  <<
        " INTEGER REFERENCES " << boost::to_lower_copy(cloud_table)  <<
        "," << " block_id INTEGER," << " num_points INTEGER," <<
        " points bytea," << " bbox box3d " << ")";

    m_session->once << oss.str();
    oss.str("");
    {
        oss << "SELECT AddGeometryColumn('" << boost::to_lower_copy(name)
            << "'," << "'extent'" << ","
            << srid << ", 'POLYGON', 'XY')";
        m_session->once << oss.str();
    }
}

void Writer::DeleteBlockTable(std::string const& cloud_table_name,
    std::string const& cloud_column_name, std::string const& block_table_name)
{
    std::ostringstream oss;

    // Delete all the items from the table first
    oss << "DELETE FROM " << block_table_name;
    m_session->once << oss.str();
    oss.str("");

    // Drop the table's dependencies
    // We need to clean up the geometry column before dropping the table
    oss << "SELECT DropGeometryColumn('" <<
        boost::to_lower_copy(block_table_name) << "', 'extent')";
    m_session->once << oss.str();
    oss.str("");

    oss << "DROP TABLE " << boost::to_lower_copy(block_table_name);
    m_session->once << oss.str();
}


void Writer::CreateCloudTable(std::string const& name, boost::uint32_t srid)
{
    std::ostringstream oss;

    ::soci::sqlite3_session_backend* backend =
        static_cast< ::soci::sqlite3_session_backend*>(
            m_session->get_backend());
        
    int did_enable = false;
    did_enable = sqlite3_enable_load_extension(
        static_cast<sqlite_api::sqlite3*>(backend->conn_), 1);
    if (did_enable == SQLITE_ERROR)
        throw sqlite_driver_error("Unable to enable extensions on "
            "sqlite backend -- can't enable spatialite");

    oss << "SELECT load_extension('libspatialite.dylib')";
    m_session->once << oss.str();
    oss.str("");

    oss << "SELECT InitSpatialMetadata()";
    m_session->once << oss.str();
    oss.str("");

    std::string cloud_column =
        getOptions().getValueOrDefault<std::string>("cloud_column", "id");
    oss << "CREATE TABLE " << boost::to_lower_copy(name) << " (" <<
        boost::to_lower_copy(cloud_column) <<
        " INTEGER PRIMARY KEY AUTOINCREMENT," << " schema TEXT," <<
        " block_table varchar(64)" << ")";

    m_session->once << oss.str();
    bool is3d = getOptions().getValueOrDefault<bool>("is3d", false);
    boost::uint32_t nDim = 2;

    oss.str("");
    oss << "SELECT AddGeometryColumn('" << boost::to_lower_copy(name) <<
        "'," << "'extent'" << "," << srid << ", 'POLYGON', 'XY')";
    m_session->once << oss.str();
}

void Writer::DeleteCloudTable(std::string const& cloud_table_name,
                              std::string const& cloud_column_name)
{
    std::ostringstream oss;

    // Delete all the items from the table first
    oss << "DELETE FROM " << cloud_table_name;
    m_session->once << oss.str();
    oss.str("");

    // Go drop the table
    // We need to clean up the geometry column before dropping the table

    try
    {
        oss << "SELECT DropGeometryColumn('" <<
            boost::to_lower_copy(cloud_table_name) << "', 'extent')";
        m_session->once << oss.str();
    }
    catch (::soci::soci_error const& e)
    {}
    oss.str("");

    try
    {
        oss << "DROP TABLE " << boost::to_lower_copy(cloud_table_name);
        m_session->once << oss.str();
    }
    catch (::soci::soci_error const& e)
    {}

    try
    {
        oss.str("");
        oss << "DROP SEQUENCE " <<
            boost::to_lower_copy(cloud_table_name)<<"_id_seq";
        m_session->once << oss.str();
    }
    catch (::soci::soci_error const& e)
    {}
}


void Writer::CreateIndexes(std::string const& table_name,
    std::string const& spatial_column_name, bool is3d, bool isBlockTable)
{
    std::ostringstream oss;
    std::ostringstream index_name_ss;

    index_name_ss << table_name << "_cloud_idx";
    std::string index_name = index_name_ss.str().substr(0,29);

    // Spatial indexes
    oss << "SELECT CreateSpatialIndex('"<< boost::to_lower_copy(table_name) <<
        "', 'extent')";
    m_session->once << oss.str();
}


Schema Writer::getPackedSchema(Schema const& schema) const
{
    schema::index_by_index const& idx =
        schema.getDimensions().get<schema::index>();
    log()->get(logDEBUG3) << "Packing ignored dimension from PointBuffer " <<
        std::endl;

    uint32_t position(0);
    pdal::Schema clean_schema;
    schema::index_by_index::size_type i(0);
    for (i = 0; i < idx.size(); ++i)
    {
        if (!idx[i].isIgnored())
        {

            Dimension d(idx[i]);
            d.setPosition(position);

            // Wipe off parent/child relationships if we're ignoring
            // same-named dimensions
            d.setParent(boost::uuids::nil_uuid());
            clean_schema.appendDimension(d);
            position++;
        }
    }
    return clean_schema;
}


std::string Writer::loadGeometryWKT(std::string const& filename_or_wkt) const
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


bool Writer::IsValidGeometryWKT(std::string const& input) const
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


void Writer::CreateSDOEntry(std::string const& block_table, uint32_t srid,
    pdal::Bounds<double> bounds, bool is3d)
{
    std::ostringstream oss;
    oss.setf(std::ios_base::fixed, std::ios_base::floatfield);
    uint32_t precision =
        getOptions().getValueOrDefault<uint32_t>("stream_output_precision", 8);
    oss.precision(precision);

    std::ostringstream s_srid;
    if (srid == 0)
        s_srid << "NULL";
    else
        s_srid << srid;

    double tolerance = 0.05;
    pdal::Bounds<double> e = bounds;
    if (srid)
    {
        SpatialReference ref;
        ref.setFromUserInput("EPSG:"+s_srid.str());
        if (ref.isGeographic())
        {
            // FIXME: This should be overrideable
            e.setMinimum(0, -180.0);
            e.setMaximum(0, 180.0);
            e.setMinimum(1, -90.0);
            e.setMaximum(1, 90.0);
            e.setMinimum(2, 0.0);
            e.setMaximum(2, 20000.0);

            tolerance = 0.0005;
        }
    }


    oss <<  "INSERT INTO user_sdo_geom_metadata VALUES ('" << block_table <<
        "','extent', MDSYS.SDO_DIM_ARRAY(";

    oss << "MDSYS.SDO_DIM_ELEMENT('X', " << e.getMinimum(0) << "," <<
        e.getMaximum(0) <<"," << tolerance << "),MDSYS.SDO_DIM_ELEMENT('Y', " <<
        e.getMinimum(1) << "," << e.getMaximum(1) <<"," << tolerance << ")";

    if (is3d)
    {
        oss << "," << "MDSYS.SDO_DIM_ELEMENT('Z', "<< e.getMinimum(2) << "," <<
            e.getMaximum(2) << "," << tolerance << ")";
    }
    oss << ")," << s_srid.str() << ")";
    m_session->once << oss.str();
}


void Writer::writeEnd(boost::uint64_t /*actualNumPointsWritten*/)
{
    if (m_doCreateIndex)
    {
        std::string block_table_name =
            getOptions().getValueOrThrow<std::string>("block_table");
        std::string cloud_table_name =
            getOptions().getValueOrThrow<std::string>("cloud_table");
        boost::uint32_t srid =
            getOptions().getValueOrThrow<boost::uint32_t>("srid");
        bool is3d = getOptions().getValueOrDefault<bool>("is3d", false);

        CreateIndexes(block_table_name, "extent", is3d);
    }

    m_session->commit();
}


void Writer::writeBufferBegin(PointBuffer const& data)
{
    if (!m_sdo_pc_is_initialized)
    {
        CreateCloud(data.getSchema());
        m_sdo_pc_is_initialized = true;
    }
}


void Writer::CreateCloud(Schema const& buffer_schema)
{
    using namespace std;

    string cloud_table = getOptions().getValueOrThrow<string>("cloud_table");
    string block_table = getOptions().getValueOrThrow<string>("block_table");

    ostringstream oss;

    pdal::Schema output_schema(buffer_schema);
    bool pack =
        getOptions().getValueOrDefault<bool>("pack_ignored_fields", true);

    if (pack)
        output_schema = getPackedSchema(buffer_schema);

    string bounds = getOptions().getValueOrDefault<string>(
        "cloud_boundary_wkt", "");
    if (bounds.size())
    {
        log()->get(logDEBUG2) << "have cloud_boundary_wkt of size " <<
            bounds.size() << std::endl;
        bounds = loadGeometryWKT(bounds);
    }

    string cloud_column =
        getOptions().getValueOrDefault<string>("cloud_column", "id");

    oss << "INSERT INTO " << boost::to_lower_copy(cloud_table) << "(" <<
        " block_table, schema) VALUES ('" <<
        boost::to_lower_copy(block_table) << "',:xml) ";
    string xml = pdal::Schema::to_xml(output_schema);
    m_session->once << oss.str(), ::soci::use(xml);
    oss.str("");

    long id;
    oss << "select last_insert_rowid()";
    m_session->once << oss.str(), ::soci::into(id);
    oss.str("");

    log()->get(logDEBUG) << "Point cloud id was " << id << std::endl;
    try
    {
        Option& pc_id = getOptions().getOptionByRef("pc_id");
        pc_id.setValue(id);
    }
    catch (pdal::option_not_found&)
    {
        Option pc_id("pc_id", id, "Point Cloud Id");
        getOptions().add(pc_id);
    }

    if (bounds.size())
    {
        uint32_t srid =
            getOptions().getValueOrDefault<uint32_t>("srid", 4326);
        bool is3d = getOptions().getValueOrDefault<bool>("is3d", false);
        string force =  "ST_Force_2D";

        oss << "UPDATE " << boost::to_lower_copy(cloud_table) <<
            " SET extent="<< force <<
            "(ST_GeometryFromText(:wkt,:srid)) where " <<
            boost::to_lower_copy(cloud_column) <<"=:id";

        m_session->once << oss.str(), ::soci::use(bounds, "wkt"),
            ::soci::use(srid,"srid"), ::soci::use(id, "id");
    }
}


boost::uint32_t Writer::writeBuffer(const PointBuffer& buffer)
{
    boost::uint32_t numPoints = buffer.getNumPoints();
    WriteBlock(buffer);
    return numPoints;
}


bool Writer::WriteBlock(PointBuffer const& buffer)
{
    using namespace std;

    boost::uint8_t* point_data;
    boost::uint32_t point_data_length;
    boost::uint32_t schema_byte_size;

    bool pack =
        getOptions().getValueOrDefault<bool>("pack_ignored_fields", true);
    if (pack)
        PackPointData(buffer, &point_data, point_data_length, schema_byte_size);
    else
    {
        point_data = buffer.getData(0);
        point_data_length =
            buffer.getSchema().getByteSize() * buffer.getNumPoints();
    }

    string block_table = getOptions().getValueOrThrow<string>("block_table");

    // // Pluck the block id out of the first point in the buffer
    pdal::Schema const& schema = buffer.getSchema();
    Dimension const& blockDim = schema.getDimension("BlockID");

    m_block_id  = buffer.getFieldAs<boost::int32_t>(blockDim, 0, false);
    m_obj_id = getOptions().getValueOrThrow<boost::int32_t>("pc_id");
    m_num_points = static_cast<boost::int64_t>(buffer.getNumPoints());

    return true;
}


void Writer::PackPointData(PointBuffer const& buffer,
    boost::uint8_t** point_data, boost::uint32_t& point_data_len,
    boost::uint32_t& schema_byte_size)
{
    // Creates a new buffer that has the ignored dimensions removed from
    // it.

    schema::index_by_index const& idx =
        buffer.getSchema().getDimensions().get<schema::index>();

    schema_byte_size = 0;
    schema::index_by_index::size_type i(0);
    for (i = 0; i < idx.size(); ++i)
        if (!idx[i].isIgnored())
            schema_byte_size = schema_byte_size + idx[i].getByteSize();

    log()->get(logDEBUG) << "Packed schema byte size " <<
        schema_byte_size << std::endl;

    point_data_len = buffer.getNumPoints() * schema_byte_size;
    *point_data = new boost::uint8_t[point_data_len];

    boost::uint8_t* current_position = *point_data;

    for (boost::uint32_t i = 0; i < buffer.getNumPoints(); ++i)
    {
        boost::uint8_t* data = buffer.getData(i);
        for (boost::uint32_t d = 0; d < idx.size(); ++d)
        {
            if (! idx[d].isIgnored())
            {
                memcpy(current_position, data, idx[d].getByteSize());
                current_position = current_position+idx[d].getByteSize();
            }
            data = data + idx[d].getByteSize();
        }
    }
}

}
}
} // namespaces
