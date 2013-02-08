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

#include <pdal/drivers/soci/Writer.hpp>
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

#ifdef USE_PDAL_PLUGIN_SOCI
MAKE_WRITER_CREATOR(sociWriter, pdal::drivers::soci::Writer)
CREATE_WRITER_PLUGIN(soci, pdal::drivers::soci::Writer)
#endif


namespace pdal
{
namespace drivers
{
namespace soci
{




Writer::Writer(Stage& prevStage, const Options& options)
    : pdal::Writer(prevStage, options)
    , m_session(0)
    , m_type(DATABASE_UNKNOWN)
    , m_doCreateIndex(false)
    , m_bounds(Bounds<double>())
    , m_sdo_pc_is_initialized(false)

{

    return;
}


Writer::~Writer()
{
    return;
}


void Writer::initialize()
{
    pdal::Writer::initialize();
    
    std::string connection = getOptions().getValueOrDefault<std::string>("connection", "");
    if (!connection.size())
    {
        throw soci_driver_error("unable to connect to database, no connection string was given!");
    }
    
    m_type = getDatabaseConnectionType(getOptions().getValueOrThrow<std::string>("type"));
    if (m_type == DATABASE_UNKNOWN)
    {
        std::stringstream oss;
        oss << "Database connection type '" << getOptions().getValueOrThrow<std::string>("type") << "' is unknown or not configured";
        throw soci_driver_error(oss.str());
    }
    try
    {
        if (m_type == DATABASE_POSTGRESQL)
            m_session = new ::soci::session(::soci::postgresql, connection);
        
        log()->get(logDEBUG) << "Connected to database" << std::endl;
        
    } catch (::soci::soci_error const& e)
    {
        std::stringstream oss;
        oss << "Unable to connect to database with error '" << e.what() << "'";
        throw pdal_error(oss.str());
    }
    
    m_session->set_log_stream(&(log()->get(logDEBUG2)));
    return;
}




Options Writer::getDefaultOptions()
{
    Options options;


    return options;
}


void Writer::writeBegin(boost::uint64_t /*targetNumPointsToWrite*/)
{
    std::string block_table = getOptions().getValueOrThrow<std::string>("block_table");
    std::string cloud_table = getOptions().getValueOrThrow<std::string>("cloud_table");
    std::string cloud_column = getOptions().getValueOrDefault<std::string>("cloud_column", "id");
    
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
    
    std::string pre_sql = getOptions().getValueOrDefault<std::string>("pre_sql", "");
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
        m_session->once << sql;
    }
    
    if (!bHaveCloudTable)
    {
        CreateCloudTable(cloud_table, getOptions().getValueOrDefault<boost::uint32_t>("srid", 4326));        
    }
    
    if (!bHaveBlockTable)
    {
        m_doCreateIndex = true;
        CreateBlockTable(block_table, getOptions().getValueOrDefault<boost::uint32_t>("srid", 4326));
    }
        
    return;
}

bool Writer::CheckTableExists(std::string const& name)
{

    std::ostringstream oss;
    
    if (m_type == DATABASE_ORACLE)
        oss << "select table_name from user_tables";
    else if (m_type == DATABASE_POSTGRESQL)
        oss << "SELECT tablename FROM pg_tables";

    log()->get(logDEBUG) << "checking for " << name << " existence ... " << std::endl;

    ::soci::rowset<std::string> rs = (m_session->prepare << oss.str());

    std::ostringstream debug;
    for (::soci::rowset<std::string>::const_iterator it = rs.begin(); it != rs.end(); ++it)
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
    
    if (m_type == DATABASE_ORACLE) 
    {
        // We just create a new block table as a copy of 
        // the SDO_PC_BLK_TYPE
        oss << "CREATE TABLE " << name << " AS SELECT * FROM MDSYS.SDO_PC_BLK_TABLE";
        m_session->once << oss.str();
        oss.str("");
    }
    else if (m_type == DATABASE_POSTGRESQL)
    {
        std::string cloud_column = getOptions().getValueOrDefault<std::string>("cloud_column", "id");
        std::string cloud_table = getOptions().getValueOrThrow<std::string>("cloud_table");
        
        oss << "CREATE TABLE " << boost::to_lower_copy(name) 
            << "(" << boost::to_lower_copy(cloud_column)  << " INTEGER REFERENCES " << boost::to_lower_copy(cloud_table)  <<","
            // << " obj_id INTEGER,"
            << " block_id INTEGER,"
            << " num_points INTEGER,"
            << " points bytea"
            << ")";

        m_session->once << oss.str();
        oss.str("");
        
        {
            bool is3d = getOptions().getValueOrDefault<bool>("is3d", false);
            boost::uint32_t nDim = is3d ? 3 : 2;

            oss << "SELECT AddGeometryColumn('', '" << boost::to_lower_copy(name) 
                << "'," << "'extent'" << "," 
                << srid << ", 'POLYGON', " << nDim << ")";
            m_session->once << oss.str();
            oss.str("");
        }
    }
}

void Writer::DeleteBlockTable(std::string const& cloud_table_name,
                              std::string const& cloud_column_name, 
                              std::string const& block_table_name)
{
    std::ostringstream oss;

    // Delete all the items from the table first
    oss << "DELETE FROM " << block_table_name;
    m_session->once << oss.str();
    oss.str("");

    // Drop the table's dependencies 
    if (m_type == DATABASE_ORACLE)
    {
        // These need to be uppercase to satisfy the PLSQL function
        oss << "declare\n"
            "begin \n"
            "  mdsys.sdo_pc_pkg.drop_dependencies('"
            << boost::to_upper_copy(cloud_table_name) <<
            "', '"
            << boost::to_upper_copy(cloud_column_name) <<
            "'); end;";
        m_session->once << oss.str();
        oss.str("");
    }
    
    // Go drop the table
    if (m_type == DATABASE_ORACLE)
    {
        // We need to clean up the geometry column before dropping the table
        // Oracle upper cases the table name when inserting it in the
        // USER_SDO_GEOM_METADATA.  
        oss << "DELETE FROM USER_SDO_GEOM_METADATA WHERE TABLE_NAME='" << boost::to_upper_copy(block_table_name) << "'" ;
        m_session->once << oss.str();
        oss.str("");
                
        oss << "DROP TABLE " << block_table_name;    
        m_session->once << oss.str();
        oss.str("");

    } else if (m_type == DATABASE_POSTGRESQL)
    {
        // We need to clean up the geometry column before dropping the table
        oss << "SELECT DropGeometryColumn('" << boost::to_lower_copy(block_table_name) << "', 'extent')";    
        m_session->once << oss.str();
        oss.str("");

        oss << "DROP TABLE " << boost::to_lower_copy(block_table_name);    
        m_session->once << oss.str();
        oss.str("");
    }

}


void Writer::CreateCloudTable(std::string const& name, boost::uint32_t srid)
{
    std::ostringstream oss;
    

    if (m_type == DATABASE_POSTGRESQL)
    {
        oss << "CREATE SEQUENCE " << boost::to_lower_copy(name)<<"_id_seq";    
        m_session->once << oss.str();
        oss.str("");
        
        std::string cloud_column = getOptions().getValueOrDefault<std::string>("cloud_column", "id");
        oss << "CREATE TABLE " << boost::to_lower_copy(name) 
            << " (" << boost::to_lower_copy(cloud_column) << " INTEGER DEFAULT NEXTVAL('"  << boost::to_lower_copy(name)<<"_id_seq"  <<"'),"
            << " schema XML,"
            << " block_table varchar(64),"
            << " PRIMARY KEY ("<< boost::to_lower_copy(cloud_column) <<")"
            << ")";

        m_session->once << oss.str();
        oss.str("");
        {
            bool is3d = getOptions().getValueOrDefault<bool>("is3d", false);
            boost::uint32_t nDim = is3d ? 3 : 2;
            
            oss << "SELECT AddGeometryColumn('', '" << boost::to_lower_copy(name) 
                << "'," << "'extent'" << "," 
                << srid << ", 'POLYGON', " << nDim << ")";
            m_session->once << oss.str();
            oss.str("");
        }
    }
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
    if (m_type == DATABASE_ORACLE)
    {

        try
        {
            // We need to clean up the geometry column before dropping the table
            // Oracle upper cases the table name when inserting it in the
            // USER_SDO_GEOM_METADATA.  
            oss << "DELETE FROM USER_SDO_GEOM_METADATA WHERE TABLE_NAME='" << boost::to_upper_copy(cloud_table_name) << "'" ;
            m_session->once << oss.str();   
            oss.str("");
        } catch (::soci::soci_error const& e)
        {
            
        }

        oss.str("");
                
        try
        {
            oss << "DROP TABLE " << cloud_table_name;    
            m_session->once << oss.str();
            oss.str("");
        } catch (::soci::soci_error const& e)
        {
            
        }
        oss.str("");

    } else if (m_type == DATABASE_POSTGRESQL)
    {
        // We need to clean up the geometry column before dropping the table

        try
        {
            oss << "SELECT DropGeometryColumn('" << boost::to_lower_copy(cloud_table_name) << "', 'extent')";    
            m_session->once << oss.str();       
        } catch (::soci::soci_error const& e)
        {
            
        }
        oss.str("");


        try
        {
            oss << "DROP TABLE " << boost::to_lower_copy(cloud_table_name);    
            m_session->once << oss.str();
        } catch (::soci::soci_error const& e)
        {
            
        }
        oss.str("");


        try
        {
            oss << "DROP SEQUENCE " << boost::to_lower_copy(cloud_table_name)<<"_id_seq";    
            m_session->once << oss.str();       
        } catch (::soci::soci_error const& e)
        {
            
        }
        oss.str("");
        
    }

}


void Writer::CreateIndexes( std::string const& table_name, 
                            std::string const& spatial_column_name, 
                            bool is3d,
                            bool isBlockTable )
{
    std::ostringstream oss;

    std::ostringstream index_name_ss;
    index_name_ss << table_name << "_cloud_idx";
    std::string index_name = index_name_ss.str().substr(0,29);
    
    // Spatial indexes
    if (m_type == DATABASE_ORACLE)
    {
        oss << "CREATE INDEX "<< index_name << " on "
            << table_name << "(" << spatial_column_name 
            << ") INDEXTYPE IS MDSYS.SPATIAL_INDEX";
        if (is3d)
        {
            oss <<" PARAMETERS('sdo_indx_dims=3')";
        }            
        m_session->once << oss.str();
        oss.str("");        
    } else if (m_type == DATABASE_POSTGRESQL)
    {
        oss << "CREATE INDEX "<< index_name << " on "
            << boost::to_lower_copy(table_name) << " USING GIST ("<< boost::to_lower_copy(spatial_column_name) << ")";
        m_session->once << oss.str();
        oss.str("");        
    }

    // Primary key
    
    if (isBlockTable)
    {
        index_name_ss.str("");
        index_name_ss <<  table_name <<"_objectid_idx";
        index_name = index_name_ss.str().substr(0,29);

        std::string cloud_column = getOptions().getValueOrDefault<std::string>("cloud_column", "id");

        oss << "ALTER TABLE "<< table_name <<  " ADD CONSTRAINT "<< index_name <<  
            "  PRIMARY KEY ("<<boost::to_lower_copy(cloud_column) <<", block_id)";
        if (m_type == DATABASE_ORACLE)
        {
            oss <<" ENABLE VALIDATE";
        }

        m_session->once << oss.str();
        
    }
    else
    {


    }
}

Schema Writer::getPackedSchema( Schema const& schema) const
{
    schema::index_by_index const& idx = schema.getDimensions().get<schema::index>();
    log()->get(logDEBUG3) << "Packing ignored dimension from PointBuffer " << std::endl;

    boost::uint32_t position(0);
    
    pdal::Schema clean_schema;
    schema::index_by_index::size_type i(0);
    for (i = 0; i < idx.size(); ++i)
    {
        if (! idx[i].isIgnored())
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

    if (!filename_or_wkt.empty())
    {
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
    if (e != 0) return false;

    return true;
#else

    throw pdal_error("GDAL support not available for WKT validation");
#endif
}

void Writer::CreateSDOEntry(std::string const& block_table, 
                            boost::uint32_t srid, 
                            pdal::Bounds<double> bounds,
                            bool is3d)
{
    std::ostringstream oss;
    oss.setf(std::ios_base::fixed, std::ios_base::floatfield);
    boost::uint32_t precision = getOptions().getValueOrDefault<boost::uint32_t>("stream_output_precision", 8);
    oss.precision(precision);

    std::ostringstream s_srid;

    if (srid == 0)
    {
        s_srid << "NULL";
    }
    else
    {
        s_srid << srid;
    }

    double tolerance = 0.05;


    pdal::Bounds<double> e = bounds;
    
    if (srid)
    {
        SpatialReference ref;
        ref.setFromUserInput("EPSG:"+s_srid.str());
        if (ref.isGeographic())
        {
            // FIXME: This should be overrideable
            e.setMinimum(0,-180.0);
            e.setMaximum(0,180.0);
            e.setMinimum(1,-90.0);
            e.setMaximum(1,90.0);
            e.setMinimum(2,0.0);
            e.setMaximum(2,20000.0);

            tolerance = 0.0005;
        }
    }


    oss <<  "INSERT INTO user_sdo_geom_metadata VALUES ('" << block_table <<
        "','extent', MDSYS.SDO_DIM_ARRAY(";

    oss << "MDSYS.SDO_DIM_ELEMENT('X', " << e.getMinimum(0) << "," << e.getMaximum(0) <<"," << tolerance << "),"
        "MDSYS.SDO_DIM_ELEMENT('Y', " << e.getMinimum(1) << "," << e.getMaximum(1) <<"," << tolerance << ")";

    if (is3d)
    {
        oss << ",";
        oss <<"MDSYS.SDO_DIM_ELEMENT('Z', "<< e.getMinimum(2) << "," << e.getMaximum(2) << "," << tolerance << ")";
    }
    oss << ")," << s_srid.str() << ")";

    m_session->once << oss.str();

    oss.str("");

}

void Writer::writeEnd(boost::uint64_t /*actualNumPointsWritten*/)
{
    
    if (m_doCreateIndex)
    {
        std::string block_table_name = getOptions().getValueOrThrow<std::string>("block_table");
        std::string cloud_table_name = getOptions().getValueOrThrow<std::string>("cloud_table");
        boost::uint32_t srid = getOptions().getValueOrThrow<boost::uint32_t>("srid");
        bool is3d = getOptions().getValueOrDefault<bool>("is3d", false);
        
        CreateIndexes(block_table_name, "extent", is3d);
        if (m_type == DATABASE_POSTGRESQL)
            CreateIndexes(cloud_table_name, "extent", is3d, false);
    }
    
    if (m_type == DATABASE_ORACLE)
    {
        std::string block_table_name = getOptions().getValueOrThrow<std::string>("block_table");
        boost::uint32_t srid = getOptions().getValueOrThrow<boost::uint32_t>("srid");
        bool is3d = getOptions().getValueOrDefault<bool>("is3d", false);
        CreateSDOEntry( block_table_name,
                        srid,
                        m_bounds,
                        is3d);
    }
    
    m_session->commit();
    return;
}

void Writer::writeBufferBegin(PointBuffer const& data)
{
    if (m_sdo_pc_is_initialized) return;

    CreateCloud(data.getSchema());
    m_sdo_pc_is_initialized = true;

    return;

}

void Writer::CreateCloud(Schema const& buffer_schema)
{
    std::string cloud_table = getOptions().getValueOrThrow<std::string>("cloud_table");
    std::string block_table = getOptions().getValueOrThrow<std::string>("block_table");
    
    std::ostringstream oss;
    
    pdal::Schema output_schema(buffer_schema);
    bool pack = getOptions().getValueOrDefault<bool>("pack_ignored_fields", true);
    if (pack)
    {
        output_schema = getPackedSchema(buffer_schema);
    }
    
    std::string bounds = getOptions().getValueOrDefault<std::string>("cloud_boundary_wkt", "");
    if (bounds.size())
    {
        log()->get(logDEBUG2) << "have cloud_boundary_wkt of size " << bounds.size() << std::endl;
        bounds = loadGeometryWKT(bounds);
        
    }
    
    if (m_type == DATABASE_POSTGRESQL)
    {
        // strk: create table tabref( ref regclass );
        // [11:32am] strk: insert into tabref values ('geometry_columns');
        // [11:33am] strk: select ref from tableref;
        // [11:33am] strk: select ref::oid from tableref;
        // [11:33am] strk: create table dropme (a int);
        // [11:33am] strk: insert into tabref values ('dropme');
        // [11:34am] strk: select ref, ref::oid from tableref;
        // [11:34am] strk: drop table dropme;
        // [11:34am] strk: -- oops, is this what you want ?
        // [11:34am] strk: select ref, ref::oid from tableref;
        // [11:34am] strk: create table dropme (a int);
        // [11:34am] strk: select ref, ref::oid from tableref; -- no, it's not back
        // [11:35am] hobu: thanks
        // [11:35am] hobu: not quite a pointer, but close enough
        // [11:36am] strk: it's really a pointer, just maybe a too "hard" one
        // [11:36am] strk: not a soft pointer
        // [11:36am] strk: it's not a string, is an oid
        // [11:36am] strk: looks like a string with the default output
        // [11:36am] strk: try changing your search_path
        // [11:36am] strk: the canonical text output should also change to include the pointed-to path
        
        long id;
        std::string cloud_column = getOptions().getValueOrDefault<std::string>("cloud_column", "id");        
        oss << "INSERT INTO " << boost::to_lower_copy(cloud_table)  
			<< "(" << boost::to_lower_copy(cloud_column) 
			<< ", block_table, schema) VALUES (DEFAULT,'" 
			<< boost::to_lower_copy(block_table)  
			<< "',:xml) RETURNING " << boost::to_lower_copy(cloud_column);
        std::string xml = pdal::Schema::to_xml(output_schema);
        m_session->once << oss.str(), ::soci::use(xml), ::soci::into(id);
        oss.str("");

        // 
        // int id;
        // oss << "SELECT CURRVAL('"<< boost::to_lower_copy(cloud_table)  <<"_id_seq')";
        // (m_session->once << oss.str(), ::soci::into(id));
        // oss.str("");
        
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
            boost::uint32_t srid = getOptions().getValueOrDefault<boost::uint32_t>("srid", 4326);
            bool is3d = getOptions().getValueOrDefault<bool>("is3d", false);
            std::string force = is3d ? "ST_Force_3D" : "ST_Force_2D";
            
            oss << "UPDATE " 
				<< boost::to_lower_copy(cloud_table) 
				<< " SET extent="<< force 
				<< "(ST_GeometryFromText(:wkt,:srid)) where "
				<< boost::to_lower_copy(cloud_column) <<"=:id";
            
			m_session->once << oss.str(), ::soci::use(bounds, "wkt"), ::soci::use(srid,"srid"), ::soci::use(id, "id");
            
        }
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

    boost::uint8_t* point_data;
    boost::uint32_t point_data_length;
    boost::uint32_t schema_byte_size;
    
    
    bool pack = getOptions().getValueOrDefault<bool>("pack_ignored_fields", true);
    if (pack)
        PackPointData(buffer, &point_data, point_data_length, schema_byte_size);
    else
    {
        point_data = buffer.getData(0);
        point_data_length = buffer.getSchema().getByteSize() * buffer.getNumPoints();
    }
    
    std::string block_table = getOptions().getValueOrThrow<std::string>("block_table");

    // // Pluck the block id out of the first point in the buffer
    pdal::Schema const& schema = buffer.getSchema();
    Dimension const& blockDim = schema.getDimension("BlockID");
    
    boost::int32_t blk_id  = buffer.getField<boost::int32_t>(blockDim, 0);    
    boost::int32_t obj_id = getOptions().getValueOrThrow<boost::int32_t>("pc_id");
    boost::int64_t num_points = static_cast<boost::int64_t>(buffer.getNumPoints());
    if (m_type == DATABASE_POSTGRESQL)
    {
        std::string cloud_column = getOptions().getValueOrDefault<std::string>("cloud_column", "id");
        bool is3d = getOptions().getValueOrDefault<bool>("is3d", false);
        std::string force = is3d ? "ST_Force_3D" : "ST_Force_2D";
        
        std::stringstream oss;
        oss << "INSERT INTO " << boost::to_lower_copy(block_table) 
            << " ("<< boost::to_lower_copy(cloud_column) <<", block_id, num_points, points, extent) VALUES (" 
            << " :obj_id, :block_id, :num_points, decode(:hex, 'hex'), " << force << "(ST_GeometryFromText(:extent,:srid)))";

        
        std::vector<boost::uint8_t> block_data;
        for (boost::uint32_t i = 0; i < point_data_length; ++i )
        {
            block_data.push_back(point_data[i]);
        }
        std::string hex = Utils::binary_to_hex_string(block_data);
        //std::cout << "hex: " << hex.substr(0, 30) << std::endl;
        boost::uint32_t srid = getOptions().getValueOrDefault<boost::uint32_t>("srid", 4326);
        
        std::string extent = buffer.calculateBounds(is3d).toWKT();
        //log()->get(logDEBUG) << "extent: " << extent << std::endl;
        ::soci::statement st = (m_session->prepare << oss.str(), ::soci::use(obj_id, "obj_id"), ::soci::use(blk_id, "block_id"), ::soci::use(num_points, "num_points"), ::soci::use(hex,"hex"), ::soci::use(extent, "extent"), ::soci::use(srid, "srid"));
        st.execute(true);
        oss.str("");
        
        // ::soci::blob blob(*m_session);
        // ::soci::indicator ind;
        // oss << "SELECT POINTS from "<< boost::to_lower_copy(block_table) << " WHERE block_id =:blk_id and obj_id = :obj_id";
        // m_session->once << oss.str(), ::soci::into(blob, ind), ::soci::use(blk_id), ::soci::use(obj_id);
        // assert(blob.get_len() == 0);
        // 
        // blob.write(0, (const char*)point_data, point_data_length);

        // m_session->commit();
    }
    // bool bUsePartition = m_block_table_partition_column.size() != 0;
    // 
    // // Pluck the block id out of the first point in the buffer
    // pdal::Schema const& schema = buffer.getSchema();
    // Dimension const& blockDim = schema.getDimension("BlockID");
    // 
    // boost::int32_t block_id  = buffer.getField<boost::int32_t>(blockDim, 0);
    // 
    // std::ostringstream oss;
    // std::ostringstream partition;
    // 
    // if (bUsePartition)
    // {
    //     partition << "," << m_block_table_partition_column;
    // }
    // 
    // oss << "INSERT INTO "<< m_block_table_name <<
    //     "(OBJ_ID, BLK_ID, NUM_POINTS, POINTS,   "
    //     "PCBLK_MIN_RES, BLK_EXTENT, PCBLK_MAX_RES, NUM_UNSORTED_POINTS, PT_SORT_DIM";
    // if (bUsePartition)
    //     oss << partition.str();
    // oss << ") "
    //     "VALUES ( :1, :2, :3, :4, 1, mdsys.sdo_geometry(:5, :6, null,:7, :8)"
    //     ", 1, 0, 1";
    // if (bUsePartition)
    //     oss << ", :9";
    // 
    // oss <<")";
    // 
    // // TODO: If gotdata == false below, this memory probably leaks --mloskot
    // OCILobLocator** locator =(OCILobLocator**) VSIMalloc(sizeof(OCILobLocator*) * 1);
    // 
    // Statement statement = Statement(m_connection->CreateStatement(oss.str().c_str()));
    // 
    // 
    // long* p_pc_id = (long*) malloc(1 * sizeof(long));
    // p_pc_id[0] = m_pc_id;
    // 
    // long* p_result_id = (long*) malloc(1 * sizeof(long));
    // p_result_id[0] = (long)block_id;
    // 
    // long* p_num_points = (long*) malloc(1 * sizeof(long));
    // p_num_points[0] = (long)buffer.getNumPoints();
    // 
    // // std::cout << "point count on write: " << buffer.getNumPoints() << std::endl;
    // 
    // 
    // // :1
    // statement->Bind(&m_pc_id);
    // 
    // // :2
    // statement->Bind(p_result_id);
    // 
    // // :3
    // statement->Bind(p_num_points);
    // 
    // // :4
    // statement->Define(locator, 1);
    // 
    // 
    // // std::vector<liblas::uint8_t> data;
    // // bool gotdata = GetResultData(result, reader, data, 3);
    // // if (! gotdata) throw std::runtime_error("unable to fetch point data byte array");
    // // boost::uint8_t* point_data = buffer.getData(0);
    // boost::uint8_t* point_data;
    // boost::uint32_t point_data_length;
    // boost::uint32_t schema_byte_size;
    // 
    // bool pack = getOptions().getValueOrDefault<bool>("pack_ignored_fields", true);
    // if (pack)
    //     PackPointData(buffer, &point_data, point_data_length, schema_byte_size);
    // else
    // {
    //     point_data = buffer.getData(0);
    //     point_data_length = buffer.getSchema().getByteSize() * buffer.getNumPoints();
    // }
    // 
    // // statement->Bind((char*)point_data,(long)(buffer.getSchema().getByteSize()*buffer.getNumPoints()));
    // statement->Bind((char*)point_data,(long)(point_data_length));
    // 
    // // :5
    // long* p_gtype = (long*) malloc(1 * sizeof(long));
    // p_gtype[0] = m_gtype;
    // 
    // statement->Bind(p_gtype);
    // 
    // // :6
    // long* p_srid  = 0;
    // 
    // 
    // if (m_srid != 0)
    // {
    //     p_srid = (long*) malloc(1 * sizeof(long));
    //     p_srid[0] = m_srid;
    // }
    // statement->Bind(p_srid);
    // 
    // // :7
    // OCIArray* sdo_elem_info=0;
    // m_connection->CreateType(&sdo_elem_info, m_connection->GetElemInfoType());
    // SetElements(statement, sdo_elem_info);
    // statement->Bind(&sdo_elem_info, m_connection->GetElemInfoType());
    // 
    // // :8
    // OCIArray* sdo_ordinates=0;
    // m_connection->CreateType(&sdo_ordinates, m_connection->GetOrdinateType());
    // 
    // // x0, x1, y0, y1, z0, z1, bUse3d
    // pdal::Bounds<double> bounds = CalculateBounds(buffer);
    // SetOrdinates(statement, sdo_ordinates, bounds);
    // statement->Bind(&sdo_ordinates, m_connection->GetOrdinateType());
    // 
    // // :9
    // long* p_partition_d = 0;
    // if (bUsePartition)
    // {
    //     p_partition_d = (long*) malloc(1 * sizeof(long));
    //     p_partition_d[0] = m_block_table_partition_value;
    //     statement->Bind(p_partition_d);
    // }
    // 
    // try
    // {
    //     statement->Execute();
    // }
    // catch (std::runtime_error const& e)
    // {
    //     std::ostringstream oss;
    //     oss << "Failed to insert block # into '" << m_block_table_name << "' table. Does the table exist? "  << std::endl << e.what() << std::endl;
    //     throw std::runtime_error(oss.str());
    // }
    // 
    // oss.str("");
    // 
    // 
    // OWStatement::Free(locator, 1);
    // 
    // if (p_pc_id != 0) free(p_pc_id);
    // if (p_result_id != 0) free(p_result_id);
    // if (p_num_points != 0) free(p_num_points);
    // if (p_gtype != 0) free(p_gtype);
    // if (p_srid != 0) free(p_srid);
    // if (p_partition_d != 0) free(p_partition_d);
    // 
    // m_connection->DestroyType(&sdo_elem_info);
    // m_connection->DestroyType(&sdo_ordinates);




    return true;
}


void Writer::PackPointData(PointBuffer const& buffer,
                           boost::uint8_t** point_data,
                           boost::uint32_t& point_data_len,
                           boost::uint32_t& schema_byte_size)

{
    // Creates a new buffer that has the ignored dimensions removed from 
    // it.
    
    schema::index_by_index const& idx = buffer.getSchema().getDimensions().get<schema::index>();

    schema_byte_size = 0;
    schema::index_by_index::size_type i(0);
    for (i = 0; i < idx.size(); ++i)
    {
        if (! idx[i].isIgnored())
            schema_byte_size = schema_byte_size+idx[i].getByteSize();
    }
    
    log()->get(logDEBUG) << "Packed schema byte size " << schema_byte_size << std::endl;;

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
