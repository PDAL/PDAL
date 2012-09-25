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
#include <pdal/FileUtils.hpp>

#include <boost/algorithm/string.hpp>

#include <sstream>

#ifdef PDAL_HAVE_GDAL
#include <gdal.h>
#include <ogr_api.h>
#endif
#ifdef USE_PDAL_PLUGIN_SOCI
PDAL_C_START

PDAL_DLL void PDALRegister_soci_text(void* factory)
{
    pdal::StageFactory& f = *(pdal::StageFactory*) factory;
    f.registerWriter(pdal::drivers::soci::Writer::s_getName(), createSociWriter);
}

PDAL_C_END

pdal::Writer* createSociWriter(pdal::Stage& prevStage, const pdal::Options& options)
{
    return new pdal::drivers::soci::Writer(prevStage, options);
}
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
    , m_type(Database_Postgresql)
    , m_doCreateIndex(false)
    , m_bounds(Bounds<double>())

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
    
    std::string connection_type = getOptions().getValueOrDefault<std::string>("type", "postgresql");
    if (boost::iequals(connection_type, "oracle"))
        m_type = Database_Oracle;
    else if (boost::iequals(connection_type, "postgresql"))
        m_type = Database_Postgresql;
    else
        m_type = Database_Unknown;
    
    if (m_type == Database_Unknown)
    {
        std::stringstream oss;
        oss << "Database connection type '" << connection_type << "' is unknown or not configured";
        throw soci_driver_error(oss.str());
    }

    try
    {
        if (m_type == Database_Postgresql)
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




const Options Writer::getDefaultOptions() const
{
    Options options;


    return options;
}


void Writer::writeBegin(boost::uint64_t /*targetNumPointsToWrite*/)
{
    std::string block_table = getOptions().getValueOrThrow<std::string>("block_table");
    std::string cloud_table = getOptions().getValueOrThrow<std::string>("cloud_table");
    std::string cloud_column = getOptions().getValueOrThrow<std::string>("cloud_column");

    bool bHaveOutputTable = BlockTableExists(block_table);

    if (getOptions().getValueOrDefault<bool>("overwrite", true))
    {
        if (bHaveOutputTable)
        {
            DeleteBlockTable(cloud_table, cloud_column, block_table);
            bHaveOutputTable = false;
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
        ::soci::statement st = m_session->prepare << sql;
        st.execute();
    }

    if (!bHaveOutputTable)
    {
        m_doCreateIndex = true;
        CreateBlockTable(block_table, getOptions().getValueOrDefault<boost::uint32_t>("srid", 4326));
    }
        
    return;
}

bool Writer::BlockTableExists(std::string const& name)
{

    std::ostringstream oss;
    
    if (m_type == Database_Oracle)
        oss << "select table_name from user_tables";
    else if (m_type == Database_Postgresql)
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
    
    if (m_type == Database_Oracle) 
    {
        // We just create a new block table as a copy of 
        // the SDO_PC_BLK_TYPE
        oss << "CREATE TABLE " << name << " AS SELECT * FROM MDSYS.SDO_PC_BLK_TABLE";
        ::soci::statement st = m_session->prepare << oss.str();
        st.execute();
        oss.str("");
    }
    else if (m_type == Database_Postgresql)
    {
        oss << "CREATE TABLE " << boost::to_lower_copy(name) 
            << " (obj_id INTEGER,"
            << " blk_id INTEGER,"
            << " num_points INTEGER,"
            << " points BYTEA"
            << ")";

        ::soci::statement create = m_session->prepare << oss.str();
        create.execute();
        oss.str("");
        
        {
            oss << "SELECT AddGeometryColumn('', '" << boost::to_lower_copy(name) 
                << "'," << "'blk_extent'" << "," 
                << srid << ", 'POLYGON', 2)";
            ::soci::statement geom = m_session->prepare << oss.str();
            geom.execute();
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
    ::soci::statement st = m_session->prepare << oss.str();
    st.execute();

    oss.str("");

    // Drop the table's dependencies 
    if (m_type == Database_Oracle)
    {
        // These need to be uppercase to satisfy the PLSQL function
        oss << "declare\n"
            "begin \n"
            "  mdsys.sdo_pc_pkg.drop_dependencies('"
            << boost::to_upper_copy(cloud_table_name) <<
            "', '"
            << boost::to_upper_copy(cloud_column_name) <<
            "'); end;";
        ::soci::statement st = m_session->prepare << oss.str();
        st.execute();
        oss.str("");
    }
    
    // Go drop the table
    if (m_type == Database_Oracle)
    {
        // We need to clean up the geometry column before dropping the table
        // Oracle upper cases the table name when inserting it in the
        // USER_SDO_GEOM_METADATA.  
        oss << "DELETE FROM USER_SDO_GEOM_METADATA WHERE TABLE_NAME='" << boost::to_upper_copy(block_table_name) << "'" ;
        ::soci::statement st = m_session->prepare << oss.str();
        st.execute();
        oss.str("");
                
        oss << "DROP TABLE " << block_table_name;    
        ::soci::statement drop = m_session->prepare << oss.str();
        drop.execute();
        oss.str("");
    } else if (m_type == Database_Postgresql)
    {
        // We need to clean up the geometry column before dropping the table
        oss << "SELECT DropGeometryColumn('" << boost::to_lower_copy(block_table_name) << "', 'blk_extent')";    
        ::soci::statement column = m_session->prepare << oss.str();
        column.execute();
        oss.str("");

        oss << "DROP TABLE " << boost::to_lower_copy(block_table_name);    
        ::soci::statement drop = m_session->prepare << oss.str();
        drop.execute();
        oss.str("");
    }

}

void Writer::CreateIndexes(std::string const& name, bool is3d )
{
    std::ostringstream oss;

    std::ostringstream index_name_ss;
    index_name_ss << name << "_cloud_idx";
    std::string index_name = index_name_ss.str().substr(0,29);
    
    // Spatial indexes
    if (m_type == Database_Oracle)
    {
        oss << "CREATE INDEX "<< index_name << " on "
            << name << "(blk_extent) INDEXTYPE IS MDSYS.SPATIAL_INDEX";
        if (is3d)
        {
            oss <<" PARAMETERS('sdo_indx_dims=3')";
        }            
        ::soci::statement st = m_session->prepare << oss.str();
        st.execute();
        oss.str("");        
    } else if (m_type == Database_Postgresql)
    {
        oss << "CREATE INDEX "<< index_name << " on "
            << boost::to_lower_copy(name) << " USING GIST (blk_extent)";
        ::soci::statement st = m_session->prepare << oss.str();
        st.execute();
        oss.str("");        
        
    }

    // Primary key
    index_name_ss.str("");
    index_name_ss <<  name <<"_objectid_idx";
    index_name = index_name_ss.str().substr(0,29);
    oss << "ALTER TABLE "<< name <<  " ADD CONSTRAINT "<< index_name <<  
        "  PRIMARY KEY (OBJ_ID, BLK_ID)";
    if (m_type == Database_Oracle)
    {
        oss <<" ENABLE VALIDATE";
    }
    
    ::soci::statement primary = m_session->prepare << oss.str();
    primary.execute();
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

std::string Writer::loadWKT(std::string const& filename_or_wkt) const
{
    std::ostringstream wkt_s;

    if (!filename_or_wkt.empty())
    {
        if (!FileUtils::fileExists(filename_or_wkt))
        {
            if (!IsValidWKT(filename_or_wkt))
            {
                std::ostringstream oss;
                oss << "WKT for base_table_boundary_wkt was not valid and '" << filename_or_wkt
                    << "' doesn't exist as a file";
                throw pdal::pdal_error(oss.str());
            }
            wkt_s << filename_or_wkt;
        }
        else
        {
            std::string wkt = FileUtils::readFileAsString(filename_or_wkt);
            if (!IsValidWKT(wkt))
            {
                std::ostringstream oss;
                oss << "WKT for base_table_boundary_wkt was from file '" << filename_or_wkt
                    << "' is not valid";
                throw pdal::pdal_error(oss.str());
            }
            wkt_s << wkt;
        }
    }
    return wkt_s.str();
}

bool Writer::IsValidWKT(std::string const& input) const
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
        "','blk_extent', MDSYS.SDO_DIM_ARRAY(";

    oss << "MDSYS.SDO_DIM_ELEMENT('X', " << e.getMinimum(0) << "," << e.getMaximum(0) <<"," << tolerance << "),"
        "MDSYS.SDO_DIM_ELEMENT('Y', " << e.getMinimum(1) << "," << e.getMaximum(1) <<"," << tolerance << ")";

    if (is3d)
    {
        oss << ",";
        oss <<"MDSYS.SDO_DIM_ELEMENT('Z', "<< e.getMinimum(2) << "," << e.getMaximum(2) << "," << tolerance << ")";
    }
    oss << ")," << s_srid.str() << ")";

    ::soci::statement st = m_session->prepare << oss.str();
    st.execute();

    oss.str("");

}

void Writer::writeEnd(boost::uint64_t /*actualNumPointsWritten*/)
{
    
    if (m_doCreateIndex)
    {
        std::string block_table_name = getOptions().getValueOrThrow<std::string>("block_table");
        boost::uint32_t srid = getOptions().getValueOrThrow<boost::uint32_t>("srid");
        bool is3d = getOptions().getValueOrDefault<bool>("is3d", false);
        
        CreateIndexes(block_table_name, is3d);
    }
    
    if (m_type == Database_Oracle)
    {
        std::string block_table_name = getOptions().getValueOrThrow<std::string>("block_table");
        boost::uint32_t srid = getOptions().getValueOrThrow<boost::uint32_t>("srid");
        bool is3d = getOptions().getValueOrDefault<bool>("is3d", false);
        CreateSDOEntry( block_table_name,
                        srid,
                        m_bounds,
                        is3d);
    }
    return;
}


boost::uint32_t Writer::writeBuffer(const PointBuffer& data)
{

    return data.getNumPoints();
}


}
}
} // namespaces
