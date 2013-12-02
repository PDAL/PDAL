/******************************************************************************
* Copyright (c) 2012, Howard Butler, hobu.inc@gmail.com
* Copyright (c) 2013, Paul Ramsey, pramsey@cleverelephant.ca
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

#include <pdal/drivers/pgpointcloud/Writer.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/Endian.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>

#include <sstream>

#ifdef USE_PDAL_PLUGIN_PGPOINTCLOUD
MAKE_WRITER_CREATOR(pgpointcloudWriter, pdal::drivers::pgpointcloud::Writer)
CREATE_WRITER_PLUGIN(pgpointcloud, pdal::drivers::pgpointcloud::Writer)
#endif

// TO DO:
// - change INSERT into COPY
//
// - PCID / Schema consistency. If a PCID is specified,
// must it be consistent with the buffer schema? Or should
// the writer shove the data into the database schema as best
// it can?
//
// - Load information table. Should PDAL write into a metadata
// table information about each load? If so, how to distinguish
// between loads? Leave to pre/post SQL?


namespace pdal
{
namespace drivers
{
namespace pgpointcloud
{

Writer::Writer(Stage& prevStage, const Options& options)
    : pdal::Writer(prevStage, options)
    , m_session(0)
    , m_schema_name("")
    , m_table_name("")
    , m_column_name("")
    , m_patch_compression_type(schema::COMPRESSION_NONE)
    , m_patch_capacity(400)
    , m_srid(0)
    , m_pcid(0)
    , m_have_postgis(false)
    , m_create_index(true)
    , m_overwrite(true)
    , m_schema_is_initialized(false)
{


    return;
}


Writer::~Writer()
{
    if (m_session)
        PQfinish(m_session);

    return;
}


//
// Called from PDAL core during start-up. Do everything
// here that you are going to absolutely require later.
// Optional things you can defer or attempt to initialize
// here.
//
void Writer::initialize()
{
    pdal::Writer::initialize();

    // If we don't know the table name, we're SOL
    m_table_name = getOptions().getValueOrThrow<std::string>("table");

    // Schema and column name can be defaulted safely
    m_column_name = getOptions().getValueOrDefault<std::string>("column", "pa");
    m_schema_name = getOptions().getValueOrDefault<std::string>("schema", "");

    // Read compression type and turn into an integer
    std::string compression_str = getOptions().getValueOrDefault<std::string>("compression", "dimensional");
    m_patch_compression_type = getCompressionType(compression_str);

    // Connection string needs to exist and actually work
    std::string connection = getOptions().getValueOrThrow<std::string>("connection");

    // Can we connect, using this string?
    m_session = pg_connect(connection);

    // Read other preferences
    m_overwrite = getOptions().getValueOrDefault<bool>("overwrite", true);
    m_patch_capacity = getOptions().getValueOrDefault<boost::uint32_t>("capacity", 400);
    m_srid = getOptions().getValueOrDefault<boost::uint32_t>("srid", 4326);
    m_pcid = getOptions().getValueOrDefault<boost::uint32_t>("pcid", 0);

    return;
}

//
// Called from somewhere (?) in PDAL core presumably to provide a user-friendly
// means of editing the reader options.
//
Options Writer::getDefaultOptions()
{
    Options options;

    Option table("table", "", "table to write to");
    Option schema("schema", "", "schema table resides in");
    Option column("column", "", "column to write to");
    Option compression("compression", "dimensional", "patch compression format to use (none, dimensional, ght)");
    Option overwrite("overwrite", true, "replace any existing table");
    Option capacity("capacity", 400, "how many points to store in each patch");
    Option srid("srid", 4326, "spatial reference id to store data in");
    Option pcid("pcid", 0, "use this existing pointcloud schema id, if it exists");
    Option pre_sql("pre_sql", "", "before the pipeline runs, read and execute this SQL file, or run this SQL command");
    Option post_sql("post_sql", "", "after the pipeline runs, read and execute this SQL file, or run this SQL command");

    options.add(table);
    options.add(schema);
    options.add(column);
    options.add(compression);
    options.add(overwrite);
    options.add(capacity);
    options.add(srid);
    options.add(pcid);
    options.add(pre_sql);
    options.add(post_sql);

    return options;
}

void Writer::writeBegin(boost::uint64_t /*targetNumPointsToWrite*/)
{}
//
// Called by PDAL core before the start of the writing process, but
// after the initialization. At this point, the machinery is all set
// up and we can apply actions to the target database, like pre-SQL and
// preparing new tables and/or deleting old ones.
//

void Writer::writeBufferBegin(PointBuffer const& data)
{
    if (m_schema_is_initialized) return;

    // Start up the database connection
    pg_begin(m_session);

    // Pre-SQL can be *either* a SQL file to execute, *or* a SQL statement
    // to execute. We find out which one here.
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
        pg_execute(m_session, sql);
    }

    bool bHaveTable = CheckTableExists(m_table_name);

    // Apply the over-write preference if it is set
    if (m_overwrite && bHaveTable)
    {
        DeleteTable(m_schema_name, m_table_name);
        bHaveTable = false;
    }

    // Read or create a PCID for our new table
    m_pcid = SetupSchema(data.getSchema(), m_srid);

    // Create the table!
    if (! bHaveTable)
    {
        CreateTable(m_schema_name, m_table_name, m_column_name, m_pcid);
    }


    m_schema_is_initialized = true;

    return;

}

void Writer::writeEnd(boost::uint64_t /*actualNumPointsWritten*/)
{
    if (m_create_index && m_have_postgis)
    {
        CreateIndex(m_schema_name, m_table_name, m_column_name);
    }

    // Post-SQL can be *either* a SQL file to execute, *or* a SQL statement
    // to execute. We find out which one here.
    std::string post_sql = getOptions().getValueOrDefault<std::string>("post_sql", "");
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
        pg_execute(m_session, sql);
    }

    pg_commit(m_session);
    return;
}


boost::uint32_t Writer::SetupSchema(Schema const& buffer_schema, boost::uint32_t srid)
{
    // We strip any ignored dimensions from the schema before creating the table
    pdal::Schema output_schema = buffer_schema.pack();

    // If the user has specified a PCID they want to use,
    // does it exist in the database?
    std::ostringstream oss;
    long schema_count;
    if (m_pcid)
    {
        oss << "SELECT Count(pcid) FROM pointcloud_formats WHERE pcid = " << m_pcid;
        char *count_str = pg_query_once(m_session, oss.str());
        schema_count = atoi(count_str);
        free(count_str);
        oss.str("");
        if (schema_count == 0)
        {
            oss << "requested PCID '" << m_pcid << "' does not exist in POINTCLOUD_FORMATS";
            throw pdal_error(oss.str());
        }
        return m_pcid;
    }

    // Do we have any existing schemas in the POINTCLOUD_FORMATS table?
    boost::uint32_t pcid = 0;
    bool bCreatePCPointSchema = true;
    oss << "SELECT Count(pcid) FROM pointcloud_formats";
    char *schema_count_str = pg_query_once(m_session, oss.str());
    schema_count = atoi(schema_count_str);
    free(schema_count_str);
    oss.str("");

    // Do any of the existing schemas match the one we want to use?
    if (schema_count > 0)
    {
        PGresult *result = pg_query_result(m_session, "SELECT pcid, schema FROM pointcloud_formats");
        for (int i=0; i<PQntuples(result); ++i)
        {
            char *pcid_str = PQgetvalue(result, i, 0);
            char *schema_str = PQgetvalue(result, i, 1);

            if (pdal::Schema::from_xml(schema_str) == output_schema)
            {
                bCreatePCPointSchema = false;
                pcid = atoi(pcid_str);
                break;
            }
        }
        PQclear(result);
    }

    if (bCreatePCPointSchema)
    {
        std::string xml;
        std::string compression;
        char *pcid_str;

        if (schema_count == 0)
        {
            pcid = 1;
        }
        else
        {
            char *pcid_str = pg_query_once(m_session, "SELECT Max(pcid)+1 AS pcid FROM pointcloud_formats");
            pcid = atoi(pcid_str);
        }

        /* If the writer specifies a compression, we should set that */
        if (m_patch_compression_type == schema::COMPRESSION_DIMENSIONAL)
        {
            compression = "dimensional";
        }
        else if (m_patch_compression_type == schema::COMPRESSION_GHT)
        {
            compression = "ght";
        }

        Metadata metadata("compression", compression, "");
        
        log()->get(logDEBUG) << "output_schema: " << output_schema.getByteSize() << std::endl;
        xml = pdal::Schema::to_xml(output_schema, &(metadata.toPTree()));

        const char** paramValues = (const char**)malloc(sizeof(char*));
        paramValues[0] = xml.c_str();

        oss << "INSERT INTO pointcloud_formats (pcid, srid, schema) VALUES (" << pcid << "," << srid << ",$1)";
        PGresult *result = PQexecParams(m_session, oss.str().c_str(), 1, NULL, paramValues, NULL, NULL, 0);
        if (PQresultStatus(result) != PGRES_COMMAND_OK)
        {
            throw pdal_error(PQresultErrorMessage(result));
        }
        PQclear(result);
    }

    m_pcid = pcid;
    return m_pcid;
}


void Writer::DeleteTable(std::string const& schema_name,
                         std::string const& table_name)
{
    std::ostringstream oss;

    oss << "DROP TABLE IF EXISTS ";

    if (schema_name.size())
    {
        oss << schema_name << ".";
    }
    oss << table_name;

    pg_execute(m_session, oss.str());
}

bool Writer::CheckPointCloudExists()
{
    log()->get(logDEBUG) << "checking for pointcloud existence ... " << std::endl;

    std::string q = "SELECT PC_Version()";

    try
    {
        pg_execute(m_session, q);
    }
    catch (pdal_error const &e)
    {
        return false;
    }

    return true;
}

bool Writer::CheckPostGISExists()
{
    std::string q = "SELECT PostGIS_Version()";

    log()->get(logDEBUG) << "checking for PostGIS existence ... " << std::endl;

    try
    {
        pg_execute(m_session, q);
    }
    catch (pdal_error const &e)
    {
        return false;
    }

    return true;
}


bool Writer::CheckTableExists(std::string const& name)
{
    std::ostringstream oss;
    oss << "SELECT count(*) FROM pg_tables WHERE tablename ILIKE '" << name << "'";

    log()->get(logDEBUG) << "checking for table '" << name << "' existence ... " << std::endl;

    char *count_str = pg_query_once(m_session, oss.str());
    int count = atoi(count_str);
    free(count_str);

    if (count == 1)
    {
        return true;
    }
    else if (count > 1)
    {
        log()->get(logDEBUG) << "found more than 1 table named '" << name << "'" << std::endl;
        return false;
    }
    else
    {
        return false;
    }
}


void Writer::CreateTable(std::string const& schema_name,
                         std::string const& table_name,
                         std::string const& column_name,
                         boost::uint32_t pcid)
{
    std::ostringstream oss;
    oss << "CREATE TABLE ";
    if (schema_name.size())
    {
        oss << schema_name << ".";
    }
    oss << table_name;
    oss << " (id SERIAL PRIMARY KEY, " << column_name << " PcPatch";
    if (pcid)
    {
        oss << "(" << pcid << ")";
    }
    oss << ")";

    pg_execute(m_session, oss.str());
}

// Make sure you test for the presence of PostGIS before calling this
void Writer::CreateIndex(std::string const& schema_name,
                         std::string const& table_name,
                         std::string const& column_name)
{
    std::ostringstream oss;

    oss << "CREATE INDEX ";
    if (schema_name.size())
    {
        oss << schema_name << "_";
    }
    oss << table_name << "_pc_gix";
    oss << " USING GIST (Geometry(" << column_name << "))";

    pg_execute(m_session, oss.str());
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

    boost::uint32_t schema_byte_size;

    // PackPointData(buffer, &point_data, point_data_length, schema_byte_size);
    
    PointBuffer output = buffer.pack();
    pointbuffer::PointBufferByteSize  point_data_length = output.getBufferByteLength();
    point_data = output.getData(0);
    // 
    // 
    // 
    // log()->get(logDEBUG) << "point_data_length " <<  output.getSchema().getByteSize() * output.getNumPoints() << std::endl;;
    // log()->get(logDEBUG) << "output.getSchema().getByteSize() " << output.getSchema().getByteSize() << std::endl;;
    // log()->get(logDEBUG) << "output.getNumPoints() " << output.getNumPoints() << std::endl;;
    // log()->get(logDEBUG) << "buffer.getNumPoints() " << buffer.getNumPoints() << std::endl;;
    // 
    // log()->get(logDEBUG) << "buffer.getBufferByteCapacity() " << buffer.getBufferByteCapacity() << std::endl;;
    // log()->get(logDEBUG) << "output.getBufferByteCapacity() " << output.getBufferByteCapacity() << std::endl;;
    // log()->get(logDEBUG) << "buffer.getBufferByteLength() " << buffer.getBufferByteLength() << std::endl;;
    // log()->get(logDEBUG) << "output.getBufferByteLength() " << output.getBufferByteLength() << std::endl;;


    boost::uint32_t num_points = static_cast<boost::uint32_t>(output.getNumPoints());

    if (num_points > m_patch_capacity)
    {
        // error here
    }

    std::vector<boost::uint8_t> block_data;
    block_data.resize(point_data_length);
    std::copy(point_data, point_data+point_data_length, block_data.begin());
    // for (boost::uint32_t i = 0; i < point_data_length; ++i)
    // {
    //     block_data.push_back(point_data[i]);
    // }

    /* We are always getting uncompressed bytes off the block_data */
    /* so we always used compression type 0 (uncompressed) in writing our WKB */
    boost::int32_t pcid = m_pcid;
    schema::CompressionType compression_v = schema::COMPRESSION_NONE;
    boost::uint32_t compression = static_cast<boost::uint32_t>(compression_v);
    
    std::stringstream oss;
    oss << "INSERT INTO " << m_table_name << " (pa) VALUES ('";

    std::stringstream options;
#ifdef BOOST_LITTLE_ENDIAN
    options << boost::format("%02x") % 1;
    SWAP_ENDIANNESS(pcid);
    SWAP_ENDIANNESS(compression);
    SWAP_ENDIANNESS(num_points);
#elif BOOST_BIG_ENDIAN
    options << boost::format("%02x") % 0;
#endif

    options << boost::format("%08x") % pcid;
    options << boost::format("%08x") % compression;
    options << boost::format("%08x") % num_points;

    oss << options.str() << Utils::binary_to_hex_string(block_data);
    oss << "')";

    pg_execute(m_session, oss.str());

    return true;
}



}
}
} // namespaces
