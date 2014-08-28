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

#include <boost/format.hpp>

#include <pdal/drivers/pgpointcloud/Writer.hpp>

#include <pdal/PointBuffer.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/Endian.hpp>
#include <pdal/XMLSchema.hpp>

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

Writer::Writer(const Options& options)
    : pdal::Writer(options)
    , m_session(0)
    , m_schema_name("")
    , m_table_name("")
    , m_column_name("")
    , m_patch_compression_type(CompressionType::None)
    , m_patch_capacity(400)
    , m_srid(0)
    , m_pcid(0)
    , m_have_postgis(false)
    , m_create_index(true)
    , m_overwrite(true)
    , m_schema_is_initialized(false)
{
}


Writer::~Writer()
{
    if (m_session)
        PQfinish(m_session);
}


void Writer::processOptions(const Options& options)
{
    // If we don't know the table name, we're SOL
    m_table_name = options.getValueOrThrow<std::string>("table");

    // Schema and column name can be defaulted safely
    m_column_name = options.getValueOrDefault<std::string>("column", "pa");
    m_schema_name = options.getValueOrDefault<std::string>("schema", "");
    //
    // Read compression type and turn into an integer
    std::string compression_str =
        options.getValueOrDefault<std::string>("compression", "dimensional");
    m_patch_compression_type = getCompressionType(compression_str);

    // Connection string needs to exist and actually work
    m_connection = options.getValueOrThrow<std::string>("connection");

    // Read other preferences
    m_overwrite = options.getValueOrDefault<bool>("overwrite", true);
    m_patch_capacity =
        options.getValueOrDefault<boost::uint32_t>("capacity", 400);
    m_srid = options.getValueOrDefault<boost::uint32_t>("srid", 4326);
    m_pcid = options.getValueOrDefault<boost::uint32_t>("pcid", 0);
    m_pack = options.getValueOrDefault<bool>("pack_ignored_fields", true);
    m_pre_sql = getOptions().getValueOrDefault<std::string>("pre_sql", "");
}


void Writer::ready(PointContext ctx)
{
    m_pointSize = 0;
    m_dims = ctx.dims();
    for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
    {
        Dimension::Type::Enum type = Dimension::defaultType(*di);
        if (type == Dimension::Type::None)
            type = Dimension::Type::Float;
        m_types.push_back(type);
        m_pointSize += Dimension::size(type);
    }
}


//
// Called from PDAL core during start-up. Do everything
// here that you are going to absolutely require later.
// Optional things you can defer or attempt to initialize
// here.
//
void Writer::initialize()
{
    m_session = pg_connect(m_connection);  
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

void Writer::writeInit()
{
    if (m_schema_is_initialized)
        return;

    // Start up the database connection
    pg_begin(m_session);

    // Pre-SQL can be *either* a SQL file to execute, *or* a SQL statement
    // to execute. We find out which one here.
    if (m_pre_sql.size())
    {
        std::string sql = FileUtils::readFileAsString(m_pre_sql);
        if (!sql.size())
        {
            // if there was no file to read because the data in pre_sql was
            // actually the sql code the user wanted to run instead of the
            // filename to open, we'll use that instead.
            sql = m_pre_sql;
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
    m_pcid = SetupSchema(m_srid);

    // Create the table!
    if (! bHaveTable)
    {
        CreateTable(m_schema_name, m_table_name, m_column_name, m_pcid);
    }

    m_schema_is_initialized = true;
}

void Writer::write(const PointBuffer& buffer)
{
    writeInit();
    writeTile(buffer);
}


void Writer::done(PointContext ctx)
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


uint32_t Writer::SetupSchema(uint32_t srid)
{

    // If the user has specified a PCID they want to use,
    // does it exist in the database?
    std::ostringstream oss;
    long schema_count;
    if (m_pcid)
    {
        oss << "SELECT Count(pcid) FROM pointcloud_formats WHERE pcid = " <<
            m_pcid;
        char *count_str = pg_query_once(m_session, oss.str());
        if (!count_str)
            throw pdal_error("Unable to count pcid's in table "
                "`pointcloud_formats`");
        schema_count = atoi(count_str);
        free(count_str);
        oss.str("");
        if (schema_count == 0)
        {
            oss << "requested PCID '" << m_pcid <<
                "' does not exist in POINTCLOUD_FORMATS";
            throw pdal_error(oss.str());
        }
        return m_pcid;
    }

    // Do we have any existing schemas in the POINTCLOUD_FORMATS table?
    boost::uint32_t pcid = 0;
    oss << "SELECT Count(pcid) FROM pointcloud_formats";
    char *schema_count_str = pg_query_once(m_session, oss.str());
    if (!schema_count_str)
        throw pdal_error("Unable to count pcid's in table "
            "`pointcloud_formats`");
    schema_count = atoi(schema_count_str);
    free(schema_count_str);
    oss.str("");

    // Create an XML output schema.
    schema::Writer writer(m_dims, m_types);
    std::string xml = writer.getXML();

    // Do any of the existing schemas match the one we want to use?
    if (schema_count > 0)
    {
        PGresult *result = pg_query_result(m_session,
            "SELECT pcid, schema FROM pointcloud_formats");
        for (int i = 0; i < PQntuples(result); ++i)
        {
            char *pcid_str = PQgetvalue(result, i, 0);
            char *schema_str = PQgetvalue(result, i, 1);

            if (xml == schema_str)
            {
                m_pcid = atoi(pcid_str);
                PQclear(result);
                return m_pcid;
            }
        }
        PQclear(result);
    }

    if (schema_count == 0)
        pcid = 1;
    else
    {
        char *pcid_str = pg_query_once(m_session,
                "SELECT Max(pcid)+1 AS pcid FROM pointcloud_formats");
        if (!pcid_str)
            throw pdal_error("Unable to get the max pcid from "
                "`pointcloud_formats`");
        pcid = atoi(pcid_str);
    }

    std::string compression;
    /* If the writer specifies a compression, we should set that */
    if (m_patch_compression_type == CompressionType::Dimensional)
        compression = "dimensional";
    else if (m_patch_compression_type == CompressionType::Ght)
        compression = "ght";

    Metadata metadata;
    MetadataNode m = metadata.getNode();
    m.add("compression", compression);

    const char* paramValues = xml.c_str();
    oss << "INSERT INTO pointcloud_formats (pcid, srid, schema) "
        "VALUES (" << pcid << "," << srid << ",$1)";
    PGresult *result = PQexecParams(m_session, oss.str().c_str(), 1,
            NULL, &paramValues, NULL, NULL, 0);
    if (PQresultStatus(result) != PGRES_COMMAND_OK)
        throw pdal_error(PQresultErrorMessage(result));
    PQclear(result);
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
    log()->get(LogLevel::Debug) << "checking for pointcloud existence ... " <<
        std::endl;

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
    log()->get(LogLevel::Debug) << "checking for PostGIS existence ... " <<
        std::endl;

    std::string q = "SELECT PostGIS_Version()";
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
    oss << "SELECT count(*) FROM pg_tables WHERE tablename ILIKE '" <<
        name << "'";

    log()->get(LogLevel::Debug) << "checking for table '" << name <<
        "' existence ... " << std::endl;

    char *count_str = pg_query_once(m_session, oss.str());
    if (!count_str)
        throw pdal_error("Unable to check for the existence of `pg_table`");
    int count = atoi(count_str);
    free(count_str);

    if (count == 1)
        return true;
    else if (count > 1)
        log()->get(LogLevel::Debug) << "found more than 1 table named '" <<
            name << "'" << std::endl;
    return false;
}


void Writer::CreateTable(std::string const& schema_name,
    std::string const& table_name, std::string const& column_name,
    boost::uint32_t pcid)
{
    std::ostringstream oss;
    oss << "CREATE TABLE ";
    if (schema_name.size())
        oss << schema_name << ".";
    oss << table_name;
    oss << " (id SERIAL PRIMARY KEY, " << column_name << " PcPatch";
    if (pcid)
        oss << "(" << pcid << ")";
    oss << ")";

    pg_execute(m_session, oss.str());
}


// Make sure you test for the presence of PostGIS before calling this
void Writer::CreateIndex(std::string const& schema_name,
    std::string const& table_name, std::string const& column_name)
{
    std::ostringstream oss;

    oss << "CREATE INDEX ";
    if (schema_name.size())
        oss << schema_name << "_";
    oss << table_name << "_pc_gix";
    oss << " USING GIST (Geometry(" << column_name << "))";

    pg_execute(m_session, oss.str());
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


void Writer::writeTile(PointBuffer const& buffer)
{
    if (buffer.size() > m_patch_capacity)
    {
        std::ostringstream oss;
        oss << "drivers.pgpointcloud.writer buffer size (" << buffer.size() 
            << ") is greater than capacity (" << m_patch_capacity << ")";
        throw pdal_error(oss.str());
    }
    
    size_t outbufSize = m_pointSize * buffer.size();
    std::unique_ptr<char> outbuf(new char[outbufSize]);
    char *pos = outbuf.get();
    size_t clicks = 0;
    size_t interrupt = m_dims.size() * 100;
    
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

    m_callback->invoke(buffer.size());    

    point_count_t num_points = buffer.size();

    /* We are always getting uncompressed bytes off the block_data */
    /* so we always used compression type 0 (uncompressed) in writing our WKB */
    int32_t pcid = m_pcid;
    CompressionType::Enum compression_v = CompressionType::None;
    uint32_t compression = static_cast<uint32_t>(compression_v);

    static char syms[] = "0123456789ABCDEF";
    m_hex.resize(outbufSize * 2);
    for (unsigned i = 0; i != outbufSize; i++)
    {
        m_hex[i * 2]   = syms[((outbuf.get()[i] >> 4) & 0xf)];
        m_hex[i * 2 + 1] = syms[outbuf.get()[i] & 0xf];
    }

    m_insert.clear();
    m_insert.resize(m_hex.size() + 3000);
    
    std::string insert_into("INSERT INTO ");
    std::string values(" (pa) VALUES ('");
    
    std::string::size_type position(0);
    m_insert.insert(position, insert_into);
    position += insert_into.size();
    m_insert.insert(position, m_table_name);
    position += m_table_name.size();

    m_insert.insert(position, values);
    position += values.size();

    std::ostringstream options;
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
    
    m_insert.insert(position, options.str());
    position += options.str().size();
    
    m_insert.insert(position, m_hex);
    position += m_hex.size();
    
    std::string tail("')");
    m_insert.insert(position, tail);

    pg_execute(m_session, m_insert);
}

} // namespace pgpointcloud
} // namespace drivers
} // namespace pdal

