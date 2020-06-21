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

#include "PgWriter.hpp"

#include <pdal/PointView.hpp>
#include <pdal/XMLSchema.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/portable_endian.hpp>
#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

static PluginInfo const s_info
{
    "writers.pgpointcloud",
    "Write points to PostgreSQL pgpointcloud output",
    "http://pdal.io/stages/writers.pgpointcloud.html"
};

CREATE_SHARED_STAGE(PgWriter, s_info)

std::string PgWriter::getName() const { return s_info.name; }

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

PgWriter::PgWriter()
    : m_session(0)
    , m_patch_compression_type(CompressionType::None)
    , m_srid(0)
    , m_pcid(0)
    , m_overwrite(true)
    , m_schema_is_initialized(false)
{}


PgWriter::~PgWriter()
{
    if (m_session)
        PQfinish(m_session);
}


void PgWriter::addArgs(ProgramArgs& args)
{
    DbWriter::doAddArgs(args);
    args.add("connection", "Connection string", m_connection).setPositional();
    args.add("table", "Table name", m_table_name);
    args.add("column", "Column name", m_column_name, "pa");
    args.add("schema", "Schema name", m_schema_name);
    args.add("compression", "Compression type", m_compressionSpec,
        "dimensional");
    args.add("overwrite", "Whether data should be overwritten", m_overwrite);
    args.add("srid", "SRID", m_srid, 4326U);
    args.add("pcid", "PCID", m_pcid);
    args.add("pre_sql", "SQL to execute before query", m_pre_sql);
    args.add("post_sql", "SQL to execute after query", m_post_sql);
}


void PgWriter::initialize()
{
    m_patch_compression_type = getCompressionType(m_compressionSpec);
    m_session = pg_connect(m_connection);
}


void PgWriter::writeInit()
{
    if (m_schema_is_initialized)
        return;

    // Start up the database connection
    pg_begin(m_session);

    // Pre-SQL can be *either* a SQL file to execute, *or* a SQL statement
    // to execute. We find out which one here.
    if (m_pre_sql.size())
    {
        std::string sql = FileUtils::readFileIntoString(m_pre_sql);
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

void PgWriter::write(const PointViewPtr view)
{
    writeInit();
    writeTile(view);
}


void PgWriter::done(PointTableRef /*table*/)
{
    //CreateIndex(m_schema_name, m_table_name, m_column_name);

    if (m_post_sql.size())
    {
        std::string sql = FileUtils::readFileIntoString(m_post_sql);
        if (!sql.size())
        {
            // if there was no file to read because the data in post_sql was
            // actually the sql code the user wanted to run instead of the
            // filename to open, we'll use that instead.
            sql = m_post_sql;
        }
        pg_execute(m_session, sql);
    }

    pg_commit(m_session);
    return;
}


uint32_t PgWriter::SetupSchema(uint32_t srid)
{

    // If the user has specified a PCID they want to use,
    // does it exist in the database?
    std::ostringstream oss;
    long schema_count;
    if (m_pcid)
    {
        oss << "SELECT Count(pcid) FROM pointcloud_formats WHERE pcid = " <<
            m_pcid;
        std::string count_str = pg_query_once(m_session, oss.str());
        if (count_str.empty())
            throwError("Unable to count pcid's in table `pointcloud_formats`");
        schema_count = atoi(count_str.c_str());
        if (schema_count == 0)
            throwError("Requested PCID '" + Utils::toString(m_pcid) +
                "' does not exist in POINTCLOUD_FORMATS");
        return m_pcid;
    }

    // Do we have any existing schemas in the POINTCLOUD_FORMATS table?
    uint32_t pcid = 0;
    oss.clear();
    oss << "SELECT Count(pcid) FROM pointcloud_formats";
    std::string schema_count_str = pg_query_once(m_session, oss.str());
    if (schema_count_str.empty())
        throwError("Unable to count pcid's in table 'pointcloud_formats'.");
    schema_count = atoi(schema_count_str.c_str());
    oss.str("");

    // Create an XML output schema.
    std::string compression;
    /* If the writer specifies a compression, we should set that */
    if (m_patch_compression_type == CompressionType::Dimensional)
        compression = "dimensional";
    else if (m_patch_compression_type == CompressionType::Lazperf)
        compression = "laz";

    Metadata metadata;
    MetadataNode m = metadata.getNode();
    m.add("compression", compression);

    XMLSchema schema(dbDimTypes(), m);
    std::string xml = schema.xml();

    // Do any of the existing schemas match the one we want to use?
    if (schema_count > 0)
    {
        oss.str("");
        oss.clear();
        oss << "SELECT pcid, schema FROM pointcloud_formats WHERE srid = " << srid;
        PGresult *result = pg_query_result(m_session, oss.str());
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
    {
        // Try using the sequence
        // pgpointcloud with the sequence is not yet
        // released. See https://github.com/PDAL/PDAL/issues/1101 for
        // SQL to create this sequence on the pointcloud_formats
        // table.
        std::string have_seq = pg_query_once(m_session,
                "select count(*) from pg_class where relname = 'pointcloud_formats_pcid_sq'");
        int seq_count = atoi(have_seq.c_str());
        if (seq_count)
        {
            // We have the sequence, use its nextval
            std::string pcid_str = pg_query_once(m_session,
                    "SELECT nextval('pointcloud_formats_pcid_sq')");
            if (pcid_str.empty())
                throwError("Unable to select nextval from "
                    "'pointcloud_formats_pcid_seq'.");
            pcid = atoi(pcid_str.c_str());
        }
        else
        {
            // We don't have the sequence installed, all we can do is
            // set to 1 and increment
            pcid = 1;
        }
    }
    else
    {
        std::string pcid_str = pg_query_once(m_session,
                "SELECT Max(pcid)+1 AS pcid FROM pointcloud_formats");
        if (pcid_str.empty())
            throw("Unable to get the max pcid from 'pointcloud_formats'.");
        pcid = atoi(pcid_str.c_str());
    }

    const char* paramValues = xml.c_str();
    oss.str("");
    oss.clear();
    oss << "INSERT INTO pointcloud_formats (pcid, srid, schema) "
        "VALUES (" << pcid << "," << srid << ",$1)";
    PGresult *result = PQexecParams(m_session, oss.str().c_str(), 1,
            NULL, &paramValues, NULL, NULL, 0);
    if (PQresultStatus(result) != PGRES_COMMAND_OK)
        throwError(PQresultErrorMessage(result));
    PQclear(result);
    m_pcid = pcid;
    return m_pcid;
}


void PgWriter::DeleteTable(std::string const& schema_name,
                         std::string const& table_name)
{
    std::ostringstream stmt;
    std::ostringstream name;

    stmt << "DROP TABLE IF EXISTS ";

    if (schema_name.size())
    {
        name << pg_quote_identifier(schema_name) << ".";
    }
    name << pg_quote_identifier(table_name);
    stmt << name.str();

    pg_execute(m_session, stmt.str());
}


bool PgWriter::CheckPointCloudExists()
{
    log()->get(LogLevel::Debug) << "checking for pointcloud existence ... " <<
        std::endl;

    std::string q = "SELECT PC_Version()";
    try
    {
        pg_execute(m_session, q);
    }
    catch (pdal_error const &)
    {
        return false;
    }
    return true;
}


bool PgWriter::CheckPostGISExists()
{
    log()->get(LogLevel::Debug) << "checking for PostGIS existence ... " <<
        std::endl;

    std::string q = "SELECT PostGIS_Version()";
    try
    {
        pg_execute(m_session, q);
    }
    catch (pdal_error const &)
    {
        return false;
    }
    return true;
}


bool PgWriter::CheckTableExists(std::string const& name)
{
    std::ostringstream oss;
    oss << "SELECT count(*) FROM pg_tables WHERE tablename ILIKE '" <<
        name << "'";

    log()->get(LogLevel::Debug) << "checking for table '" << name <<
        "' existence ... " << std::endl;

    std::string count_str = pg_query_once(m_session, oss.str());
    if (count_str.empty())
        throwError("Unable to check for the existence of 'pg_table'.");
    int count = atoi(count_str.c_str());

    if (count == 1)
        return true;
    else if (count > 1)
        log()->get(LogLevel::Debug) << "found more than 1 table named '" <<
            name << "'" << std::endl;
    return false;
}

void PgWriter::CreateTable(std::string const& schema_name,
    std::string const& table_name, std::string const& column_name,
    uint32_t pcid)
{
    std::ostringstream oss;
    oss << "CREATE TABLE ";
    if (schema_name.size())
        oss << pg_quote_identifier(schema_name) << ".";
    oss << pg_quote_identifier(table_name);
    oss << " (id SERIAL PRIMARY KEY, " <<
        pg_quote_identifier(column_name) << " PcPatch";
    if (pcid)
        oss << "(" << pcid << ")";
    oss << ")";

    pg_execute(m_session, oss.str());
}


// Make sure you test for the presence of PostGIS before calling this
void PgWriter::CreateIndex(std::string const& schema_name,
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


void PgWriter::writeTile(const PointViewPtr view)
{
    std::vector<char> storage(packedPointSize());
    std::string hexrep;
    size_t maxHexrepSize = packedPointSize() * view->size() * 2;
    hexrep.reserve(maxHexrepSize);

    m_insert.clear();
    m_insert.reserve(maxHexrepSize + 3000);

    for (PointId idx = 0; idx < view->size(); ++idx)
    {
        size_t size = readPoint(*view.get(), idx, storage.data());

        /* We are always getting uncompressed bytes off the block_data */
        /* so we always used compression type 0 (uncompressed) in writing */
        /* our WKB */
        static char syms[] = "0123456789ABCDEF";
        for (size_t i = 0; i != size; i++)
        {
            hexrep.push_back(syms[((storage[i] >> 4) & 0xf)]);
            hexrep.push_back(syms[storage[i] & 0xf]);
        }
    }

    std::string insert_into("INSERT INTO ");
    std::string values(" (" + pg_quote_identifier(m_column_name) +
                       ") VALUES ('");

    m_insert.append(insert_into);

    if (m_schema_name.size())
    {
        m_insert.append(pg_quote_identifier(m_schema_name));
        m_insert.append(".");
    }

    m_insert.append(pg_quote_identifier(m_table_name));
    m_insert.append(values);

    std::ostringstream options;

    if (view->size() > (std::numeric_limits<uint32_t>::max)())
        throwError("Too many points for tile.");
    uint32_t num_points = htobe32(static_cast<uint32_t>(view->size()));
    int32_t pcid = htobe32(m_pcid);
    CompressionType compression_v = CompressionType::None;
    uint32_t compression = htobe32(static_cast<uint32_t>(compression_v));

#if BYTE_ORDER == LITTLE_ENDIAN
    options << "01";
#elif BYTE_ORDER == BIG_ENDIAN
    options << "00";
#endif

    // needs to be 4 bytes
    options << std::hex << std::setfill('0') << std::setw(8) << pcid;
    // needs to be 4 bytes
    options << std::hex << std::setfill('0') << std::setw(8) << compression;
    // needs to be 4 bytes
    options << std::hex << std::setfill('0') << std::setw(8) << num_points;

    m_insert.append(options.str());
    m_insert.append(hexrep);
    m_insert.append("')");

    pg_execute(m_session, m_insert);
}

} // namespace pdal
