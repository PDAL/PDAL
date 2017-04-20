/******************************************************************************
* Copyright (c) 2016, Oscar Martinez-Rubi, o.rubi@esciencecenter.nl
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

#include "PgFlatWriter.hpp"

#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/XMLSchema.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/portable_endian.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/util/Utils.hpp>

#if defined(__linux__) || defined(__CYGWIN__) || defined(__FreeBSD_kernel__) || defined(__GNU__)

#   include <endian.h>

#elif defined(__APPLE__)

#   include <libkern/OSByteOrder.h>

#   define htobe16(x) OSSwapHostToBigInt16(x)
#   define htole16(x) OSSwapHostToLittleInt16(x)
#   define be16toh(x) OSSwapBigToHostInt16(x)
#   define le16toh(x) OSSwapLittleToHostInt16(x)

#   define htobe32(x) OSSwapHostToBigInt32(x)
#   define htole32(x) OSSwapHostToLittleInt32(x)
#   define be32toh(x) OSSwapBigToHostInt32(x)
#   define le32toh(x) OSSwapLittleToHostInt32(x)

#   define htobe64(x) OSSwapHostToBigInt64(x)
#   define htole64(x) OSSwapHostToLittleInt64(x)
#   define be64toh(x) OSSwapBigToHostInt64(x)
#   define le64toh(x) OSSwapLittleToHostInt64(x)

#   define __BYTE_ORDER    BYTE_ORDER
#   define __BIG_ENDIAN    BIG_ENDIAN
#   define __LITTLE_ENDIAN LITTLE_ENDIAN
#   define __PDP_ENDIAN    PDP_ENDIAN

#elif defined(__OpenBSD__)

#   include <sys/endian.h>

#elif defined(__NetBSD__) || defined(__FreeBSD__) || defined(__DragonFly__)

#   include <sys/endian.h>

#   define be16toh(x) betoh16(x)
#   define le16toh(x) letoh16(x)

#   define be32toh(x) betoh32(x)
#   define le32toh(x) letoh32(x)

#   define be64toh(x) betoh64(x)
#   define le64toh(x) letoh64(x)

#elif defined(__WINDOWS__)

#   include <winsock2.h>
#   include <sys/param.h>

#   if BYTE_ORDER == LITTLE_ENDIAN

#       define htobe16(x) htons(x)
#       define htole16(x) (x)
#       define be16toh(x) ntohs(x)
#       define le16toh(x) (x)

#       define htobe32(x) htonl(x)
#       define htole32(x) (x)
#       define be32toh(x) ntohl(x)
#       define le32toh(x) (x)

#       define htobe64(x) htonll(x)
#       define htole64(x) (x)
#       define be64toh(x) ntohll(x)
#       define le64toh(x) (x)

#   elif BYTE_ORDER == BIG_ENDIAN

        /* that would be xbox 360 */
#       define htobe16(x) (x)
#       define htole16(x) __builtin_bswap16(x)
#       define be16toh(x) (x)
#       define le16toh(x) __builtin_bswap16(x)

#       define htobe32(x) (x)
#       define htole32(x) __builtin_bswap32(x)
#       define be32toh(x) (x)
#       define le32toh(x) __builtin_bswap32(x)

#       define htobe64(x) (x)
#       define htole64(x) __builtin_bswap64(x)
#       define be64toh(x) (x)
#       define le64toh(x) __builtin_bswap64(x)

#   else

#       error byte order not supported

#   endif

#   define __BYTE_ORDER    BYTE_ORDER
#   define __BIG_ENDIAN    BIG_ENDIAN
#   define __LITTLE_ENDIAN LITTLE_ENDIAN
#   define __PDP_ENDIAN    PDP_ENDIAN

#else

#   error platform not supported

#endif

#ifndef _BSD_SOURCE
#define _BSD_SOURCE
#endif

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "writers.pgflatpointcloud",
    "Write points to PostgreSQL flat table output",
    "http://pdal.io/stages/writers.pgflatpointcloud.html" );

CREATE_SHARED_PLUGIN(1, 0, PgFlatWriter, Writer, s_info)

std::string PgFlatWriter::getName() const { return s_info.name; }

static const int CHUNK_SIZE = 10000;

unsigned int unity = 1;
#define is_littleEndian() (*(unsigned char *)&unity) // will return 1 if little endian, otherwise 0

uint64_t bigEndian_double(double a)
{
    uint64_t b;
    unsigned char *src = (unsigned char *)&a,
    *dst = (unsigned char *)&b;

    if (is_littleEndian())
    {
        dst[0] = src[7];
        dst[1] = src[6];
        dst[2] = src[5];
        dst[3] = src[4];
        dst[4] = src[3];
        dst[5] = src[2];
        dst[6] = src[1];
        dst[7] = src[0];
    }
    else
        b = *(uint64_t *)&a;

        return b;
}


PgFlatWriter::PgFlatWriter()
    : m_session(0)
    , m_overwrite(true)
{}


PgFlatWriter::~PgFlatWriter()
{
    if (m_session)
        PQfinish(m_session);
}


void PgFlatWriter::addArgs(ProgramArgs& args)
{
    DbWriter::doAddArgs(args);
    args.add("connection", "Connection string", m_connection).setPositional();
    args.add("table", "Table name", m_table_name);
    args.add("schema", "Schema name", m_schema_name);
    args.add("overwrite", "Whether data should be overwritten", m_overwrite,
        true);
    args.add("pre_sql", "SQL to execute before query", m_pre_sql);
    args.add("post_sql", "SQL to execute after query", m_post_sql);
}


void PgFlatWriter::initialize()
{
    m_session = pg_connect(m_connection);
}


void PgFlatWriter::writeInit()
{
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

    // Create the table!
    if (! bHaveTable)
    {
        CreateTable(m_schema_name, m_table_name);
    }
}

void PgFlatWriter::write(const PointViewPtr view)
{
    writeInit();
    writeTile(view);
}


void PgFlatWriter::done(PointTableRef /*table*/)
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


void PgFlatWriter::DeleteTable(std::string const& schema_name,
                         std::string const& table_name)
{
    std::ostringstream stmt;
    std::ostringstream name;

    stmt << "DROP TABLE IF EXISTS ";

    if (schema_name.size())
    {
        name << schema_name << ".";
    }
    name << table_name;
    stmt << pg_quote_identifier(name.str());


    pg_execute(m_session, stmt.str());
}

bool PgFlatWriter::CheckTableExists(std::string const& name)
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

void PgFlatWriter::CreateTable(std::string const& schema_name,
    std::string const& table_name)
{
    std::ostringstream oss;
    oss << "CREATE TABLE ";
    if (schema_name.size())
        oss << pg_quote_identifier(schema_name) << ".";
    oss << pg_quote_identifier(table_name);

    oss << " (";
    XMLDimList m_dims = dbDimTypes();
    for (auto di = m_dims.begin(); di != m_dims.end(); ++di)
    {
        if (di != m_dims.begin())
             oss << ", ";

        std::string dimtype = "integer";
        if (di->m_name == "X" || di->m_name == "Y" || di->m_name == "Z" || di->m_name == "GpsTime")
             dimtype = "double precision";

        oss << di->m_name << " " << dimtype;
    }
    oss << ")";

    pg_execute(m_session, oss.str());
}

void PgFlatWriter::writeTile(const PointViewPtr view){
  XMLDimList m_dims = dbDimTypes();

  PGresult *res;
  std::string copyCommand = "COPY " + pg_quote_identifier(m_table_name) + " FROM STDIN with binary";
  res=PQexec(m_session,copyCommand.c_str());
  if(PQresultStatus(res) != PGRES_COPY_IN){
      std::cout << "ERROR: copy in not ok";
      return;
  }

  const std::string headerStr = "PGCOPY\n\377\r\n\0";
  int i1T = 0, i2T = 0;
  uint32_t i1 = htonl(i1T);
  uint32_t i2 = htonl(i2T);

  pg_put_copy_data(m_session, headerStr.c_str(), 11);
  pg_put_copy_data(m_session, &i1, sizeof(uint32_t));
  pg_put_copy_data(m_session, &i2, sizeof(uint32_t));

  uint16_t hT = m_dims.size();
  const uint16_t row_h = htons(hT);
  uint32_t size;
  size = sizeof(double);
  const uint32_t row_vardSize = htonl(size);
  size = sizeof(uint32_t);
  const uint32_t row_varSize = htonl(size);

  unsigned long long int vardL, varL;

  for (PointId idx = 0; idx < view->size(); ++idx){
    pg_put_copy_data(m_session, &row_h, 2);
    for (auto di = m_dims.begin(); di != m_dims.end(); ++di){
      if (di->m_name == "X" || di->m_name == "Y" || di->m_name == "Z" || di->m_name == "GpsTime"){
        pg_put_copy_data(m_session, &row_vardSize, sizeof(uint32_t));
        vardL = bigEndian_double(view->getFieldAs<double>(di->m_dimType.m_id, idx));
        pg_put_copy_data(m_session, &vardL, sizeof(double));
      }else{
        pg_put_copy_data(m_session, &row_varSize, sizeof(uint32_t));
        varL = htonl(view->getFieldAs<int>(di->m_dimType.m_id, idx));
        pg_put_copy_data(m_session, &varL, sizeof(uint32_t));
      }
    }
  }

  short endT = -1;
  short end = htons(endT);
  pg_put_copy_data(m_session, &end, sizeof(end));

  if(PQputCopyEnd(m_session,NULL) == 1){
    PGresult *res = PQgetResult(m_session);
    if(PQresultStatus(res) != PGRES_COMMAND_OK){
      std::cout<<PQerrorMessage(m_session);
    }
  }else{
    std::cout<<PQerrorMessage(m_session);
  }
}
} // namespace pdal
