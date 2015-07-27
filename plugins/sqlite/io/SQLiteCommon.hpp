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

#pragma once

#include <pdal/Options.hpp>
#include <pdal/Log.hpp>
#include <pdal/XMLSchema.hpp>
#include <pdal/Compression.hpp>

#include <sqlite3.h>
#include <memory>
#include <sstream>

namespace pdal
{

class column
{
public:

    column() : null(true), blobBuf(0), blobLen(0){};
    template<typename T> column( T v) : null(false), blobBuf(0), blobLen(0)
    {
        data = boost::lexical_cast<std::string>(v);
    }
    column(std::string v) : null(false), blobBuf(0), blobLen(0)
    {
        data = v;
    }

    std::string data;
    bool null;
    std::vector<uint8_t> blobBuf;
    std::size_t blobLen;
};

class blob : public column
{
public:
    blob(const char* buffer, std::size_t size) : column()
    {
        blobBuf.resize(size);
        std::copy(buffer, buffer+size, blobBuf.begin());
        blobLen = size;
        null = false;

    }
};

typedef std::vector<column> row;
typedef std::vector<row> records;

class Patch
{
public:
    Patch() : count(0), remaining(0), m_isCompressed(false), idx(0)
    {}

    point_count_t count;
    point_count_t remaining;

    MetadataNode m_metadata;
    bool m_isCompressed;
    std::string m_compVersion;
    std::vector<unsigned char> buf;
    size_t idx;

    void putBytes(const unsigned char* b, size_t len) {
        while(len --) {
            buf.push_back(*b++);
        }
    }

    void putByte(const unsigned char b) {
        buf.push_back(b);
    }

    unsigned char getByte() {
        return buf[idx++];
    }

    void getBytes(unsigned char *b, int len) {
        for (int i = 0 ; i < len ; i ++) {
            b[i] = getByte();
        }
    }

    void setBytes(const std::vector<uint8_t>& data)
        { buf = data; }

    const std::vector<uint8_t>& getBytes() const
        { return buf; }

    size_t byte_size()
        { return buf.size(); }
};
typedef std::shared_ptr<Patch> PatchPtr;


class SQLite
{
public:
    SQLite(std::string const& connection, LogPtr log)
        : m_log(log)
        , m_connection(connection)
        , m_session(0)
        , m_statement(0)
        , m_position(-1)
    {
        m_log->get(LogLevel::Debug3) << "Setting up config " << std::endl;
        sqlite3_shutdown();
        sqlite3_config(SQLITE_CONFIG_LOG, log_callback, this);
        sqlite3_initialize();
        m_log->get(LogLevel::Debug3) << "Set up config " << std::endl;
        m_log->get(LogLevel::Debug3) << "SQLite version: " << sqlite3_libversion() << std::endl;
    }

    ~SQLite()
    {

        if (m_session)
        {
#ifdef sqlite3_close_v2
            sqlite3_close_v2(m_session);
#else
            sqlite3_close(m_session);
#endif
        }
        sqlite3_shutdown();

    }

    static void log_callback(void *p, int num, char const* msg)
    {
        SQLite* sql = reinterpret_cast<SQLite*>(p);
        sql->log()->get(LogLevel::Debug) << "SQLite code: "
            << num << " msg: '" << msg << "'"
            << std::endl;
    }


    void connect(bool bWrite=false)
    {
        if (!m_connection.size())
        {
            throw pdal_error("Unable to connect to database: empty connection string [SQLite::connect]");
        }

        int flags = SQLITE_OPEN_NOMUTEX;
        if (bWrite)
        {
            m_log->get(LogLevel::Debug3) << "Connecting db for write"<< std::endl;
            flags |= SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE;
        }
        else
        {
            m_log->get(LogLevel::Debug3) << "Connecting db for read"<< std::endl;
            flags |= SQLITE_OPEN_READONLY;
        }

        int status = sqlite3_open_v2(m_connection.c_str(), &m_session, flags, 0);
        if (status != SQLITE_OK)
        {
            error("Unable to open database", "connect");
        }
    }

    void execute(std::string const& sql)
    {
        checkSession();

        m_log->get(LogLevel::Debug3) << "Executing '" << sql <<"'"<< std::endl;

        int status = sqlite3_exec(m_session, sql.c_str(), NULL, NULL, NULL);
        if (status != SQLITE_OK)
        {
            std::ostringstream oss;
            oss << "Database operation failed: "
                << sql;
            error(oss.str(), "execute");
        }
    }

    void begin()
    {
        execute("BEGIN");
    }

    void commit()
    {
        execute("COMMIT");
    }

    // Executes an SQL query statement and provides the returned rows via
    // an iterator.
    //
    // Usage example:
    //   query("SELECT * from TABLE");
    //     do
    //     {
    //       const row* r = get();
    //       if (!r) break ; // no more rows
    //       column const& c = r->at(0); // get 1st column of this row
    //       ... use c.data ...
    //     } while (next());
    //
    void query(std::string const& query)
    {
        checkSession();

        m_position = 0;
        m_columns.clear();
        m_data.clear();
        assert(!m_statement);

        int status;

        m_log->get(LogLevel::Debug3) << "Querying '" << query.c_str() <<"'"<< std::endl;

        char const* tail = 0; // unused;
        status = sqlite3_prepare_v2(m_session,
                                    query.c_str(),
                                    static_cast<int>(query.size()),
                                    &m_statement,
                                    &tail);
        if (status != SQLITE_OK)
        {
            error("query preparation failed", "query");
        }

        int numCols = -1;

        while (status != SQLITE_DONE)
        {
            status = sqlite3_step(m_statement);

            if (SQLITE_ROW == status)
            {
                // only need to set the number of columns once
                if (-1 == numCols)
                {
                    numCols = sqlite3_column_count(m_statement);
                }

                row r;
                for (int v = 0; v < numCols; ++v)
                {

                    if (m_columns.size() != static_cast<std::vector<std::string>::size_type > (numCols))
                    {
                        std::string ccolumnName = Utils::toupper(std::string(sqlite3_column_name(m_statement, v)));
                        const char* coltype = sqlite3_column_decltype(m_statement, v);
                        if (!coltype)
                        {
                            coltype = "unknown";
                        }
                        std::string ccolumnType = Utils::toupper(std::string(coltype));
                        m_columns.insert(std::pair<std::string, int32_t>(ccolumnName, v));
                        m_types.push_back(ccolumnType);
                    }

                    column c;

                    if (sqlite3_column_type(m_statement, v) == SQLITE_BLOB)
                    {
                        int len = sqlite3_column_bytes(m_statement, v);
                        const char* buf = (const char*) sqlite3_column_blob(m_statement, v);
                        c.blobLen = len;
                        c.blobBuf.resize(len);
                        std::copy(buf, buf+len, c.blobBuf.begin());
                    } else if (sqlite3_column_type(m_statement, v) == SQLITE_NULL)
                    {
                        c.null = true;
                    } else
                    {
                        char const* buf =
                            reinterpret_cast<char const*>(sqlite3_column_text(m_statement, v));

                        if (0 == buf)
                        {
                            c.null = true;
                            buf = "";
                        }
                        c.data = buf;
                    }


                    r.push_back(c);
                }
                m_data.push_back(r);
            }
            else if (status == SQLITE_DONE)
            {
                // ok
            }
            else
            {
                error("query step failed", "query");
            }
        }

        status = sqlite3_finalize(m_statement);
        if (status != SQLITE_OK)
        {
            error("query finalization failed", "query");
        }

        m_statement = NULL;
    }

    bool next()
    {
        m_position++;

        if (m_position >= m_data.size())
            return false;
        return true;
    }

    const row* get() const
    {
        if ( m_position >= m_data.size() )
            return 0;
        else
            return &m_data[m_position];
    }

    std::map<std::string, int32_t> const& columns() const
    {
        return m_columns;
    }

    std::vector<std::string> const& types() const
    {
        return m_types;
    }


    int64_t last_row_id() const
    {
        return (int64_t)sqlite3_last_insert_rowid(m_session);
    }

    void insert(std::string const& statement, records const& rs)
    {
        checkSession();

        int status;

        records::size_type rows = rs.size();

        assert(!m_statement);
        status = sqlite3_prepare_v2(m_session,
                                    statement.c_str(),
                                    static_cast<int>(statement.size()),
                                    &m_statement,
                                    0);
        if (status != SQLITE_OK)
        {
            error("insert preparation failed", "insert");
        }

        m_log->get(LogLevel::Debug3) << "Inserting '" << statement << "'"<<
            std::endl;

        for (records::size_type r = 0; r < rows; ++r)
        {
            int const totalPositions = static_cast<int>(rs[0].size());
            for (int pos = 0; pos <= totalPositions-1; ++pos)
            {
                const column& c = rs[r][pos];
                if (c.null)
                {
                    status = sqlite3_bind_null(m_statement, pos+1);
                }
                else if (c.blobLen != 0)
                {
                    status = sqlite3_bind_blob(m_statement, pos+1,
                                               &(c.blobBuf.front()),
                                               static_cast<int>(c.blobLen),
                                               SQLITE_STATIC);
                }
                else
                {
                    status = sqlite3_bind_text(m_statement, pos+1,
                                               c.data.c_str(),
                                               static_cast<int>(c.data.length()),
                                               SQLITE_STATIC);
                }

                if (SQLITE_OK != status)
                {
                    std::ostringstream oss;
                    oss << "insert bind failed (row=" << r
                        <<", position=" << pos
                        << ")";
                    error(oss.str(), "insert");
                }
            }

            status = sqlite3_step(m_statement);

            if (status != SQLITE_DONE && status != SQLITE_ROW)
            {
                error("insert step failed", "insert");
            }
        }

        status = sqlite3_finalize(m_statement);
        if (status != SQLITE_OK)
        {
            error("insert finalize failed", "insert");
        }

        m_statement = NULL;
    }

    bool loadSpatialite(const std::string& module_name="")
    {
        std::string so_extension;
        std::string lib_extension;
#ifdef __APPLE__
        so_extension = "dylib";
        lib_extension = "mod_";
#endif

#ifdef __linux__
        so_extension = "so";
        lib_extension = "lib";
#endif

#ifdef _WIN32
        so_extension = "dll";
        lib_extension = "mod_";
#endif

// #if !defined(sqlite3_enable_load_extension)
// #error "sqlite3_enable_load_extension and spatialite is required for sqlite PDAL support"
// #endif
        int status = sqlite3_enable_load_extension(m_session, 1);
        if (status != SQLITE_OK)
        {
            error("spatialite library load failed", "loadSpatialite");
        }

        std::ostringstream oss;

        oss << "SELECT load_extension('";
        if (module_name.size())
            oss << module_name;
        else
            oss << lib_extension << "spatialite" << "." << so_extension;
        oss << "')";
        execute(oss.str());
        oss.str("");

        m_log->get(LogLevel::Debug3) <<  "SpatiaLite version: " << getSpatialiteVersion() << std::endl;

        return true;

    }

    bool haveSpatialite()
    {
        return doesTableExist("geometry_columns");
    }

    void initSpatialiteMetadata()
    {
        execute("SELECT InitSpatialMetadata(1)");
    }

    bool doesTableExist(std::string const& name)
    {
        const std::string sql("SELECT name FROM sqlite_master WHERE type = 'table'");

        query(sql);

        do
        {
            const row* r = get();
            if (!r)
                break ;// didn't have anything

            column const& c = r->at(0); // First column is table name!
            if (Utils::iequals(c.data, name))
            {
                return true;
            }
        } while (next());
        return false;
    }

    std::string getSpatialiteVersion()
    {
        // TODO: ought to parse this numerically, so we can do version checks
        const std::string sql("SELECT spatialite_version()");
        query(sql);

        const row* r = get();
        assert(r); // should get back exactly one row
        std::string ver = r->at(0).data;
        return ver;
    }

    std::string getSQLiteVersion()
    {
         // TODO: parse this numerically, so we can do version checks
         std::string v(sqlite3_libversion());
         return v;
    }

    LogPtr log() { return m_log; };

private:
    pdal::LogPtr m_log;
    std::string m_connection;
    sqlite3* m_session;
    sqlite3_stmt* m_statement;
    records m_data;
    records::size_type m_position;
    std::map<std::string, int32_t> m_columns;
    std::vector<std::string> m_types;

    void error(std::string const& userMssg, std::string const& func)
    {
        char const* sqlMssg = sqlite3_errmsg(m_session);

        std::ostringstream oss;
        oss << userMssg
           << " [SQLite::" << func << "]"
           << std::endl
           << "sqlite3 error: " << sqlMssg;
        throw pdal_error(oss.str());
    }

    void checkSession()
    {
        if (!m_session)
        {
            throw pdal_error("Database session not opened [SQLite::execute]");
        }
    }
};

} // namespace pdal
