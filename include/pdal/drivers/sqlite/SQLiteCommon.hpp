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

#include <pdal/pdal_error.hpp>
#include <pdal/Options.hpp>
#include <pdal/Log.hpp>
#include <pdal/XMLSchema.hpp>

#include <boost/algorithm/string.hpp>

#include <sqlite3.h>
#include <sstream>

namespace pdal
{
namespace drivers
{
namespace sqlite
{


    class sqlite_driver_error : public pdal_error
    {
    public:
        sqlite_driver_error(std::string const& msg)
            : pdal_error(msg)
        {}
    };

    class connection_failed : public sqlite_driver_error
    {
    public:
        connection_failed(std::string const& msg)
            : sqlite_driver_error(msg)
        {}
    };

    class buffer_too_small : public sqlite_driver_error
    {
    public:
        buffer_too_small(std::string const& msg)
            : sqlite_driver_error(msg)
        {}
    };

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
    Patch() : count(0), remaining(0), byte_size(0), bytes(0)
    {
    };
    
    point_count_t count;
    point_count_t remaining;
    
    size_t byte_size;
    std::vector<uint8_t> bytes;
    pdal::schema::XMLSchema m_schema;
    PointContext m_ctx;    
    double xOffset() const
        { return m_schema.m_scale.m_x.m_offset; }
    double yOffset() const
        { return m_schema.m_scale.m_y.m_offset; }
    double zOffset() const
        { return m_schema.m_scale.m_z.m_offset; }
    double xScale() const
        { return m_schema.m_scale.m_x.m_scale; }
    double yScale() const
        { return m_schema.m_scale.m_y.m_scale; }
    double zScale() const
        { return m_schema.m_scale.m_z.m_scale; }

    void update(pdal::schema::XMLSchema *s)
    {
        using namespace pdal;
        
        remaining = count;
        m_schema.m_orientation = s->m_orientation;
        for (auto di = s->m_dims.begin(); di != s->m_dims.end(); ++di)
        {
            schema::DimInfo& d = *di;
            Dimension::Id::Enum id = m_ctx.findDim(d.m_name);
            if (id == Dimension::Id::X)
            {
                m_schema.m_scale.m_x.m_scale = d.m_scale;
                m_schema.m_scale.m_x.m_offset = d.m_offset;
            }
            if (id == Dimension::Id::Y)
            {
                m_schema.m_scale.m_y.m_scale = d.m_scale;
                m_schema.m_scale.m_y.m_offset = d.m_offset;
            }
            if (id == Dimension::Id::Z)
            {
                m_schema.m_scale.m_z.m_scale = d.m_scale;
                m_schema.m_scale.m_z.m_offset = d.m_offset;
            }
        }
    }

};

typedef boost::shared_ptr<Patch> PatchPtr;

 
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
    void log(int num, char const* msg)
    {
        std::ostringstream oss;
        oss << "SQLite code: " << num << " msg: '" << msg << "'";
        m_log->get(LogLevel::Debug) << oss.str() << std::endl;
    }

    static void log_callback(void *p, int num, char const* msg)
    {
        SQLite* sql = reinterpret_cast<SQLite*>(p);
        sql->log(num, msg);
    }
    
        
    void connect(bool bWrite=false)
    {
        if ( ! m_connection.size() )
        {
            throw connection_failed("unable to connect to sqlite3 database, no connection string was given!");
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
        
        int code = sqlite3_open_v2(m_connection.c_str(), &m_session, flags, 0);
        if ( (code != SQLITE_OK) ) 
        {
            check_error("unable to connect to database");
        }
    }
    
    void execute(std::string const& sql, std::string errmsg="")
    {
        if (!m_session)
            throw sqlite_driver_error("Session not opened!");
        m_log->get(LogLevel::Debug3) << "Executing '" << sql <<"'"<< std::endl;  
        
        int code = sqlite3_exec(m_session, sql.c_str(), NULL, NULL, NULL);
        if (code != SQLITE_OK)
        {
            std::ostringstream oss;
            oss << errmsg <<" '" << sql << "'";
            throw sqlite_driver_error(oss.str());
        }

  
        
    }
    
    void begin()
    {
        execute("BEGIN", "Unable to begin transaction");
    }
    
    void commit()
    {
        execute("COMMIT", "Unable to commit transaction");
    }
    
    void query(std::string const& query)
    {
        m_position = 0;
        m_columns.clear();
        m_data.clear();
        sqlite3_reset(m_statement);

        m_log->get(LogLevel::Debug3) << "Querying '" << query.c_str() <<"'"<< std::endl;  
        
        char const* tail = 0; // unused;
        int res = sqlite3_prepare_v2(m_session,
                                  query.c_str(),
                                  static_cast<int>(query.size()),
                                  &m_statement,
                                  &tail);
        if (res != SQLITE_OK)
        {
            char const* zErrMsg = sqlite3_errmsg(m_session);

              std::ostringstream ss;
              ss << "sqlite3_statement_backend::prepare: "
                 << zErrMsg;
              throw sqlite_driver_error(ss.str());
        }
        int numCols = -1;

        while (res != SQLITE_DONE)
        {
            res = sqlite3_step(m_statement);

            if (SQLITE_ROW == res)
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
                        std::string ccolumnName = boost::to_upper_copy(std::string(sqlite3_column_name(m_statement, v)));
                        std::string ccolumnType = boost::to_upper_copy(std::string(sqlite3_column_decltype(m_statement, v)));                            
                        m_columns.insert(std::pair<std::string, int32_t>(ccolumnName, v));
                        m_types.push_back(ccolumnType);
                    }

                    column c;
                    
                    if (sqlite3_column_type(m_statement, v) == SQLITE_BLOB)
                    {
                        int len = sqlite3_column_bytes(m_statement, v);
                        const char* buf = (char*) sqlite3_column_blob(m_statement, v);
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
        }        
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
    
    bool insert(std::string const& statement, records const& rs)
    {
        records::size_type rows = rs.size();

        int res = sqlite3_prepare_v2(m_session,
                                  statement.c_str(),
                                  static_cast<int>(statement.size()),
                                  &m_statement,
                                  0);
        m_log->get(LogLevel::Debug3) << "Inserting '" << statement <<"'"<< std::endl;  

        if (res != SQLITE_OK)
        {
            char const* zErrMsg = sqlite3_errmsg(m_session);

              std::ostringstream ss;
              ss << "sqlite insert prepare: "
                 << zErrMsg;
              throw sqlite_driver_error(ss.str());
        }

        for (records::size_type r = 0; r < rows; ++r)
        {
            int const totalPositions = static_cast<int>(rs[0].size());
            for (int pos = 0; pos <= totalPositions-1; ++pos)
            {
                int didBind = SQLITE_OK;
                const column& c = rs[r][pos];
                if (c.null)
                {
                    didBind = sqlite3_bind_null(m_statement, pos+1);
                }
                else if (c.blobLen != 0)
                {
                    didBind = sqlite3_bind_blob(m_statement, pos+1,
                                                &(c.blobBuf.front()),
                                                static_cast<int>(c.blobLen),
                                                SQLITE_STATIC);
                }
                else
                {
                    didBind = sqlite3_bind_text(m_statement, pos+1,
                                                c.data.c_str(),
                                                static_cast<int>(c.data.length()),
                                                SQLITE_STATIC);
                }

                if (SQLITE_OK != didBind)
                {
                    std::ostringstream oss;
                    oss << "Failure to bind row number '" 
                        << r <<"' at position number '" <<pos <<"'";
                    throw sqlite_driver_error(oss.str());
                }
            }
            
            res = sqlite3_step(m_statement);

            if (SQLITE_DONE == res)
            {
            }
            else if (SQLITE_ROW == res)
            {
            }
            else
            {
                char const* zErrMsg = sqlite3_errmsg(m_session);

                std::ostringstream ss;
                ss << "sqlite insert failure: "
                   << zErrMsg;
                throw sqlite_driver_error(ss.str());                
            }
        }
        sqlite3_finalize(m_statement);
        return true;
    }

    bool spatialite()
    {
        std::string so_extension;
#ifdef __APPLE__
        so_extension = "dylib";
#endif

#ifdef __linux__
        so_extension = "so";
#endif

#ifdef _WIN32
        so_extension = "dll";
#endif

// #if !defined(sqlite3_enable_load_extension)
// #error "sqlite3_enable_load_extension and spatialite is required for sqlite PDAL support"
// #endif
        int code = sqlite3_enable_load_extension(m_session, 1);
        if (code != SQLITE_OK)
        {
            std::ostringstream oss;
            oss << "Unable to load spatialite extension!";
            throw sqlite_driver_error(oss.str());
        }

        std::ostringstream oss;
        
        oss << "SELECT load_extension('libspatialite." << so_extension <<"')";
        execute(oss.str());
        oss.str("");

        return true;

    }



    bool doesTableExist(std::string const& name)
    {
        std::ostringstream oss;

        oss << "SELECT name FROM sqlite_master WHERE type = \"table\"";

        query(oss.str());
    
        std::ostringstream debug;
        while (next())
        {
            const row* r = get();
            if (!r)
                break ;// didn't have anything
        
            column const& c = r->at(0); // First column is table name!
            if (boost::iequals(c.data, name))
            {
                return true;
            }
        }
        return false;
    }
            
private:  
    pdal::LogPtr m_log;
    std::string m_connection;
    sqlite3* m_session;
    sqlite3_stmt* m_statement;
    records m_data;
    records::size_type m_position;
    std::map<std::string, int32_t> m_columns;
    std::vector<std::string> m_types;
        
    void check_error(std::string const& msg)
    {
        const char *zErrMsg = sqlite3_errmsg(m_session);
        std::ostringstream ss;
        ss << msg << " sqlite error: " << zErrMsg;
        throw sqlite_driver_error(ss.str());        
    }
    

    
};

}
}
} // namespace pdal::driver::soci

