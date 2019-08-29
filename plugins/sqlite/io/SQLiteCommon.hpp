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

#include <pdal/Log.hpp>
#include <pdal/Metadata.hpp>
#include <pdal/util/Utils.hpp>

#include <sqlite3.h>

namespace pdal
{

class column
{
public:

    column() : null(true), blobBuf(0), blobLen(0){};
    template<typename T> column( T v) : null(false), blobBuf(0), blobLen(0)
    {
        data = Utils::toString(v);
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

    const unsigned char *getBytes() const
        { return buf.data(); }

    size_t byte_size()
        { return buf.size(); }

    void clear()
        {
           buf.clear();
           count = 0;
           remaining = 0;
           idx = 0;
         }
};
typedef std::shared_ptr<Patch> PatchPtr;

class PDAL_LOCAL SQLite
{
public:
    SQLite(std::string const& connection, LogPtr log);
    ~SQLite();

    static void log_callback(void *p, int num, char const* msg);
    void connect(bool bWrite = false);
    void execute(std::string const& sql);
    void begin();
    void commit();
    void query(std::string const& query);
    bool next();
    const row* get() const;
    std::map<std::string, int32_t> const& columns() const;
    std::vector<std::string> const& types() const;
    int64_t last_row_id() const;
    void insert(std::string const& statement, records const& rs);
    bool loadSpatialite(const std::string& module_name = "");
    bool haveSpatialite();
    void initSpatialiteMetadata();
    bool doesTableExist(std::string const& name);
    std::string getSpatialiteVersion();
    std::string getSQLiteVersion();
    LogPtr log();

private:
    pdal::LogPtr m_log;
    std::string m_connection;
    sqlite3* m_session;
    sqlite3_stmt* m_statement;
    records m_data;
    records::size_type m_position;
    std::map<std::string, int32_t> m_columns;
    std::vector<std::string> m_types;
    static std::vector<SQLite *> m_sqlites;

    void error(std::string const& userMssg, std::string const& func);
    void checkSession();
};

} // namespace pdal
