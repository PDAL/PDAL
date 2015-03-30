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

#include <pdal/DbReader.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/XMLSchema.hpp>

#include "PgCommon.hpp"

#include <vector>

namespace pdal
{

class PDAL_DLL PgReader : public DbReader
{
    class Patch
    {
    public:
        Patch() : count(0), remaining(0)
        {}

        point_count_t count;
        point_count_t remaining;
        std::string hex;

        std::vector<uint8_t> binary;
        static const uint32_t trim = 26;

#define _base(x) ((x >= '0' && x <= '9') ? '0' : \
             (x >= 'a' && x <= 'f') ? 'a' - 10 : \
             (x >= 'A' && x <= 'F') ? 'A' - 10 : \
                '\255')
#define HEXOF(x) (x - _base(x))

        inline void update_binary()
        {
            // http://stackoverflow.com/questions/8197838/convert-a-long-hex-string-in-to-int-array-with-sscanf
            binary.resize((hex.size() - trim)/2);

            char const* source = hex.c_str() + trim;
            char const* p = 0;

            for (p = source; p && *p; p += 2)
                binary[(p - source) >> 1] =
                    ((HEXOF(*p)) << 4) + HEXOF(*(p + 1));
        }
    };

public:
    PgReader();
    ~PgReader();

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

    Options getDefaultOptions();
    virtual point_count_t getNumPoints() const;
    point_count_t getMaxPoints() const;
    std::string getDataQuery() const;
    std::string connString() const
        { return m_connection; }
    void getSession() const;

private:
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void processOptions(const Options& options);
    virtual void ready(PointTableRef table);
    virtual void initialize();
    virtual point_count_t read(PointViewPtr view, point_count_t count);
    virtual void done(PointTableRef table);
    virtual bool eof()
        { return m_atEnd; }

    SpatialReference fetchSpatialReference() const;
    uint32_t fetchPcid() const;
    point_count_t readPgPatch(PointViewPtr view, point_count_t numPts);

    // Internal functions for managing scroll cursor
    void CursorSetup();
    void CursorTeardown();
    bool NextBuffer();

    PGconn* m_session;
    std::string m_connection;
    std::string m_table_name;
    std::string m_schema_name;
    std::string m_column_name;
    std::string m_where;
    mutable uint32_t m_pcid;
    mutable point_count_t m_cached_point_count;
    mutable point_count_t m_cached_max_points;

    bool m_atEnd;
    uint32_t m_cur_row;
    uint32_t m_cur_nrows;
    PGresult* m_cur_result;
    Patch m_patch;

    PgReader& operator=(const PgReader&); // not implemented
    PgReader(const PgReader&); // not implemented
};

} // pdal
