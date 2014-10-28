/******************************************************************************
* Copyright (c) 2014, Howard Butler, howard@hobu.co
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

#include <pdal/Reader.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/XMLSchema.hpp>

#include "SQLiteCommon.hpp"

#include <vector>

PDAL_C_START

PDAL_DLL void PDALRegister_sqlite_reader(void* factory);

PDAL_C_END

namespace pdal
{
namespace drivers
{
namespace sqlite
{

class PDAL_DLL SQLiteReader : public pdal::Reader
{
public:
    SET_STAGE_NAME("drivers.sqlite.reader", "SQLite3 Reader")
    SET_STAGE_ENABLED(true)

    SQLiteReader();
    static Options getDefaultOptions();
    schema::XMLSchema fetchSchema(std::string const& query) const;
    SpatialReference fetchSpatialReference(std::string const& query) const;
    SQLite& getSession() { return *m_session.get(); }

private:
    std::unique_ptr<SQLite> m_session;
    std::string m_query;
    std::string m_schemaFile;
    std::string m_connection;
    std::string m_modulename;
    boost::optional<SpatialReference> m_spatialRef;
    PatchPtr m_patch;

    bool m_at_end;
    bool b_doneQuery;
    int32_t m_point_size;

    virtual void initialize();
    virtual void processOptions(const Options& options);
    virtual void addDimensions(PointContextRef ctx);
    virtual void ready(PointContextRef ctx);
    point_count_t read(PointBuffer& buf, point_count_t count);
    bool eof()
        { return m_at_end; }

    void validateQuery() const;
    point_count_t readPatch(PointBuffer& buffer, point_count_t count);
    bool NextBuffer();

    SQLiteReader& operator=(const SQLiteReader&); // not implemented
    SQLiteReader(const SQLiteReader&); // not implemented
};

} // namespace sqlite
} // namespace drivers
} // namespace pdal

