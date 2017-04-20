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

#pragma once

#include <netinet/in.h>

#include <pdal/DbWriter.hpp>
#include <pdal/StageFactory.hpp>
#include "PgCommon.hpp"

namespace pdal
{

class PDAL_DLL PgFlatWriter : public DbWriter
{
public:
    PgFlatWriter();
    ~PgFlatWriter();

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

private:
    PgFlatWriter& operator=(const PgFlatWriter&) = delete;
    PgFlatWriter(const PgFlatWriter&) = delete;

    virtual void addArgs(ProgramArgs& args);
    virtual void initialize();
    virtual void write(const PointViewPtr view);
    virtual void done(PointTableRef table);

    void writeInit();
    void writeTile(const PointViewPtr view);

    bool CheckTableExists(std::string const& name);

    void CreateTable(std::string const& schema_name,
                     std::string const& table_name);

    void DeleteTable(std::string const& schema_name,
                     std::string const& table_name);

    bool WriteTile(const PointViewPtr view);


    PGconn* m_session;
    std::string m_schema_name;
    std::string m_table_name;
    std::string m_connection;

    bool m_overwrite;
    std::string m_pre_sql;
    std::string m_post_sql;
};

} // namespace pdal
