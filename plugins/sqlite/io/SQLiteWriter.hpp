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

#include <pdal/DbWriter.hpp>
#include <pdal/StageFactory.hpp>
#include "SQLiteCommon.hpp"

namespace pdal
{


class PDAL_DLL SQLiteWriter : public DbWriter
{
public:
    SQLiteWriter();

    static void * create();
    static int32_t destroy(void *);
    std::string getName() const;

private:

    SQLiteWriter& operator=(const SQLiteWriter&); // not implemented
    SQLiteWriter(const SQLiteWriter&); // not implemented

    virtual void initialize();
    virtual void processOptions(const Options& options);
    virtual void ready(PointContextRef ctx);
    virtual void write(const PointBuffer& pointBuffer);
    virtual void done(PointContextRef ctx);

    void writeInit();
    void writeTile(const PointBuffer& buffer);
    void CreateBlockTable();
    void CreateCloudTable();
    bool CheckTableExists(std::string const& name);
    void DeleteBlockTable();
    void DeleteCloudTable();
    void CreateIndexes(std::string const& table_name,
                       std::string const& spatial_column_name,
                       bool is3d);

    bool IsValidGeometryWKT(std::string const& wkt) const;
    std::string loadGeometryWKT(std::string const& filename_or_wkt) const;
    void CreateCloud();

    std::unique_ptr<SQLite> m_session;

    bool m_doCreateIndex;
    BOX3D m_bounds; // Bounds of the entire point cloud
    bool m_sdo_pc_is_initialized;
	std::ostringstream m_block_insert_query;
	int32_t m_obj_id;
	int32_t m_block_id;
	uint32_t m_srid;
	int64_t m_num_points;
    Orientation::Enum m_orientation;
    bool m_pack;
    std::string m_block_table;
    std::string m_cloud_table;
    std::string m_cloud_column;
    std::string m_connection;
    std::string m_modulename;
    bool m_is3d;
    bool m_doCompression;;
    PatchPtr m_patch;
    PointContextRef m_context;
};

} // namespaces
