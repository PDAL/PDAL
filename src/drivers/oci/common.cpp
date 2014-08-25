/******************************************************************************
* Copyright (c) 2011, Howard Butler, hobu.inc@gmail.com
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


#include <pdal/drivers/oci/common.hpp>

#include <iostream>

#include <pdal/Bounds.hpp>
#include <pdal/Dimension.hpp>
#include <pdal/Utils.hpp>

namespace pdal
{
namespace drivers
{
namespace oci
{

Connection connect(std::string connSpec)
{
    using namespace std;

    if (connSpec.empty())
        throw pdal_error("Oracle connection string empty! Unable to connect");

    string::size_type pos = connSpec.find("/", 0);
    string username = connSpec.substr(0, pos);
    connSpec = connSpec.substr(pos + 1);
    pos = connSpec.find("@", 0);
    string password = connSpec.substr(0, pos);
    string instance = connSpec.substr(pos + 1);

    Connection con = make_shared<OWConnection>(username.c_str(),
        password.c_str(), instance.c_str());
    if (!con->Succeeded())
        throw connection_failed("Oracle connection failed");
    return con;
}


schema::XMLSchema fetchSchema(Statement stmt, BlockPtr block)
{
    // Fetch the XML that defines the schema for this point cloud
    std::ostringstream schemaQuery;
    OCILobLocator* metadata = NULL;
    schemaQuery <<
        "DECLARE" << std::endl << "PC_TABLE VARCHAR2(32) := '" <<
            stmt->GetString(block->pc->base_table) << "';" << std::endl <<
        "PC_ID NUMBER := " << stmt->GetInteger(&(block->pc->pc_id)) <<
            ";" << std::endl <<
        "PC_COLUMN VARCHAR2(32) := '" <<
            stmt->GetString(block->pc->base_column) << "';" << std::endl <<
        "BEGIN" << std::endl <<
        std::endl <<
        "EXECUTE IMMEDIATE" << std::endl <<
        " 'SELECT T.'||PC_COLUMN||'.PC_OTHER_ATTRS.getClobVal()"
            "FROM '||pc_table||' T WHERE T.ID='||"
            "PC_ID INTO :metadata;" << std::endl <<
        "END;" << std::endl;

    Statement getSchemaStmt(
        block->m_connection->CreateStatement(schemaQuery.str().c_str()));
    getSchemaStmt->BindName(":metadata", &metadata);

    getSchemaStmt->Execute();

    char* pc_schema = getSchemaStmt->ReadCLob(metadata);
    std::string pc_schema_xml;
    if (pc_schema)
    {
        pc_schema_xml = pc_schema;
        CPLFree(pc_schema);
    }
    return schema::Reader(pc_schema_xml).schema();
}


Block::Block(Connection connection) : num_points(0), m_connection(connection),
    m_num_remaining(0), m_fetched(false), m_point_size(0)
{
    m_connection->CreateType(&blk_extent);
    m_connection->CreateType(&blk_extent->sdo_ordinates,
        m_connection->GetOrdinateType());
    m_connection->CreateType(&blk_extent->sdo_elem_info,
        m_connection->GetElemInfoType());
    m_connection->CreateType(&blk_domain);
    m_connection->CreateType(&pc);
}


Block::~Block()
{
    m_connection->DestroyType(&blk_domain);
    m_connection->DestroyType(&blk_extent->sdo_elem_info);
    m_connection->DestroyType(&blk_extent->sdo_ordinates);
    m_connection->DestroyType(&pc);
    // For some reason having the dtor destroy this
    // causes a segfault
    // m_connection->DestroyType(&blk_extent);
}


void Block::update(schema::XMLSchema *s)
{
    m_point_size = 0; // Wipe the size to reset
    m_num_remaining = num_points;
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
        
        m_point_size += Dimension::size(di->m_type);
    }
}

} // namespace oci
} // namespace drivers
} // namespace pdal

