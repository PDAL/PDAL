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

#include <pdal/drivers/pgpointcloud/PgReader.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/XMLSchema.hpp>

#include <iostream>

#ifdef USE_PDAL_PLUGIN_PGPOINTCLOUD
MAKE_READER_CREATOR(pgpointcloudReader, pdal::drivers::pgpointcloud::PgReader)
CREATE_READER_PLUGIN(pgpointcloud, pdal::drivers::pgpointcloud::PgReader)
#endif

namespace pdal
{
namespace drivers
{
namespace pgpointcloud
{

PgReader::PgReader(const Options& options)
    : pdal::Reader(options), m_session(NULL), m_pcid(0),
    m_cached_point_count(0), m_cached_max_points(0)
{}


PgReader::~PgReader()
{
    if (m_session)
        PQfinish(m_session);
}


Options PgReader::getDefaultOptions()
{
    Options ops;

    ops.add("connection", "", "Connection string to connect to database");
    ops.add("table", "", "Table to read out of");
    ops.add("schema", "", "Schema to read out of");
    ops.add("column", "", "Column to read out of");
    ops.add("where", "", "SQL where clause to filter query");
    ops.add("spatialreference", "",
        "override the source data spatialreference");

    return ops;
}


void PgReader::processOptions(const Options& options)
{
    // If we don't know the table name, we're SOL
    m_table_name = options.getValueOrThrow<std::string>("table");

    // Connection string needs to exist and actually work
    m_connection = options.getValueOrThrow<std::string>("connection");

    // Schema and column name can be defaulted safely
    m_column_name = options.getValueOrDefault<std::string>("column", "pa");
    m_schema_name = options.getValueOrDefault<std::string>("schema", "");

    // Read other preferences
    m_where = options.getValueOrDefault<std::string>("where", "");

    // Spatial reference.
    setSpatialReference(options.getValueOrDefault<SpatialReference>(
        "spatialreference", SpatialReference()));
}


void PgReader::ready(PointContext ctx)
{
    // Database connection
    m_session = pg_connect(m_connection);

    if (getSpatialReference().empty())
        setSpatialReference(fetchSpatialReference());    
}


uint64_t PgReader::getNumPoints() const
{
    if (m_cached_point_count)
        return m_cached_point_count;

    std::ostringstream oss;
    oss << "SELECT Sum(PC_NumPoints(" << m_column_name << ")) AS numpoints, ";
    oss << "Max(PC_NumPoints(" << m_column_name << ")) AS maxpoints FROM ";
    if (m_schema_name.size())
        oss << m_schema_name << ".";
    oss << m_table_name;
    if (m_where.size())
        oss << " WHERE " << m_where;

    PGresult *result = pg_query_result(m_session, oss.str());

    if (PQresultStatus(result) != PGRES_TUPLES_OK)
    {
        throw pdal_error("unable to get point count");
    }
    m_cached_point_count = atoi(PQgetvalue(result, 0, 0));
    m_cached_max_points = atoi(PQgetvalue(result, 0, 1));
    PQclear(result);

    return m_cached_point_count;
}


std::string PgReader::getDataQuery() const
{
    std::ostringstream oss;
    oss << "SELECT text(PC_Uncompress(" << m_column_name << ")) AS pa, ";
    oss << "PC_NumPoints(" << m_column_name << ") AS npoints FROM ";
    if (m_schema_name.size())
        oss << m_schema_name << ".";
    oss << m_table_name;
    if (m_where.size())
        oss << " WHERE " << m_where;

    log()->get(LogLevel::Debug) << "Constructed data query " <<
        oss.str() << std::endl;
    return oss.str();
}


uint64_t PgReader::getMaxPoints() const
{
    if (m_cached_point_count == 0)
        getNumPoints();  // Fills m_cached_max_points.
    return m_cached_max_points;
}


uint32_t PgReader::fetchPcid() const
{ 
    if (m_pcid)
        return m_pcid;

    log()->get(LogLevel::Debug) << "Fetching pcid ..." << std::endl;

    std::ostringstream oss;
    oss << "SELECT PC_Typmod_Pcid(a.atttypmod) AS pcid ";
    oss << "FROM pg_class c, pg_attribute a ";
    oss << "WHERE c.relname = '" << m_table_name << "' ";
    oss << "AND a.attname = '" << m_column_name << "' ";

    char *pcid_str = pg_query_once(m_session, oss.str());

    if (! pcid_str)
    {
        std::ostringstream oss;
        oss << "Unable to fetch pcid with column '" 
            << m_column_name <<"' and  table '" 
            << m_table_name <<"'";
        throw pdal_error(oss.str());
    }

    uint32_t pcid = atoi(pcid_str);
    free(pcid_str);

    if (!pcid)
    {
        // Are pcid == 0 valid?
        std::ostringstream oss;
        oss << "Unable to fetch pcid with column '" 
            << m_column_name <<"' and  table '" 
            << m_table_name <<"'";
        throw pdal_error(oss.str());
    }

    log()->get(LogLevel::Debug) << "     got pcid = " << pcid << std::endl;
    m_pcid = pcid;
    return pcid;
}


void PgReader::addDimensions(PointContext ctx)
{
    log()->get(LogLevel::Debug) << "Fetching schema object" << std::endl;

    uint32_t pcid = fetchPcid();

    std::ostringstream oss;
    oss << "SELECT schema FROM pointcloud_formats WHERE pcid = " << pcid;

    char *xml_str = pg_query_once(m_session, oss.str());
    if (!xml_str)
        throw pdal_error("Unable to fetch schema from `pointcloud_formats`");

    m_schema = schema::Reader(xml_str).schema();
    free(xml_str);

    schema::DimInfoList& dims = m_schema.m_dims;
    for (auto di = dims.begin(); di != dims.end(); ++di)
        di->m_id = ctx.registerOrAssignDim(di->m_name, di->m_type);
}


pdal::SpatialReference PgReader::fetchSpatialReference() const
{
    // Fetch the WKT for the SRID to set the coordinate system of this stage
    log()->get(LogLevel::Debug) << "Fetching SRID ..." << std::endl;

    uint32_t pcid = fetchPcid();

    std::ostringstream oss;
    oss << "SELECT srid FROM pointcloud_formats WHERE pcid = " << pcid;

    char *srid_str = pg_query_once(m_session, oss.str());
    if (! srid_str)
        throw pdal_error("Unable to fetch srid for this table and column");

    int32_t srid = atoi(srid_str);
    log()->get(LogLevel::Debug) << "     got SRID = " << srid << std::endl;

    oss.str("");
    oss << "EPSG:" << srid;

    if (srid >= 0)
        return pdal::SpatialReference(oss.str());
    else
        return pdal::SpatialReference();
}


pdal::StageSequentialIterator* PgReader::createSequentialIterator() const
{
    return new iterators::sequential::PgIterator(*this, m_schema.m_dims);
}

} // pgpointcloud
} // drivers
} // pdal
