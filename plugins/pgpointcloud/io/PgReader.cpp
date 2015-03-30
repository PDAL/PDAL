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

#include "PgReader.hpp"
#include <pdal/PointView.hpp>
#include <pdal/XMLSchema.hpp>

#include <iostream>

namespace pdal
{

static PluginInfo const s_info = PluginInfo(
    "readers.pgpointcloud",
    "Read data from pgpointcloud format. \"query\" option needs to be a \n" \
        "SQL statment selecting the data.",
    "http://pdal.io/stages/readers.pgpointcloud.html" );

CREATE_SHARED_PLUGIN(1, 0, PgReader, Reader, s_info)

std::string PgReader::getName() const { return s_info.name; }

PgReader::PgReader() : m_session(NULL), m_pcid(0), m_cached_point_count(0),
    m_cached_max_points(0)
{}


PgReader::~PgReader()
{
    //ABELL - Do bad things happen if we don't do this?  Already in done().
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


point_count_t PgReader::getNumPoints() const
{
    if (m_cached_point_count)
        return m_cached_point_count;

    std::ostringstream oss;
    oss << "SELECT Sum(PC_NumPoints(" << pg_quote_identifier(m_column_name) << ")) AS numpoints, ";
    oss << "Max(PC_NumPoints(" << pg_quote_identifier(m_column_name) << ")) AS maxpoints FROM ";
    if (m_schema_name.size())
        oss << pg_quote_identifier(m_schema_name) << ".";
    oss << pg_quote_identifier(m_table_name);
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
    oss << "SELECT text(PC_Uncompress(" << pg_quote_identifier(m_column_name) <<
        ")) AS pa, ";
    oss << "PC_NumPoints(" << pg_quote_identifier(m_column_name) <<
        ") AS npoints FROM ";
    if (!m_schema_name.empty())
        oss << pg_quote_identifier(m_schema_name) << ".";
    oss << pg_quote_identifier(m_table_name);
    if (!m_where.empty())
        oss << " WHERE " << m_where;

    log()->get(LogLevel::Debug) << "Constructed data query " <<
        oss.str() << std::endl;
    return oss.str();
}


point_count_t PgReader::getMaxPoints() const
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
    oss << "FROM pg_class c, pg_attribute a";
    if (!m_schema_name.empty())
    {
      oss << ", pg_namespace n";
    }
    oss << " WHERE c.relname = " << pg_quote_literal(m_table_name);
    oss << " AND a.attname = " << pg_quote_literal(m_column_name);
    if (!m_schema_name.empty())
    {
        oss << " AND c.relnamespace = n.oid AND n.nspname = " <<
            pg_quote_literal(m_schema_name);
    }

    char *pcid_str = pg_query_once(m_session, oss.str());

    uint32_t pcid = 0;
    if (pcid_str)
    {
       pcid = atoi(pcid_str);
       free(pcid_str);
    }

    if (!pcid) // Are pcid == 0 valid?
    {
        std::ostringstream oss;
        oss << "Unable to fetch pcid with column '"
            << m_column_name <<"' and  table ";
        if (!m_schema_name.empty())
          oss << "'" << m_schema_name << "'.";
        oss << "'" << m_table_name << "'";
        throw pdal_error(oss.str());
    }

    log()->get(LogLevel::Debug) << "     got pcid = " << pcid << std::endl;
    m_pcid = pcid;
    return pcid;
}


void PgReader::addDimensions(PointLayoutPtr layout)
{
    log()->get(LogLevel::Debug) << "Fetching schema object" << std::endl;

    uint32_t pcid = fetchPcid();

    std::ostringstream oss;
    oss << "SELECT schema FROM pointcloud_formats WHERE pcid = " << pcid;

    char *xmlStr = pg_query_once(m_session, oss.str());
    if (!xmlStr)
        throw pdal_error("Unable to fetch schema from `pointcloud_formats`");

    loadSchema(layout, xmlStr);
    free(xmlStr);
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


void PgReader::ready(PointTableRef /*table*/)
{
    m_atEnd = false;
    m_cur_row = 0;
    m_cur_nrows = 0;
    m_cur_result = NULL;

    if (getSpatialReference().empty())
        setSpatialReference(fetchSpatialReference());

    CursorSetup();
}


void PgReader::done(PointTableRef /*table*/)
{
    CursorTeardown();
    if (m_session)
        PQfinish(m_session);
    m_session = NULL;
    if (m_cur_result)
        PQclear(m_cur_result);
}

void PgReader::initialize()
{
    // First thing we do, is set up a connection
    if (!m_session)
        m_session = pg_connect(m_connection);

}


void PgReader::CursorSetup()
{
    std::ostringstream oss;
    oss << "DECLARE cur CURSOR FOR " << getDataQuery();
    pg_begin(m_session);
    pg_execute(m_session, oss.str());

    log()->get(LogLevel::Debug) << "SQL cursor prepared: " <<
        oss.str() << std::endl;
}


void PgReader::CursorTeardown()
{
    pg_execute(m_session, "CLOSE cur");
    pg_commit(m_session);
    log()->get(LogLevel::Debug) << "SQL cursor closed." << std::endl;
}


point_count_t PgReader::readPgPatch(PointViewPtr view, point_count_t numPts)
{
    point_count_t numRemaining = m_patch.remaining;
    PointId nextId = view->size();
    point_count_t numRead = 0;

    size_t offset = (m_patch.count - m_patch.remaining) * packedPointSize();
    char *pos = (char *)(m_patch.binary.data() + offset);

    while (numRead < numPts && numRemaining > 0)
    {
        writePoint(*view.get(), nextId, pos);
        pos += packedPointSize();
        numRemaining--;
        nextId++;
        numRead++;
    }
    m_patch.remaining = numRemaining;
    return numRead;
}


bool PgReader::NextBuffer()
{
    if (m_cur_row >= m_cur_nrows || !m_cur_result)
    {
        static std::string fetch = "FETCH 2 FROM cur";
        if (m_cur_result)
            PQclear(m_cur_result);
        m_cur_result = pg_query_result(m_session, fetch);
        bool logOutput = (log()->getLevel() > LogLevel::Debug3);
        if (logOutput)
            log()->get(LogLevel::Debug3) << "SQL: " << fetch << std::endl;
        if ((PQresultStatus(m_cur_result) != PGRES_TUPLES_OK) ||
            (PQntuples(m_cur_result) == 0))
        {
            PQclear(m_cur_result);
            m_cur_result = NULL;
            m_atEnd = true;
            return false;
        }

        m_cur_row = 0;
        m_cur_nrows = PQntuples(m_cur_result);
    }
    m_patch.hex = PQgetvalue(m_cur_result, m_cur_row, 0);
    m_patch.count = atoi(PQgetvalue(m_cur_result, m_cur_row, 1));
    m_patch.remaining = m_patch.count;
    m_patch.update_binary();

    m_cur_row++;
    return true;
}


point_count_t PgReader::read(PointViewPtr view, point_count_t count)
{
    if (eof())
        return 0;

    log()->get(LogLevel::Debug) << "readBufferImpl called with "
        "PointView filled to " << view->size() << " points" <<
        std::endl;

    point_count_t totalNumRead = 0;
    while (totalNumRead < count)
    {
        if (m_patch.remaining == 0)
            if (!NextBuffer())
                return totalNumRead;
        PointId bufBegin = view->size();
        point_count_t numRead = readPgPatch(view, count - totalNumRead);
        PointId bufEnd = bufBegin + numRead;
        totalNumRead += numRead;
    }
    return totalNumRead;
}

} // pdal
