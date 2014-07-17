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
#include <pdal/FileUtils.hpp>
#include <pdal/Utils.hpp>
#include <pdal/StageFactory.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/tokenizer.hpp>
#include <boost/make_shared.hpp>
#include <boost/algorithm/string.hpp>

#include <iostream>
#include <map>



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

//*********************************************************************************
//  pdal.drivers.pgpointcloud.Reader
//
//  The iterator downbelow controls the actual reading, the Reader just does some
//  basic setup and returning of metadata to the Writer at the other end of
//  the chain.
//
//  The core of PDAL only calls the following methods:
//
//  Options PgReader::getDefaultOptions()
//  void PgReader::initialize()
//  boost::uint64_t PgReader::getNumPoints() const
//  pdal::StageSequentialIterator* PgReader::createSequentialIterator(PointBuffer& buffer) const
//
//*********************************************************************************


PgReader::PgReader(const Options& options)
    : pdal::Reader(options)
    , m_session(NULL)
    , m_connection("")
    , m_table_name("")
    , m_schema_name("")
    , m_column_name("")
    , m_where("")
    , m_pcid(0)
    , m_cached_point_count(0)
    , m_cached_max_points(0)
{}

PgReader::~PgReader()
{
    if (m_session)
        PQfinish(m_session);

    return;
}

Options PgReader::getDefaultOptions()
{
    Options options;

    Option connection("connection", "", "Connection string to connect to database");
    Option table("table", "", "Table to read out of");
    Option schema("schema", "", "Schema to read out of");
    Option column("column", "", "Column to read out of");
    Option where("where", "", "SQL where clause to filter query");
    Option spatialreference("spatialreference", "", "override the source data spatialreference");

    options.add(connection);
    options.add(table);
    options.add(schema);
    options.add(column);
    options.add(where);
    options.add(spatialreference);

    return options;
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
}




void PgReader::initialize()
{
    // Database connection
    m_session = pg_connect(m_connection);

    // Read schema from pointcloud_formats if possible
//ABELL - Fix
//    m_schema = fetchSchema();

    // Allow spatialreference override if desired
//ABELL - Move to processOptions()
    try
    {
        setSpatialReference(getOptions().getValueOrThrow<pdal::SpatialReference>("spatialreference"));
    }
    catch (pdal::option_not_found const&)
    {
        // Read from pointcloud_formats otherwise
        setSpatialReference(fetchSpatialReference());
    }
}

boost::uint64_t PgReader::getNumPoints() const
{
    if (m_cached_point_count == 0)
    {
        std::ostringstream oss;
        oss << "SELECT Sum(PC_NumPoints(" << m_column_name << ")) AS numpoints, ";
        oss << "Max(PC_NumPoints(" << m_column_name << ")) AS maxpoints FROM ";
        if (m_schema_name.size())
        {
            oss << m_schema_name << ".";
        }
        oss << m_table_name;
        if (m_where.size())
        {
            oss << " WHERE " << m_where;
        }

        PGresult *result = pg_query_result(m_session, oss.str());

        if (PQresultStatus(result) != PGRES_TUPLES_OK)
        {
            throw pdal_error("unable to get point count");
        }
        m_cached_point_count = atoi(PQgetvalue(result, 0, 0));
        m_cached_max_points = atoi(PQgetvalue(result, 0, 1));
        PQclear(result);
    }

    return m_cached_point_count;
}

std::string PgReader::getDataQuery() const
{
    std::ostringstream oss;
    oss << "SELECT text(PC_Uncompress(" << m_column_name << ")) AS pa, ";
    oss << "PC_NumPoints(" << m_column_name << ") AS npoints FROM ";
    if (m_schema_name.size())
    {
        oss << m_schema_name << ".";
    }
    oss << m_table_name;
    if (m_where.size())
    {
        oss << " WHERE " << m_where;
    }

    log()->get(logDEBUG) << "Constructed data query " << oss.str() << std::endl;
    return oss.str();
}

boost::uint64_t PgReader::getMaxPoints() const
{
    if (m_cached_point_count == 0)
    {
        boost::uint64_t npoints = getNumPoints();
    }
    return m_cached_max_points;
}


boost::uint32_t PgReader::fetchPcid() const
{
    boost::uint32_t pcid = 0;

    if (m_pcid) return m_pcid;

    log()->get(logDEBUG) << "Fetching pcid ..." << std::endl;

    std::ostringstream oss;
    oss << "SELECT PC_Typmod_Pcid(a.atttypmod) AS pcid ";
    oss << "FROM pg_class c, pg_attribute a ";
    oss << "WHERE c.relname = '" << m_table_name << "' ";
    oss << "AND a.attname = '" << m_column_name << "' ";

    char *pcid_str(0);
    pcid_str = pg_query_once(m_session, oss.str());

    if (! pcid_str)
    {
        std::ostringstream oss;
        oss << "Unable to fetch pcid with column '" 
            << m_column_name <<"' and  table '" 
            << m_table_name <<"'";
        throw pdal_error(oss.str());
    }

    pcid = atoi(pcid_str);
    free(pcid_str);

    if (! pcid)
    {
        // Are pcid == 0 valid?
        std::ostringstream oss;
        oss << "Unable to fetch pcid with column '" 
            << m_column_name <<"' and  table '" 
            << m_table_name <<"'";
        throw pdal_error(oss.str());
    }


    log()->get(logDEBUG) << "     got pcid = " << pcid << std::endl;

    m_pcid = pcid;
    return pcid;
}

void PgReader::buildSchema(Schema *s)
{
    log()->get(logDEBUG) << "Fetching schema object" << std::endl;

    boost::uint32_t pcid = fetchPcid();

    std::ostringstream oss;
    oss << "SELECT schema FROM pointcloud_formats WHERE pcid = " << pcid;

    char *xml_str = pg_query_once(m_session, oss.str());
    if (!xml_str)
    {
        throw pdal_error("Unable to fetch schema from `pointcloud_formats`");
    }
    std::string xml = std::string(xml_str);
    free(xml_str);

    Schema storedSchema = Schema::from_xml(xml);

    for (size_t i = 0; i < storedSchema.numDimensions(); ++i)
    {
        Dimension d = storedSchema.getDimension(i);

        // For dimensions that do not have namespaces, we'll set the namespace
        // to the namespace of the current stage
        if (d.getNamespace().empty())
        {
            log()->get(logDEBUG4) << "setting namespace for dimension " <<
                d.getName() << " to "  << getName() << std::endl;

            if (d.getUUID().is_nil())
                d.createUUID();
            d.setNamespace(getName());
        }
        m_dims.push_back(s->appendDimension(d));
    }    

}

pdal::SpatialReference PgReader::fetchSpatialReference() const
{
    // Fetch the WKT for the SRID to set the coordinate system of this stage
    log()->get(logDEBUG) << "Fetching SRID ..." << std::endl;

    boost::uint32_t pcid = fetchPcid();

    // query_oss << "select ST_SRID(query.extent)::integer as code from (" << query << ") as query";
    // query_oss << "SELECT ST_SRID(extent)::integer as code from cloud";

    std::ostringstream oss;
    oss << "SELECT srid FROM pointcloud_formats WHERE pcid = " << pcid;

    char *srid_str = pg_query_once(m_session, oss.str());
    if (! srid_str)
        throw pdal_error("Unable to fetch srid for this table and column");

    boost::int32_t srid = atoi(srid_str);

    log()->get(logDEBUG) << "     got SRID = " << srid << std::endl;

    oss.str("");
    oss << "EPSG:" << srid;

    if (srid >= 0)
        return pdal::SpatialReference(oss.str());
    else
        return pdal::SpatialReference();
}


pdal::StageSequentialIterator* PgReader::createSequentialIterator() const
{
    return new pdal::drivers::pgpointcloud::iterators::sequential::PgIterator(*this, m_dims);
}


} // pgpointcloud
} // drivers
} // pdal
