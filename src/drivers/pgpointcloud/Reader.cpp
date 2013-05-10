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

#include <pdal/drivers/pgpointcloud/Reader.hpp>
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
MAKE_READER_CREATOR(pgpointcloudReader, pdal::drivers::pgpointcloud::Reader)
CREATE_READER_PLUGIN(pgpointcloud, pdal::drivers::pgpointcloud::Reader)
#endif


namespace pdal
{
namespace drivers
{
namespace pgpointcloud
{


Reader::Reader(const Options& options)
    : pdal::Reader(options)
    , m_session(0)
    , m_connection("")
    , m_table_name("")
    , m_schema_name("")
    , m_column_name("")
    , m_where("")
    , m_pcid(0)
    , m_cached_point_count(0)
    , m_cached_patch_count(0)
    , m_cached_max_points(0)
{

}

Options Reader::getDefaultOptions()
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

Reader::~Reader()
{
    return;
}


void Reader::initialize()
{
    pdal::Reader::initialize();

    // If we don't know the table name, we're SOL
    m_table_name = getOptions().getValueOrThrow<std::string>("table");

    // Connection string needs to exist and actually work
    m_connection = getOptions().getValueOrThrow<std::string>("connection");

    // Schema and column name can be defaulted safely
    m_column_name = getOptions().getValueOrDefault<std::string>("column", "pa");
    m_schema_name = getOptions().getValueOrDefault<std::string>("schema", "");

    // Read other preferences
    m_where = getOptions().getValueOrDefault<std::string>("where", "");

    // Database connection
    m_session = connectToDataBase(m_connection);
 
    // Read schema from pointcloud_formats if possible
    Schema& schema = getSchemaRef();
    schema = fetchSchema();

    // Allow spatialreference override if desired
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

boost::uint64_t Reader::getNumPoints() const
{
    if ( m_cached_point_count == 0 )
    {
        std::ostringstream oss;
        oss << "SELECT Sum(PC_NumPoints(" << m_column_name << ")) AS numpoints, ";
        oss << "Count(*) AS numpatches, ";
        oss << "Max(PC_NumPoints(" << m_column_name << ")) AS maxpoints FROM ";
        if ( m_schema_name.size() )
        {
            oss << m_schema_name << ".";
        }
        oss << m_table_name;
        if ( m_where.size() )
        {
            oss << " " << m_where;
        }

        m_session->once << oss.str(), ::soci::into(m_cached_point_count), ::soci::into(m_cached_patch_count), ::soci::into(m_cached_max_points);
        oss.str("");
    }

    return m_cached_point_count;
}

boost::uint64_t Reader::getNumPatches() const
{
    if ( m_cached_point_count == 0 )
    {
        boost::uint64_t npoints = getNumPoints();
    }
    return m_cached_patch_count;
}

boost::uint64_t Reader::getMaxPoints() const
{
    if ( m_cached_point_count == 0 )
    {
        boost::uint64_t npoints = getNumPoints();
    }
    return m_cached_max_points;
}


boost::uint32_t Reader::fetchPcid() const
{
    boost::uint32_t pcid = 0;

    if ( m_pcid ) return m_pcid;

    std::ostringstream oss;
    oss << "SELECT PC_Typmod_Pcid(a.atttypmod) AS pcid ";
    oss << "FROM pg_class c, pg_attribute a ";
    oss << "WHERE c.relname = '" << m_table_name << "' ";
    oss << "AND a.attname = '" << m_column_name << "' ";

    m_session->once << oss.str(), ::soci::into(pcid);
    oss.str("");

    if ( ! pcid )
        throw pdal_error("Unable to fetch pcid specified column and table");
    
    return pcid;
}

pdal::Schema Reader::fetchSchema() const
{
    log()->get(logDEBUG) << "Fetching schema object" << std::endl;

    boost::uint32_t pcid = fetchPcid();

    std::ostringstream oss;
    oss << "SELECT schema FROM pointcloud_formats WHERE pcid = " << pcid;

    ::soci::row r;
    ::soci::statement schemas = (m_session->prepare << oss.str(), ::soci::into(r));
    schemas.execute();
        
    if ( ! schemas.fetch() )
        throw pdal_error("Unable to retreive schema XML for specified column and table");

    std::string xml = r.get<std::string>("schema");    

    Schema schema = Schema::from_xml(xml);

    schema::index_by_index const& dims = schema.getDimensions().get<schema::index>();

    for (schema::index_by_index::const_iterator iter = dims.begin(); iter != dims.end(); ++iter)
    {
        // For dimensions that do not have namespaces, we'll set the namespace 
        // to the namespace of the current stage
        
        if (iter->getNamespace().size() == 0)
        {
            log()->get(logDEBUG4) << "setting namespace for dimension " << iter->getName() << " to "  << getName() << std::endl;
            

            Dimension d(*iter);
            if (iter->getUUID().is_nil())
            {
                d.createUUID();
            }            
            d.setNamespace(getName());
            schema.setDimension(d);        
        }
    }

    return schema;
}

pdal::SpatialReference Reader::fetchSpatialReference() const
{
    // Fetch the WKT for the SRID to set the coordinate system of this stage
    log()->get(logDEBUG) << "Fetching spatial setSpatialReference object" << std::endl;

    boost::uint32_t pcid = fetchPcid();

    // query_oss << "select ST_SRID(query.extent)::integer as code from (" << query << ") as query";
    // query_oss << "SELECT ST_SRID(extent)::integer as code from cloud";

    std::ostringstream oss;
    oss << "SELECT srid FROM pointcloud_formats WHERE pcid = " << pcid;

    ::soci::row r;
    ::soci::indicator ind;
    ::soci::statement srids = (m_session->prepare << oss.str(), ::soci::into(r, ind));
    srids.execute();
    oss.str("");
    
    if ( ! srids.fetch() )
        throw pdal_error("Unable to fetch srid for this table and column");
    
    boost::int32_t srid = r.get<boost::int32_t>("srid");

    if (ind == ::soci::i_null)
    {
        log()->get(logDEBUG) << "No SRID was selected for query" << std::endl;
        return pdal::SpatialReference();
    }

    log()->get(logDEBUG) << "query returned " << srid << std::endl;    

    oss << "EPSG:" << srid;

    if ( srid >= 0 )
        return pdal::SpatialReference(oss.str());
    else
        return pdal::SpatialReference();
}


pdal::StageSequentialIterator* Reader::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::drivers::pgpointcloud::iterators::sequential::Iterator(*this, buffer);
}

//*********************************************************************************
//  pdal.drivers.pgpointcloud.iterators.sequential.Iterator
//  
//  The iterator controls the actual reading of features, via calls to 
//
//  boost::uint64_t skipImpl(boost::uint64_t count)
//  bool            atEndImpl() const
//  boost::uint32_t readBufferImpl(PointBuffer& data)
//
//*********************************************************************************

namespace iterators
{
namespace sequential
{

Iterator::Iterator(const pdal::drivers::pgpointcloud::Reader& reader, PointBuffer& buffer)
    : pdal::StageSequentialIterator(reader, buffer)
    , m_at_end(false)
    , m_buffer_position(0)
    , m_reader(reader)
{
    pdal::Options const& options = reader.getOptions();
    std::string const& connection = options.getValueOrThrow<std::string>("connection");
    m_session = connectToDataBase(connection);
    
    return;
}

Iterator::~Iterator()
{
}

const pdal::drivers::pgpointcloud::Reader& Iterator::getReader() const
{
    return m_reader;
}


boost::uint64_t Iterator::skipImpl(boost::uint64_t count)
{

    return naiveSkipImpl(count);
}


bool Iterator::atEndImpl() const
{
    return true;
    // return m_at_end;
}


boost::uint32_t Iterator::readBufferImpl(PointBuffer& data)
{
    return data.getNumPoints();
    // return myReadBuffer(data);
}



}
} // iterators::sequential::

}
}
} // namespace pdal::driver::pgpointcloud
