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
    , m_session(NULL)
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
    if ( m_session )
        delete m_session;

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
            oss << " WHERE " << m_where;
        }

        m_session->once << oss.str(), ::soci::into(m_cached_point_count), ::soci::into(m_cached_patch_count), ::soci::into(m_cached_max_points);
        oss.str("");
    }

    return m_cached_point_count;
}

std::string Reader::getDataQuery() const
{
    std::ostringstream oss;
    oss << "SELECT text(PC_Uncompress(" << m_column_name << ")) AS pa, ";
    oss << "PC_NumPoints(" << m_column_name << ") AS npoints FROM ";
    if ( m_schema_name.size() )
    {
        oss << m_schema_name << ".";
    }
    oss << m_table_name;
    if ( m_where.size() )
    {
        oss << " WHERE " << m_where;
    }

    log()->get(logDEBUG) << "Constructed data query " << oss.str() << std::endl;
    return oss.str();
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

    log()->get(logDEBUG) << "Fetching pcid ..." << std::endl;

    std::ostringstream oss;
    oss << "SELECT PC_Typmod_Pcid(a.atttypmod) AS pcid ";
    oss << "FROM pg_class c, pg_attribute a ";
    oss << "WHERE c.relname = '" << m_table_name << "' ";
    oss << "AND a.attname = '" << m_column_name << "' ";

    m_session->once << oss.str(), ::soci::into(pcid);
    oss.str("");

    if ( ! pcid )
        throw pdal_error("Unable to fetch pcid specified column and table");

    log()->get(logDEBUG) << "     got pcid = " << pcid << std::endl;
    
    m_pcid = pcid;
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
    log()->get(logDEBUG) << "Fetching SRID ..." << std::endl;

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

    log()->get(logDEBUG) << "     got SRID = " << srid << std::endl;    

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
    , m_reader(reader)
    , m_at_end(false)
    , m_buffer(NULL)
    , m_buffer_position(0)
    , m_statement(NULL)
    , m_patch_hex("")
    , m_patch_npoints(0)
    , m_session(NULL)
    , m_dimension_map(NULL)
{
    pdal::Options const& options = reader.getOptions();
    std::string const& connection = options.getValueOrThrow<std::string>("connection");
    m_session = connectToDataBase(connection);

    return;
}

Iterator::~Iterator()
{
    if ( m_statement )
        delete m_statement;

    if ( m_session )
        delete m_session;

    if ( m_dimension_map )
        delete m_dimension_map;

    if ( m_buffer )
        delete m_buffer;
}

const pdal::drivers::pgpointcloud::Reader& Iterator::getReader() const
{
    return m_reader;
}


boost::uint64_t Iterator::skipImpl(boost::uint64_t count)
{
    getReader().log()->get(logDEBUG) << "skipImpl called" << std::endl;

    return naiveSkipImpl(count);
}


bool Iterator::atEndImpl() const
{
    getReader().log()->get(logDEBUG) << "atEndImpl called" << std::endl;
    // return true;
    return m_at_end;
}


boost::uint32_t Iterator::readBufferImpl(PointBuffer& user_buffer)
{
    // do we already have a statement in place?
    // no, set one up ('select from table where')
    // yes,
    //   do we already have a cached patch?
    //   no, set one up
    //   yes, 
    //     is it all read?
    //     yes, get a new one 
    //        are there any more? no? set at_end to true, shut down statment
    //        yes, copy it into the cached patch
    //     no, read cached patch into user pointbuffer until it's empty
    // 


    getReader().log()->get(logDEBUG) << "readBufferImpl called with request for " << user_buffer.getNumPoints() << " points" << std::endl;

    // First time through, create the SQL statement, allocate holding pens
    // and fire it off!
    if ( ! m_statement )
    {
        m_statement = new ::soci::statement(*m_session);
        *m_statement = (m_session->prepare << getReader().getDataQuery(), ::soci::into(m_patch_hex), ::soci::into(m_patch_npoints));
        m_statement->execute();
        getReader().log()->get(logDEBUG) << "SQL statement prepared" << std::endl;
    }

    // Is the cache for patches ready?
    if ( ! m_buffer )
    {
        uint32_t max_points = getReader().getMaxPoints();
        m_buffer = new pdal::PointBuffer(getReader().getSchema(), max_points);
        m_buffer->setNumPoints(0);
        m_buffer_position = 0;
        getReader().log()->get(logDEBUG) << "allocated a cached point buffer with capacity of " << max_points << std::endl;
    }

    // Create a dimension map if we don't already have one
    if ( m_buffer && ! m_dimension_map )
    {
        m_dimension_map = pdal::PointBuffer::mapDimensions(*m_buffer, user_buffer);
    }

    boost::uint32_t num_loops = 0;
    // Read from the SQL statement until we run out of blocks, or break the loop
    // when we've filled up the user data buffer.
    while ( true ) 
    {
        // User buffer is full? We need to get out of this loop and 
        // let the writer decide what to do next.
        if ( user_buffer.getNumPoints() == user_buffer.getCapacity() )
        {
            getReader().log()->get(logDEBUG) << "User buffer is full, returning control to pdal core" << std::endl;
            break;
        }

        // If we've read all the contents of the cache buffer, get a fresh
        // patch from the database
        if ( m_buffer_position >= m_buffer->getNumPoints() )
        {
            // No more patches! We're done!
            if ( ! m_statement->fetch() )
            {
                m_at_end = true;
                break;
            }
            getReader().log()->get(logDEBUG) << "Fetched a patch from the database" << std::endl;

            // Copy data from the hex WKB string obtained by the database
            // into a pdal::PointBuffer for transfer to the user data buffer later
            //
            // Note: pointcloud hex WKB  has some header matter we need to trim off
            // before we can copy the raw data into the pdal::PointBuffer
            // endian (2) + pcid (8) + compression (8) + npoints (8) = 26 characters
            boost::uint32_t trim = 26;
            std::string hex_trimmed = m_patch_hex.substr(trim, m_patch_hex.size()-trim);
            std::vector<boost::uint8_t> binary_data = Utils::hex_string_to_binary(hex_trimmed);
            unsigned char* data = (unsigned char*) &(binary_data.front());
            schema::size_type point_size = m_buffer->getSchema().getByteSize();
            m_buffer->setDataStride(data, 0, m_patch_npoints * point_size);
            m_buffer->setNumPoints(m_patch_npoints);
            m_buffer_position = 0;
            getReader().log()->get(logDEBUG) << "Copied patch into cache, npoints = " << m_patch_npoints << std::endl;
        }

        // Much many more points do we have to process in this cache?
        boost::uint32_t points_in_cache = m_buffer->getNumPoints() - m_buffer_position;
        // How much space is left in the user buffer?
        boost::uint32_t space_in_user_buffer = user_buffer.getCapacity() - user_buffer.getNumPoints();
        boost::uint32_t points_to_copy = 0;
        // If there's space, put the whole cache into the user buffer,
        if ( space_in_user_buffer > points_in_cache )
        {
            points_to_copy = points_in_cache;
        }
        // otherwise, just fill the buffer to full.
        else
        {
            points_to_copy = space_in_user_buffer;
        }

        getReader().log()->get(logDEBUG) << "space_in_user_buffer = " << space_in_user_buffer << std::endl;
        getReader().log()->get(logDEBUG) << "points_in_cache = " << points_in_cache << std::endl;
        getReader().log()->get(logDEBUG) << "points_to_copy = " << points_to_copy << std::endl;
        getReader().log()->get(logDEBUG) << "m_buffer_position = " << m_buffer_position << std::endl;
        getReader().log()->get(logDEBUG) << "user_buffer.getNumPoints() = " << user_buffer.getNumPoints() << std::endl;

        // Do the copying from cache to user buffer
        // To do: this should be more tolerant of variations in source/dest schema
        PointBuffer::copyLikeDimensions(*m_buffer, user_buffer, 
                           *m_dimension_map,
                           m_buffer_position, user_buffer.getNumPoints(), 
                           points_to_copy);

        // Update the buffers regarding how full/empty they are
        m_buffer_position += points_to_copy;
        user_buffer.setNumPoints(user_buffer.getNumPoints()+points_to_copy);

        num_loops++;
        getReader().log()->get(logDEBUG) << "User buffer filling loop, iteration " << num_loops << std::endl;
    }
 
    return user_buffer.getNumPoints();
}

// void Iterator::copyPointBufferData(PointBuffer& source, 
//                               PointBuffer& destination,
//                               boost::uint32_t source_position,
//                               boost::uint32_t destination_position)
// {
//     Schema const& source_schema = source.getSchema();
//     schema::index_by_index const& source_idx = source_schema.getDimensions().get<schema::index>();
//     Schema const& destination_schema = destination.getSchema();
//     schema::index_by_index const& destination_idx = destination_schema.getDimensions().get<schema::index>();
// }




}} // pdal.drivers.pgpointcloud.iterators.sequential.Iterator

} // pgpointcloud
} // drivers
} // pdal
