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

#include <pdal/drivers/soci/Reader.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/Utils.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/tokenizer.hpp>
#include <boost/make_shared.hpp>
#include <boost/algorithm/string.hpp>

#include <iostream>
#include <map>


namespace pdal
{
namespace drivers
{
namespace soci
{


Reader::Reader(const Options& options)
    : pdal::Reader(options)
    , m_database_type(DATABASE_UNKNOWN)
    , m_query_type(QUERY_UNKNOWN)
    , m_cachedPointCount(0)
{

}


boost::uint64_t Reader::getNumPoints() const
{
    if (m_cachedPointCount != 0) return m_cachedPointCount;

    std::string const& query = getOptions().getValueOrThrow<std::string>("query");      
    if (m_query_type == QUERY_CLOUD)
    {
        std::ostringstream block_query;
        std::ostringstream cloud_query;
        
        ::soci::row r;
        ::soci::statement clouds = (m_session->prepare << query, ::soci::into(r));
        clouds.execute();
            
        bool bDidRead = clouds.fetch();
        if (!bDidRead) 
            throw pdal_error("Unable to fetch a cloud in getNumPoints!");
        
        boost::int64_t total_count(0);        
        while(bDidRead)
        {
            std::stringstream blocks_query;
            
            std::string block_table = r.get<std::string>("block_table");
            boost::int32_t cloud_id = r.get<boost::int32_t>("cloud_id");
            
            blocks_query  << "select sum(num_points) from " << block_table 
                << " where cloud_id=" << cloud_id;

            boost::int64_t count;
            *m_session << blocks_query.str(), ::soci::into(count);
            
            if (count < 0)
            {
                throw pdal_error("getNumPoints returned a count < 0!");
            }
            
            total_count = total_count + count;
            
            bDidRead = clouds.fetch();
        }        
        m_cachedPointCount = static_cast<boost::uint64_t>(total_count);     
        return m_cachedPointCount;        
            
        
    }
    else if (m_query_type == QUERY_BLOCKS_PLUS_CLOUD_VIEW)
    {
        std::ostringstream query_oss;
        query_oss << "select sum(num_points) from (" << query << ") as summation";

        boost::int64_t count;
        *m_session << query_oss.str(), ::soci::into(count);
        
        if (count < 0)
        {
            throw pdal_error("getNumPoints returned a count < 0!");
        }
        
        m_cachedPointCount = static_cast<boost::uint64_t>(count);     
        return m_cachedPointCount;
    }    
    return m_cachedPointCount;
}

void Reader::initialize()
{
    pdal::Reader::initialize();

    std::string const& query = getOptions().getValueOrThrow<std::string>("query");
    std::string const& connection = getOptions().getValueOrThrow<std::string>("connection");
        
    m_database_type = getDatabaseConnectionType(getOptions().getValueOrThrow<std::string>("type"));
    m_session = connectToDataBase(connection, m_database_type);
    // if (m_database_type == DATABASE_UNKNOWN)
    // {
    //     std::stringstream oss;
    //     oss << "Database connection type '" << getOptions().getValueOrThrow<std::string>("type") << "' is unknown or not configured";
    //     throw soci_driver_error(oss.str());
    // }
    // 
    // try
    // {
    //     if (m_database_type == DATABASE_POSTGRESQL)
    //         m_session = new ::soci::session(::soci::postgresql, connection);
    //     
    //     log()->get(logDEBUG) << "Connected to database" << std::endl;
    //     
    // } catch (::soci::soci_error const& e)
    // {
    //     std::stringstream oss;
    //     oss << "Unable to connect to database with error '" << e.what() << "'";
    //     throw pdal_error(oss.str());
    // }

    m_session->set_log_stream(&(log()->get(logDEBUG2)));    
    m_query_type = describeQueryType(query);

    Schema& schema = getSchemaRef();
    schema = fetchSchema(query);

    try
    {
        setSpatialReference(getOptions().getValueOrThrow<pdal::SpatialReference>("spatialreference"));

    }
    catch (pdal::option_not_found const&)
    {
        // If one wasn't set on the options, we'll ignore at this
        setSpatialReference(fetchSpatialReference(query));

    }

}




Options Reader::getDefaultOptions()
{
    Options options;

    Option connection("connection",
                      "",
                      "Connection string to connect to database");

    Option query("query",
                 "",
                 "SELECT statement that returns point cloud");

    Option capacity("capacity",
                    0,
                    "Block capacity");

    Option xml_schema_dump("xml_schema_dump", std::string(""), "Filename to dump the XML schema to.");

    options.add(connection);
    options.add(query);
    options.add(capacity);
    options.add(xml_schema_dump);

    return options;
}

Reader::~Reader()
{
    return;
}

QueryType Reader::describeQueryType(std::string const& query) const
{

    typedef std::pair<std::string, bool> SB;
    std::map<std::string, bool> block_fields;
    std::map<std::string, bool> cloud_fields;
    // 
    // // We must have all of these field names present to be considered block
    // // data.
    if (m_database_type == DATABASE_ORACLE)
    {
        block_fields.insert(SB("OBJ_ID", false));
        block_fields.insert(SB("BLK_ID", false));
        block_fields.insert(SB("BLK_EXTENT", false));
        block_fields.insert(SB("BLK_DOMAIN", false));
        block_fields.insert(SB("PCBLK_MIN_RES", false));
        block_fields.insert(SB("PCBLK_MAX_RES", false));
        block_fields.insert(SB("NUM_POINTS", false));
        block_fields.insert(SB("NUM_UNSORTED_POINTS", false));
        block_fields.insert(SB("PT_SORT_DIM", false));
        block_fields.insert(SB("POINTS", false));
        block_fields.insert(SB("SRID", false));
                
    } else if (m_database_type == DATABASE_POSTGRESQL)
    {
        block_fields.insert(SB("CLOUD_ID", false));
        block_fields.insert(SB("BLOCK_ID", false));
        block_fields.insert(SB("NUM_POINTS", false));
        block_fields.insert(SB("POINTS", false));
        block_fields.insert(SB("EXTENT", false));
        block_fields.insert(SB("SRID", false));
    }

    if (m_database_type == DATABASE_ORACLE)
    {
        cloud_fields.insert(SB("OBJ_ID", false));
        
    } else if (m_database_type == DATABASE_POSTGRESQL)
    {
        cloud_fields.insert(SB("BLOCK_TABLE", false));
        cloud_fields.insert(SB("SCHEMA", false));
        cloud_fields.insert(SB("CLOUD_ID", false));
        cloud_fields.insert(SB("EXTENT", false));
        cloud_fields.insert(SB("SRID", false));
    }
    
    ::soci::row r;
    *m_session << query, ::soci::into(r);
    
    for(std::size_t i = 0; i != r.size(); ++i)
    {
        ::soci::column_properties const& props = r.get_properties(i);
        std::string const& name = boost::to_upper_copy(props.get_name());
        log()->get(logDEBUG3) << "Query returned field name: " << name << std::endl;        
        std::map<std::string, bool>::iterator b = block_fields.find(name);
        if (b != block_fields.end())
        {
            b->second = true;
        }
        std::map<std::string, bool>::iterator c = cloud_fields.find(name);
        if (c != cloud_fields.end())
        {
            c->second = true;
        }        
        
    }
    
    bool bHasBlockFields(false);
    std::map<std::string, bool>::const_iterator q;
    for (q = block_fields.begin(); q != block_fields.end(); ++q)
    {
        if (q->second == false)
        {
            bHasBlockFields = false;
            break;
        }
        bHasBlockFields = true;
    }

    bool bHasCloudFields(false);
    for (q = cloud_fields.begin(); q != cloud_fields.end(); ++q)
    {
        if (q->second == false)
        {
            bHasCloudFields = false;
            break;
        }
        bHasCloudFields = true;
    }

    if (bHasBlockFields && bHasCloudFields)
    {
        log()->get(logDEBUG) << "Query type is QUERY_BLOCKS_PLUS_CLOUD_VIEW" << std::endl;
        return QUERY_BLOCKS_PLUS_CLOUD_VIEW;
    }

    if (bHasCloudFields)
    {
        log()->get(logDEBUG) << "Query type is QUERY_CLOUD" << std::endl;
        return QUERY_CLOUD;
    }

    return QUERY_UNKNOWN;

}


pdal::SpatialReference Reader::fetchSpatialReference(std::string const& query) const
{
    // Fetch the WKT for the SRID to set the coordinate system of this stage
    log()->get(logDEBUG) << "Fetching schema object" << std::endl;

    // query_oss << "select ST_SRID(query.extent)::integer as code from (" << query << ") as query";
    // query_oss << "SELECT ST_SRID(extent)::integer as code from cloud";

    ::soci::row r;
    ::soci::indicator ind;
    boost::int64_t srid;
    ::soci::statement clouds = (m_session->prepare << query, ::soci::into(r, ind));
    clouds.execute();
    
    bool bDidRead = clouds.fetch();
    
    srid = (boost::int64_t)r.get<boost::int32_t>("srid");
    
    if (!bDidRead)
        throw pdal_error("Unable to fetch srid for query");

    if (ind == ::soci::i_null)
    {
        log()->get(logDEBUG) << "No SRID was selected for query" << std::endl;
        return pdal::SpatialReference();
        
    }

    log()->get(logDEBUG) << "query returned " << srid << std::endl;    
    std::ostringstream oss;
    oss <<"EPSG:" << srid;

    if (srid >= 0)
        return pdal::SpatialReference(oss.str());
    else
        return pdal::SpatialReference();
}

pdal::Schema Reader::fetchSchema(std::string const& query) const
{
    log()->get(logDEBUG) << "Fetching schema object" << std::endl;

    ::soci::row r;
    ::soci::statement clouds = (m_session->prepare << query, ::soci::into(r));
    clouds.execute();
        
    bool bDidRead = clouds.fetch();
    
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

pdal::StageSequentialIterator* Reader::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::drivers::soci::iterators::sequential::Reader(*this, buffer);
}

namespace iterators
{
namespace sequential
{


IteratorBase::IteratorBase(const pdal::drivers::soci::Reader& reader)
    : m_at_end(false)
    , m_active_buffer(BufferPtr())
    , m_buffer_position(0)
    , m_reader(reader)
{
    pdal::Options const& options = reader.getOptions();
    std::string const& connection = options.getValueOrThrow<std::string>("connection");
    std::string const& query = options.getValueOrThrow<std::string>("query");
        
    m_database_type = getReader().getDatabaseType();
    m_session = connectToDataBase(connection, m_database_type);
    
    return;
}

::soci::statement IteratorBase::getNextCloud(   std::string const& cloud_table_name, 
                                                boost::int32_t& cloud_id, 
                                                ::soci::row& r)
{

    std::ostringstream select_blocks;

    select_blocks
            << "select T.cloud_id, T.block_id, T.extent::box3d::text, T.num_points, T.points from "
            << cloud_table_name << " T WHERE T.cloud_id = "
            << cloud_id;

    ::soci::statement blocks = (m_session->prepare << select_blocks.str(), ::soci::into(r));
    blocks.execute();
    return blocks;

}

IteratorBase::~IteratorBase()
{
}



const pdal::drivers::soci::Reader& IteratorBase::getReader() const
{
    return m_reader;
}

void IteratorBase::readBlob(::soci::row& block,
                            boost::uint32_t howMany)
{
    boost::uint32_t nAmountRead = 0;
    
    std::stringstream hex_data;
    hex_data << block.get<std::string>("points");
    
    std::size_t trim = 2;
    std::string trimmed = hex_data.str().substr(trim, hex_data.str().size()-trim);
    std::vector<boost::uint8_t> binary_data = Utils::hex_string_to_binary(trimmed);
    
    
    unsigned char* data = (unsigned char*) &(binary_data.front());

    Schema const& oracle_schema = m_active_buffer->getSchema();

    boost::uint32_t howMuchWeRead = binary_data.size();
    boost::uint32_t howMuchTheBlobShouldBe = block.get<int>("num_points") * oracle_schema.getByteSize();
    if (howMuchWeRead != howMuchTheBlobShouldBe)
    {
        std::stringstream oss;
        oss << "Did not read the amount of binary data as expected -- read: " << howMuchWeRead << " should read: " << howMuchTheBlobShouldBe;
        throw soci_driver_error(oss.str());
    }
    boost::uint32_t howMuchToRead = howMany * oracle_schema.getByteSize();
    m_active_buffer->setDataStride(data, 0, howMuchToRead);

    m_active_buffer->setNumPoints(howMany);    
}

void IteratorBase::fillUserBuffer(PointBuffer& user_buffer)
{

    Schema const& user_schema = user_buffer.getSchema();
    schema::index_by_index const& idx = user_schema.getDimensions().get<schema::index>();

    boost::int32_t numUserSpace = user_buffer.getCapacity() - user_buffer.getNumPoints();
    if (numUserSpace < 0)
        throw pdal_error("We ran out of space!");

    boost::int32_t numOraclePoints = m_active_buffer->getNumPoints() - m_buffer_position;
    
    schema::index_by_index::size_type i(0);
    for (i = 0; i < idx.size(); ++i)
    {
            copyDatabaseData( *m_active_buffer, 
                            user_buffer, 
                            idx[i], 
                            m_buffer_position, 
                            user_buffer.getNumPoints(), 
                            (std::min)(numOraclePoints,numUserSpace));

    }

    bool bSetPointSourceId = getReader().getOptions().getValueOrDefault<bool>("populate_pointsourceid", false);
    if (bSetPointSourceId)
    {  
        Dimension const* point_source_field = &(user_buffer.getSchema().getDimensionOptional("PointSourceId").get());
        if (point_source_field)
        {  
            for (boost::int32_t i = 0; i < numUserSpace; ++i)
            {  
                if (i < 0)
                    throw soci_driver_error("point_source_field point index is less than 0!");
                user_buffer.setField(*point_source_field, i, m_active_cloud_id);
            }
        }
    }
        
    if (numOraclePoints > numUserSpace)
        m_buffer_position = m_buffer_position + numUserSpace;
    else if (numOraclePoints < numUserSpace)
        m_buffer_position = 0;
    
    boost::uint32_t howManyThisRead = (std::min)(numUserSpace, numOraclePoints);
    user_buffer.setNumPoints(howManyThisRead + user_buffer.getNumPoints());
}

void IteratorBase::copyDatabaseData(  PointBuffer& source, 
                                      PointBuffer& destination, 
                                      Dimension const& dest_dim, 
                                      boost::uint32_t source_starting_position, 
                                      boost::uint32_t destination_starting_position,
                                      boost::uint32_t howMany)
{
    
    boost::optional<Dimension const&> source_dim = source.getSchema().getDimensionOptional(dest_dim.getName());
    
    if (!source_dim)
    {
        return;
    }

    for (boost::uint32_t i = 0; i < howMany; ++i)
    {
        if (dest_dim.getInterpretation() == source_dim->getInterpretation() &&
            dest_dim.getByteSize() == source_dim->getByteSize() && 
            pdal::Utils::compare_distance(dest_dim.getNumericScale(), source_dim->getNumericScale()) &&
            pdal::Utils::compare_distance(dest_dim.getNumericOffset(), source_dim->getNumericOffset()) &&
            dest_dim.getEndianness() == source_dim->getEndianness() 
            )
        {
            // FIXME: This test could produce false positives
            boost::uint8_t* source_position = source.getData(source_starting_position+i) + source_dim->getByteOffset();
            boost::uint8_t* destination_position = destination.getData(destination_starting_position + i) + dest_dim.getByteOffset();
            memcpy(destination_position, source_position, source_dim->getByteSize());
        }
        else
        {
            PointBuffer::scaleData( source, 
                                    destination, 
                                    *source_dim, 
                                    dest_dim, 
                                    source_starting_position + i,
                                    destination_starting_position + i);
        }
    }
    
}

boost::uint32_t IteratorBase::myReadBuffer(PointBuffer& data)
{
        std::cout << "we have: " << data.getNumPoints() <<  " for capcity: " << data.getCapacity() << std::endl;
    if (getReader().getQueryType() == QUERY_CLOUD)
        return myReadClouds(data);
    if (getReader().getQueryType() == QUERY_BLOCKS_PLUS_CLOUD_VIEW)
    {
        std::string const& query = getReader().getOptions().getValueOrThrow<std::string>("query");
        ::soci::statement blocks = m_session->prepare << query;
        ::soci::row block;

        return myReadBlocks(data, blocks, block);
    }

    return 0;
}

boost::uint32_t IteratorBase::myReadClouds(PointBuffer& user_buffer)
{
    boost::uint32_t numRead(0);


    ::soci::row cloud;
    std::string query = getReader().getOptions().getValueOrThrow<std::string>("query");
    ::soci::statement clouds = (m_session->prepare << query, ::soci::into(cloud));
    clouds.execute();
    bool bReadCloud = clouds.fetch();
    
    if (bReadCloud)
        getReader().log()->get(logDEBUG2) << "Fetched buffer with cloud id: " << cloud.get<int>("cloud_id") << " for myReadClouds" << std::endl;
    else
        getReader().log()->get(logDEBUG) << "unable to fetch initial cloud!" << std::endl;
        
    while (bReadCloud)
    {
        
        // std::string blocks_query = fetchBlocks(cloud);
        std::string blocks_query("SELECT SOMETHING");
        std::string const& block_table = cloud.get<std::string>("block_table");
        boost::int32_t cloud_id = cloud.get<int>("cloud_id");
        std::string const& schema = cloud.get<std::string>("schema");
        m_active_buffer = fetchPointBuffer(cloud_id, schema);

        ::soci::row block;
        ::soci::statement blocks = getNextCloud(block_table, cloud_id, block);
        boost::uint32_t numReadThisCloud = myReadBlocks(user_buffer, blocks, block);
        numRead = numRead + numReadThisCloud;

        getReader().log()->get(logDEBUG2) << "Read " << numReadThisCloud << " points from myReadBlocks" << std::endl;
        
        bReadCloud = clouds.fetch();
        if (!bReadCloud && numReadThisCloud == 0)
        {
            m_at_end = true;
            break;
        }
    }

    return numRead;
}

BufferPtr IteratorBase::fetchPointBuffer(   boost::int32_t const& cloud_id,
                                            std::string const& schema_xml)
{
    BufferMap::const_iterator i = m_buffers.find(cloud_id);

    if (i != m_buffers.end())
    {
        getReader().log()->get(logDEBUG2) << "IteratorBase::fetchPointBuffer: found existing PointBuffer with id " << cloud_id << std::endl;
        return i->second;
    }
    else
    {
        std::stringstream query;

        Schema schema = Schema::from_xml(schema_xml);

        BufferPtr output  = BufferPtr(new PointBuffer(schema));
        std::pair<int, BufferPtr> p(cloud_id, output);
        m_buffers.insert(p);
        getReader().log()->get(logDEBUG2) << "IteratorBase::fetchPointBuffer: creating new PointBuffer with id " << cloud_id << std::endl;

        return p.second;
    }
}

boost::uint32_t IteratorBase::myReadBlocks(PointBuffer& user_buffer, ::soci::statement& blocks, ::soci::row& block)
{
    boost::uint32_t numPointsRead = 0;
    
    // user_buffer.setNumPoints(0);

    if (!m_active_buffer) 
    {
        m_active_buffer = fetchPointBuffer(block.get<int>("cloud_id"), block.get<std::string>("schema"));
        m_active_cloud_id = block.get<int>("cloud_id");
    }
    // 
    // This shouldn't ever happen
    int num_points = block.get<int>("num_points");
    if (num_points > static_cast<boost::int32_t>(m_active_buffer->getCapacity()))
    {
        std::ostringstream oss;
        oss << "Block size, " << num_points <<", is too large to fit in "
            << "buffer of size " << user_buffer.getCapacity() <<". Increase buffer capacity with writer's \"chunk_size\" option "
            << "or increase the read buffer size";
        throw buffer_too_small(oss.str());
    }
    // 
    // if (!m_block->num_points)
    // {
    //     // We still have a block of data from the last readBuffer call
    //     // that was partially read.
    //     getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: fetching first block" << std::endl;
    //     bDidRead = m_initialQueryStatement->Fetch();
    //     if (!bDidRead)
    //     {
    //         m_at_end = true;
    //         return 0;
    //     }
    // 
    //     user_buffer.setSpatialBounds(getBounds(m_initialQueryStatement, m_block));
    // 
    // }
    // else
    // {
    //     // Our read was already "done" last readBuffer call, but if we're done,
    //     // we're done
    //     if (m_at_end)
    //         getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: we are at end of the blocks;" << std::endl;
    //     else
    //         getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: we have points left to read on this block" << std::endl;
    // 
    //     if (m_at_end) return 0;
    //     bDidRead = true;
    // 
    // }
    // 
    
    bool bDidRead = blocks.fetch();
    while (bDidRead)
    {
        boost::uint32_t numReadThisBlock = static_cast<boost::uint32_t>(block.get<int>("num_points"));
        boost::uint32_t numSpaceLeftThisBuffer = user_buffer.getCapacity() - user_buffer.getNumPoints();
    
        getReader().log()->get(logDEBUG4) << "IteratorBase::myReadBlocks:" "numReadThisBlock: "
                                          << numReadThisBlock << " numSpaceLeftThisBlock: "
                                          << numSpaceLeftThisBuffer << " total numPointsRead: "
                                          << numPointsRead << std::endl;
    
        numPointsRead = numPointsRead + numReadThisBlock;
    
        readBlob(block, numSpaceLeftThisBuffer);
        fillUserBuffer(user_buffer);
        // if (m_buffer_position != 0)
        // {
        //     return user_buffer.getNumPoints();
        // } 
        // else
        // {
        //     bDidRead = m_initialQueryStatement->Fetch();
        //     if (!bDidRead)
        //     {
        //         getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: done reading block. Read " << numPointsRead << " points" << std::endl;
        //         // m_at_end = true;
        //         return user_buffer.getNumPoints();
        //     }
        // }
        
        bDidRead = blocks.fetch();
        if (!bDidRead)
            return user_buffer.getNumPoints();
    
        if (getReader().getQueryType() == QUERY_BLOCKS_PLUS_CLOUD_VIEW)
        {
            boost::int32_t const& current_cloud_id = block.get<int>("cloud_id");
            if (current_cloud_id != m_active_cloud_id)
            {
                

                getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: current_cloud_id: "
                                                  << current_cloud_id << " m_active_cloud_id: "
                                                  << m_active_cloud_id << std::endl;                
                std::string schema;
                m_active_buffer = fetchPointBuffer(current_cloud_id, schema);
    
                m_active_cloud_id = current_cloud_id;
                return user_buffer.getNumPoints();
            }
        }
    }


    return numPointsRead;
}
// 
// pdal::Bounds<double> IteratorBase::getBounds(Statement statement, BlockPtr block)
// {
//     pdal::Vector<double> mins;
//     pdal::Vector<double> maxs;
// 
//     boost::int32_t bounds_length = statement->GetArrayLength(&(block->blk_extent->sdo_ordinates));
// 
//     getReader().log()->get(logDEBUG3) << "IteratorBase::getBounds: bounds length " << bounds_length << std::endl;
// 
//     double x(0.0);
//     double y(0.0);
// 
//     statement->GetElement(&(block->blk_extent->sdo_ordinates), 0, &x);
//     mins.add(x);
//     statement->GetElement(&(block->blk_extent->sdo_ordinates), 1, &y);
//     mins.add(y);
//     statement->GetElement(&(block->blk_extent->sdo_ordinates), 2, &x);
//     maxs.add(x);
//     statement->GetElement(&(block->blk_extent->sdo_ordinates), 3, &y);
//     maxs.add(y);
// 
//     pdal::Bounds<double> block_bounds(mins, maxs);
// 
//     getReader().log()->get(logDEBUG2) << "IteratorBase::getBounds: Fetched bounds of " << block_bounds << std::endl;
//     return block_bounds;
// }



//---------------------------------------------------------------------------
//
// SequentialIterator
//
//---------------------------------------------------------------------------

Reader::Reader(const pdal::drivers::soci::Reader& reader, PointBuffer& buffer)
    : IteratorBase(reader)
    , pdal::StageSequentialIterator(reader, buffer)
{
    return;
}


Reader::~Reader()
{
    return;
}


boost::uint64_t Reader::skipImpl(boost::uint64_t count)
{

    return naiveSkipImpl(count);
}


bool Reader::atEndImpl() const
{
    // return true;
    return m_at_end;
}


boost::uint32_t Reader::readBufferImpl(PointBuffer& data)
{
    // return data.getNumPoints();
    return myReadBuffer(data);
}



}
} // iterators::sequential::

}
}
} // namespace pdal::driver::soci
