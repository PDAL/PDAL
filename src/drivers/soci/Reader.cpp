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
    if (m_database_type == DATABASE_UNKNOWN)
    {
        std::stringstream oss;
        oss << "Database connection type '" << getOptions().getValueOrThrow<std::string>("type") << "' is unknown or not configured";
        throw soci_driver_error(oss.str());
    }
    
    try
    {
        if (m_database_type == DATABASE_POSTGRESQL)
            m_session = new ::soci::session(::soci::postgresql, connection);
        
        log()->get(logDEBUG) << "Connected to database" << std::endl;
        
    } catch (::soci::soci_error const& e)
    {
        std::stringstream oss;
        oss << "Unable to connect to database with error '" << e.what() << "'";
        throw pdal_error(oss.str());
    }

    m_session->set_log_stream(&(log()->get(logDEBUG2)));    
    m_query_type = describeQueryType(query);

    Schema& schema = getSchemaRef();
    schema = fetchSchema(query);

}




const Options Reader::getDefaultOptions() const
{
    Options options;

    Option connection("connection",
                      "",
                      "Oracle connection string to connect to database");

    Option query("query",
                 "",
                 "SELECT statement that returns an SDO_PC object \
                 as its first and only queried item.");

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
        
    } else if (m_database_type == DATABASE_POSTGRESQL)
    {
        block_fields.insert(SB("CLOUD_ID", false));
        block_fields.insert(SB("BLOCK_ID", false));
        block_fields.insert(SB("NUM_POINTS", false));
        block_fields.insert(SB("POINTS", false));
        block_fields.insert(SB("EXTENT", false));
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

// 
// pdal::SpatialReference Reader::fetchSpatialReference(Statement statement, sdo_pc* pc) const
// {
//     // Fetch the WKT for the SRID to set the coordinate system of this stage
//     int srid = statement->GetInteger(&(pc->pc_geometry.sdo_srid));
// 
//     std::ostringstream oss;
//     oss <<"EPSG:" << srid;
// 
//     if (srid)
//         return pdal::SpatialReference(oss.str());
//     else
//         return pdal::SpatialReference();
// }
// 
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

void Reader::addDefaultDimensions()
{

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
    : m_reader(reader)

{
   return;
}
// 
// Statement IteratorBase::getNextCloud(BlockPtr block, boost::int32_t& cloud_id)
// {
// 
//     std::ostringstream select_blocks;
//     BlockPtr cloud_block = m_reader.getBlock();
//     Statement cloud_statement = m_reader.getInitialQueryStatement();
// 
//     cloud_id = cloud_statement->GetInteger(&cloud_block->pc->pc_id);
//     std::string cloud_table = std::string(cloud_statement->GetString(cloud_block->pc->blk_table));
//     select_blocks
//             << "select T.OBJ_ID, T.BLK_ID, T.BLK_EXTENT, T.NUM_POINTS, T.POINTS from "
//             << cloud_table << " T WHERE T.OBJ_ID = "
//             << cloud_id;
// 
// 
//     Statement output = Statement(m_reader.getConnection()->CreateStatement(select_blocks.str().c_str()));
// 
//     output->Execute(0);
//     m_reader.defineBlock(output, block);
//     return output;

// }

IteratorBase::~IteratorBase()
{
}



const pdal::drivers::soci::Reader& IteratorBase::getReader() const
{
    return m_reader;
}

// void IteratorBase::readBlob(Statement statement,
//                             BlockPtr block,
//                             boost::uint32_t howMany)
// {
//     boost::uint32_t nAmountRead = 0;
//     boost::uint32_t nBlobLength = statement->GetBlobLength(block->locator);
// 
//     if (block->chunk->size() < nBlobLength)
//     {
//         block->chunk->resize(nBlobLength);
//     }
// 
//     getReader().log()->get(logDEBUG4) << "IteratorBase::readBlob expected nBlobLength: " << nBlobLength << std::endl;
// 
//     bool read_all_data = statement->ReadBlob(block->locator,
//                          (void*)(&(*block->chunk)[0]),
//                          block->chunk->size() ,
//                          &nAmountRead);
//     if (!read_all_data) throw pdal_error("Did not read all blob data!");
// 
//     getReader().log()->get(logDEBUG4) << "IteratorBase::readBlob actual nAmountRead: " << nAmountRead  << std::endl;
// 
//     Schema const& oracle_schema = m_oracle_buffer->getSchema();
// 
//     boost::uint32_t howMuchToRead = howMany * oracle_schema.getByteSize();
//     m_oracle_buffer->setDataStride(&(*block->chunk)[0], 0, howMuchToRead);
// 
//     m_oracle_buffer->setNumPoints(howMany);    
// }

// void IteratorBase::fillUserBuffer(PointBuffer& user_buffer)
// {
// 
//     Schema const& user_schema = user_buffer.getSchema();
//     schema::index_by_index const& idx = user_schema.getDimensions().get<schema::index>();
// 
//     boost::int32_t numUserSpace = user_buffer.getCapacity() - user_buffer.getNumPoints();
//     if (numUserSpace <= 0)
//         return;
//     boost::int32_t numOraclePoints = m_oracle_buffer->getNumPoints() - m_buffer_position;
//     
//     schema::index_by_index::size_type i(0);
//     for (i = 0; i < idx.size(); ++i)
//     {
//             copyOracleData( *m_oracle_buffer, 
//                             user_buffer, 
//                             idx[i], 
//                             m_buffer_position, 
//                             user_buffer.getNumPoints(), 
//                             (std::min)(numOraclePoints,numUserSpace));
// 
//     }
// 
//     bool bSetPointSourceId = getReader().getOptions().getValueOrDefault<bool>("populate_pointsourceid", false);
//     if (bSetPointSourceId)
//     {  
//         Dimension const* point_source_field = &(user_buffer.getSchema().getDimensionOptional("PointSourceId").get());
//         if (point_source_field)
//         {  
//             for (boost::int32_t i = 0; i < numUserSpace; ++i)
//             {  
//                 if (i < 0)
//                     throw pdal_error("point_source_field point index is less than 0!");
//                 user_buffer.setField(*point_source_field, i, m_active_cloud_id);
//             }
//         }
//     }
//         
//     if (numOraclePoints > numUserSpace)
//         m_buffer_position = m_buffer_position + numUserSpace;
//     else if (numOraclePoints < numUserSpace)
//         m_buffer_position = 0;
//     
//     boost::uint32_t howManyThisRead = (std::min)(numUserSpace, numOraclePoints);
//     user_buffer.setNumPoints(howManyThisRead + user_buffer.getNumPoints());
// }
// 
// void IteratorBase::copyOracleData(  PointBuffer& source, 
//                                     PointBuffer& destination, 
//                                     Dimension const& dest_dim, 
//                                     boost::uint32_t source_starting_position, 
//                                     boost::uint32_t destination_starting_position,
//                                     boost::uint32_t howMany)
// {
//     
//     boost::optional<Dimension const&> source_dim = source.getSchema().getDimensionOptional(dest_dim.getName());
//     
//     if (!source_dim)
//     {
//         return;
//     }
// 
//     for (boost::uint32_t i = 0; i < howMany; ++i)
//     {
//         if (dest_dim.getInterpretation() == source_dim->getInterpretation() &&
//             dest_dim.getByteSize() == source_dim->getByteSize() && 
//             pdal::Utils::compare_distance(dest_dim.getNumericScale(), source_dim->getNumericScale()) &&
//             pdal::Utils::compare_distance(dest_dim.getNumericOffset(), source_dim->getNumericOffset()) &&
//             dest_dim.getEndianness() == source_dim->getEndianness() 
//             )
//         {
//             // FIXME: This test could produce false positives
//             boost::uint8_t* source_position = source.getData(source_starting_position+i) + source_dim->getByteOffset();
//             boost::uint8_t* destination_position = destination.getData(destination_starting_position + i) + dest_dim.getByteOffset();
//             memcpy(destination_position, source_position, source_dim->getByteSize());
//         }
//         else
//         {
//             PointBuffer::scaleData( source, 
//                                     destination, 
//                                     *source_dim, 
//                                     dest_dim, 
//                                     source_starting_position + i,
//                                     destination_starting_position + i);
//         }
//     }
//     
// }
// 
// boost::uint32_t IteratorBase::myReadBuffer(PointBuffer& data)
// {
//     if (m_querytype == QUERY_SDO_PC)
//         return myReadClouds(data);
//     if (m_querytype == QUERY_SDO_BLK_PC_VIEW)
//         return myReadBlocks(data);
// 
//     return 0;
// }
// 
// boost::uint32_t IteratorBase::myReadClouds(PointBuffer& user_buffer)
// {
//     boost::uint32_t numRead(0);
// 
//     getReader().log()->get(logDEBUG2) << "Fetched buffer with cloud id: " << m_active_cloud_id << " for myReadClouds" << std::endl;
// 
//     bool bReadCloud(true);
//     while (bReadCloud)
//     {
//         m_oracle_buffer = fetchPointBuffer(m_initialQueryStatement, getReader().getBlock()->pc);
// 
//         boost::uint32_t numReadThisCloud = myReadBlocks(user_buffer);
//         numRead = numRead + numReadThisCloud;
// 
//         getReader().log()->get(logDEBUG2) << "Read " << numReadThisCloud << " points from myReadBlocks" << std::endl;
// 
//         if (m_at_end == true)
//         {
//             getReader().log()->get(logDEBUG2) << "At end of current block and trying to fetch another cloud " << std::endl;
// 
//             bReadCloud = getReader().getInitialQueryStatement()->Fetch();
// 
//             if (bReadCloud)
//             {
//                 getReader().log()->get(logDEBUG2) << "Fetched another cloud " << std::endl;
//                 m_block = BlockPtr(new Block(getReader().getConnection()));
//                 m_initialQueryStatement = getNextCloud(m_block, m_active_cloud_id);
//                 m_at_end = false;
//                 continue;
// 
//             }
//             return numRead;
//         }
//         else
//         {
//             getReader().log()->get(logDEBUG2) << "At end of current block and have no more blocks to fetch" << std::endl;
//             return numRead;
//         }
// 
// 
//     }
// 
//     return numRead;
// }
// 
// BufferPtr IteratorBase::fetchPointBuffer(Statement statement, sdo_pc* pc)
// {
//     boost::int32_t id = statement->GetInteger(&pc->pc_id);
//     BufferMap::const_iterator i = m_buffers.find(id);
// 
//     if (i != m_buffers.end())
//     {
//         getReader().log()->get(logDEBUG2) << "IteratorBase::fetchPointBuffer: found existing PointBuffer with id " << id << std::endl;
//         return i->second;
//     }
//     else
//     {
//         boost::uint32_t block_capacity(0);
//         Schema schema = m_reader.fetchSchema(statement, pc, block_capacity);
// 
//         // if (block_capacity > capacity)
//         // {
//         //     std::ostringstream oss;
//         //     oss << "Block capacity, " << block_capacity <<", is too large to fit in "
//         //         << "buffer of size " << capacity<<". Increase buffer capacity with writer's \"chunk_size\" option "
//         //         << "or increase the read buffer size";
//         //     throw buffer_too_small(oss.str());
//         // }
// 
//         BufferPtr output  = BufferPtr(new PointBuffer(schema, block_capacity));
//         std::pair<int, BufferPtr> p(id, output);
//         m_buffers.insert(p);
//         getReader().log()->get(logDEBUG2) << "IteratorBase::fetchPointBuffer: creating new PointBuffer with id " << id << std::endl;
// 
//         return p.second;
//     }
// }
// boost::uint32_t IteratorBase::myReadBlocks(PointBuffer& user_buffer)
// {
//     boost::uint32_t numPointsRead = 0;
// 
//     user_buffer.setNumPoints(0);
// 
//     bool bDidRead = false;
//     
//     if (!m_oracle_buffer) 
//     {
//         m_oracle_buffer = fetchPointBuffer(m_initialQueryStatement, m_block->pc);
//         boost::int32_t current_cloud_id(0);
//         current_cloud_id  = m_initialQueryStatement->GetInteger(&m_block->pc->pc_id);
//         m_active_cloud_id = current_cloud_id;
//     }
//     
//     // This shouldn't ever happen
//     if (m_block->num_points > static_cast<boost::int32_t>(m_oracle_buffer->getCapacity()))
//     {
//         std::ostringstream oss;
//         oss << "Block size, " << m_block->num_points <<", is too large to fit in "
//             << "buffer of size " << user_buffer.getCapacity() <<". Increase buffer capacity with writer's \"chunk_size\" option "
//             << "or increase the read buffer size";
//         throw buffer_too_small(oss.str());
//     }
// 
//     if (!m_block->num_points)
//     {
//         // We still have a block of data from the last readBuffer call
//         // that was partially read.
//         getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: fetching first block" << std::endl;
//         bDidRead = m_initialQueryStatement->Fetch();
//         if (!bDidRead)
//         {
//             m_at_end = true;
//             return 0;
//         }
// 
//         user_buffer.setSpatialBounds(getBounds(m_initialQueryStatement, m_block));
// 
//     }
//     else
//     {
//         // Our read was already "done" last readBuffer call, but if we're done,
//         // we're done
//         if (m_at_end)
//             getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: we are at end of the blocks;" << std::endl;
//         else
//             getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: we have points left to read on this block" << std::endl;
// 
//         if (m_at_end) return 0;
//         bDidRead = true;
// 
//     }
// 
//     while (bDidRead)
//     {
//         boost::uint32_t numReadThisBlock = m_block->num_points;
//         boost::uint32_t numSpaceLeftThisBuffer = user_buffer.getCapacity() - user_buffer.getNumPoints();
// 
//         getReader().log()->get(logDEBUG4) << "IteratorBase::myReadBlocks:" "numReadThisBlock: "
//                                           << numReadThisBlock << " numSpaceLeftThisBlock: "
//                                           << numSpaceLeftThisBuffer << " total numPointsRead: "
//                                           << numPointsRead << std::endl;
// 
//         numPointsRead = numPointsRead + numReadThisBlock;
// 
//         readBlob(m_initialQueryStatement, m_block, m_block->num_points);
//         fillUserBuffer(user_buffer);
//         if (m_buffer_position != 0)
//         {
//             return user_buffer.getNumPoints();
//         } 
//         else
//         {
//             bDidRead = m_initialQueryStatement->Fetch();
//             if (!bDidRead)
//             {
//                 getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: done reading block. Read " << numPointsRead << " points" << std::endl;
//                 m_at_end = true;
//                 return user_buffer.getNumPoints();
//             }
//         }
// 
// 
//         if (m_querytype == QUERY_SDO_BLK_PC_VIEW)
//         {
//             boost::int32_t current_cloud_id(0);
//             current_cloud_id  = m_initialQueryStatement->GetInteger(&m_block->pc->pc_id);
// 
//             getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: current_cloud_id: "
//                                               << current_cloud_id << " m_active_cloud_id: "
//                                               << m_active_cloud_id << std::endl;
// 
//             if (current_cloud_id != m_active_cloud_id)
//             {
//                 m_oracle_buffer = fetchPointBuffer(m_initialQueryStatement, m_block->pc);
// 
//                 m_active_cloud_id = current_cloud_id;
//                 return user_buffer.getNumPoints();
//             }
//         }
//     }
// 
// 
//     return numPointsRead;
// }
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
    return true;
    // return m_at_end;
}


boost::uint32_t Reader::readBufferImpl(PointBuffer& data)
{
    return data.getNumPoints();
    // return myReadBuffer(data);
}



}
} // iterators::sequential::

}
}
} // namespace pdal::driver::soci
