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

#include <pdal/drivers/oci/Iterator.hpp>

#include <pdal/PointBuffer.hpp>
#include <pdal/drivers/oci/Reader.hpp>
#include <pdal/Vector.hpp>

#include <boost/algorithm/string.hpp>


#include <sstream>
#include <map>
#include <algorithm>


namespace pdal { namespace drivers { namespace oci {

IteratorBase::IteratorBase(const Reader& reader)
    : m_statement(Statement())
    , m_at_end(false)
    , m_block(BlockPtr(new Block(reader.getConnection())))
    , m_active_cloud_id(0)
    , m_new_buffer(BufferPtr())
    , bGetNewBuffer(false)
    , bReadFirstCloud(true)
    , m_reader(reader)

{
    
    m_querytype = reader.getQueryType();
    
    if (m_querytype == QUERY_SDO_PC)
    {
        m_statement = getNextCloud(m_block, m_active_cloud_id);
    }

    if (m_querytype == QUERY_SDO_BLK_PC_VIEW)
    {
        m_statement = reader.getStatement();
        m_block = reader.getBlock();
        
        m_active_cloud_id = m_statement->GetInteger(&m_block->pc->pc_id);
    }
    
    return;
}

Statement IteratorBase::getNextCloud(BlockPtr block, boost::int32_t& cloud_id)
{

    std::ostringstream select_blocks;
    BlockPtr cloud_block = m_reader.getBlock();
    Statement cloud_statement = m_reader.getStatement();
    
    // bool bDidRead(true);
    // if(!bReadFirstCloud)
    //     bDidRead = cloud_statement->Fetch();
    // bReadFirstCloud = false;
    // if (!bDidRead) return Statement();
    
    cloud_id = cloud_statement->GetInteger(&cloud_block->pc->pc_id);
    std::string cloud_table = std::string(cloud_statement->GetString(cloud_block->pc->blk_table));
    select_blocks
        << "select T.OBJ_ID, T.BLK_ID, T.BLK_EXTENT, T.NUM_POINTS, T.POINTS from " 
        << cloud_table << " T WHERE T.OBJ_ID = " 
        << cloud_id;


    Statement output = Statement(m_reader.getConnection()->CreateStatement(select_blocks.str().c_str()));

    output->Execute(0);
    m_reader.defineBlock(output, block);
    return output;
    
}

IteratorBase::~IteratorBase()
{
}



const Reader& IteratorBase::getReader() const
{
    return m_reader;
}

void IteratorBase::read(PointBuffer& data,
                        Statement statement,
                        BlockPtr block,
                        boost::uint32_t howMany, 
                        boost::uint32_t whichPoint, 
                        boost::uint32_t whichBlobPosition)
{
    
    boost::uint32_t nAmountRead = 0;
    boost::uint32_t nBlobLength = statement->GetBlobLength(block->locator);

    if (block->chunk->size() < nBlobLength)
    {
        block->chunk->resize(nBlobLength);
    }
    
    getReader().log()->get(logDEBUG4) << "IteratorBase::read expected nBlobLength: " << nBlobLength << std::endl;
    
    bool read_all_data = statement->ReadBlob( block->locator,
                                     (void*)(&(*block->chunk)[0]),
                                     block->chunk->size() , 
                                     &nAmountRead);
    if (!read_all_data) throw pdal_error("Did not read all blob data!");

    getReader().log()->get(logDEBUG4) << "IteratorBase::read actual nAmountRead: " << nAmountRead  << std::endl;
    
    data.getSchema().getByteSize();
    boost::uint32_t howMuchToRead = howMany * data.getSchema().getByteSize();
    data.setDataStride(&(*block->chunk)[whichBlobPosition], whichPoint, howMuchToRead);

    data.setNumPoints(data.getNumPoints() + howMany);

}

boost::uint32_t IteratorBase::myReadBuffer(PointBuffer& data)
{
    if (m_querytype == QUERY_SDO_PC)
        return myReadClouds(data);
    if (m_querytype == QUERY_SDO_BLK_PC_VIEW)
        return myReadBlocks(data);
    
    return 0;
}

boost::uint32_t IteratorBase::myReadClouds(PointBuffer& data)
{
    boost::uint32_t numRead(0);
    
    getReader().log()->get(logDEBUG2) << "Fetched buffer with cloud id: " << m_active_cloud_id << " for myReadClouds" << std::endl;
    
    bool bReadCloud(true);
    while( bReadCloud) 
    {
        m_new_buffer = fetchPointBuffer(m_statement, getReader().getBlock()->pc, data.getCapacity());

        bGetNewBuffer = true;
            
        boost::uint32_t numReadThisCloud = myReadBlocks(data);
        numRead = numRead + numReadThisCloud;
        
        getReader().log()->get(logDEBUG2) << "Read " << numReadThisCloud << " points from myReadBlocks" << std::endl;
        
        bReadCloud = getReader().getStatement()->Fetch();
        m_block = BlockPtr(new Block(getReader().getConnection()));
        m_statement = getNextCloud(m_block, m_active_cloud_id);
        if (m_at_end == true && bReadCloud) 
        {
            getReader().log()->get(logDEBUG2) << "At end of current block and have another cloud to fetch" << std::endl;
            m_at_end = false;
            return numRead;
        }
        else if (m_at_end == true && !bReadCloud)
        {
            getReader().log()->get(logDEBUG2) << "At end of current block and have no more blocks to fetch" << std::endl;
            return numRead;
        }
        else {
            throw pdal_error("don't know WTF!");
        }

    }

    return numRead;
}

BufferPtr IteratorBase::fetchPointBuffer(Statement statement, sdo_pc* pc, boost::uint32_t capacity)
{
    boost::int32_t id = m_statement->GetInteger(&pc->pc_id);
    BufferMap::const_iterator i = m_buffers.find(id);
    
    if (i != m_buffers.end())
    {
        getReader().log()->get(logDEBUG2) << "IteratorBase::fetchPointBuffer: found existing PointBuffer with id " << id << std::endl;
        return i->second;
    } else {
        boost::uint32_t cap(0);
        Schema schema = m_reader.fetchSchema(statement, pc, cap);
        

        BufferPtr output  = BufferPtr(new PointBuffer(schema, (std::max)(capacity, cap)));
        std::pair<int, BufferPtr> p(id, output);
        m_buffers.insert(p);
        getReader().log()->get(logDEBUG2) << "IteratorBase::fetchPointBuffer: creating new PointBuffer with id " << id << std::endl;

        return p.second;
    }    
}
boost::uint32_t IteratorBase::myReadBlocks(PointBuffer& data)
{
    boost::uint32_t numPointsRead = 0;
    
    if (bGetNewBuffer)
    {
        
        getReader().log()->get(logDEBUG2) << "IteratorBase::myReadBlocks: Switching buffer to m_new_buffer with id " << m_active_cloud_id << std::endl;
        data = *m_new_buffer;
        bGetNewBuffer = false;
    }
    data.setNumPoints(0);
    
    bool bDidRead = false;




    getReader().log()->get(logDEBUG4) << "IteratorBase::myReadBlocks: m_block->num_points: " << m_block->num_points << std::endl;

    getReader().log()->get(logDEBUG4) << "IteratorBase::myReadBlocks: data.getCapacity(): " << data.getCapacity() << std::endl;
        
    if (!m_block->num_points) 
    {
        // We still have a block of data from the last readBuffer call
        // that was partially read. 
        getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: fetching first block" << std::endl;
        bDidRead = m_statement->Fetch();        
        if (!bDidRead)
        {
            m_at_end = true;
            return 0;
        }
        
        data.setSpatialBounds(getBounds(m_statement, m_block));
 
        if (m_block->num_points > static_cast<boost::int32_t>(data.getCapacity()))
        {
            throw buffer_too_small("The PointBuffer is too small to contain this block.");
        }
    
    } else 
    {
        // Our read was already "done" last readBuffer call, but if we're done,
        // we're done
        if (m_at_end)
            getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: we are at end of the blocks;" << std::endl;
        else
            getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: we have points left to read on this block" << std::endl;
        
        if (m_at_end) return 0;
        bDidRead = true;

    }
    
    while (bDidRead)
    {
        boost::uint32_t numReadThisBlock = m_block->num_points;
        boost::uint32_t numSpaceLeftThisBlock = data.getCapacity() - data.getNumPoints();

        getReader().log()->get(logDEBUG4) << "IteratorBase::myReadBlocks:" "numReadThisBlock: " 
                                          << numReadThisBlock << " numSpaceLeftThisBlock: " 
                                          << numSpaceLeftThisBlock << " total numPointsRead: " 
                                          << numPointsRead << std::endl;

        if (numReadThisBlock > numSpaceLeftThisBlock)
        {
            // We're done.  We still have more data, but the 
            // user is going to have to request another buffer.
            // We're not going to fill the buffer up to *exactly* 
            // the number of points the user requested.  
            // If the buffer's capacity isn't large enough to hold 
            // an oracle block, they're just not going to get anything 
            // back right now (FIXME)
            getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: numReadThisBlock > numSpaceLeftThisBlock. Coming back around." << std::endl;
            break;
            
        }

        numPointsRead = numPointsRead + numReadThisBlock;
        
        read(data, m_statement, m_block, numReadThisBlock, data.getNumPoints(), 0);
        
        bDidRead = m_statement->Fetch();
        if (!bDidRead)
        {
            getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: done reading block. Read " << numPointsRead << " points" << std::endl;
            m_at_end = true;
            return numPointsRead;
        }

       
        if (m_querytype == QUERY_SDO_BLK_PC_VIEW)
        {
            boost::int32_t current_cloud_id(0);
            current_cloud_id  = m_statement->GetInteger(&m_block->pc->pc_id);

            getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: current_cloud_id: " 
                                              << current_cloud_id << " m_active_cloud_id: " 
                                              << m_active_cloud_id << std::endl;
            
            if (current_cloud_id != m_active_cloud_id)
            {
                m_new_buffer = fetchPointBuffer(m_statement, m_block->pc, data.getCapacity());

                bGetNewBuffer = true;
                m_active_cloud_id = current_cloud_id;
                return numPointsRead;
            }
        }
    }

    
    return numPointsRead;
}

pdal::Bounds<double> IteratorBase::getBounds(Statement statement, BlockPtr block)
{
    pdal::Vector<double> mins;
    pdal::Vector<double> maxs;
    
    boost::int32_t bounds_length = statement->GetArrayLength(&(block->blk_extent->sdo_ordinates));

    getReader().log()->get(logDEBUG3) << "IteratorBase::getBounds: bounds length " << bounds_length << std::endl;

    for (boost::int32_t i = 0; i < bounds_length; i = i + 2)
    {
        double v;
        statement->GetElement(&(block->blk_extent->sdo_ordinates), i, &v);
        mins.add(v);
        statement->GetElement(&(block->blk_extent->sdo_ordinates), i+1, &v);
        maxs.add(v);
    }
    
    pdal::Bounds<double> block_bounds(mins, maxs);

    getReader().log()->get(logDEBUG2) << "IteratorBase::getBounds: Fetched bounds of " << block_bounds << std::endl;
    return block_bounds;    
}



//---------------------------------------------------------------------------
//
// SequentialIterator
//
//---------------------------------------------------------------------------

SequentialIterator::SequentialIterator(const Reader& reader)
    : IteratorBase(reader)
    , pdal::StageSequentialIterator(reader)
{
    return;
}


SequentialIterator::~SequentialIterator()
{
    return;
}


boost::uint64_t SequentialIterator::skipImpl(boost::uint64_t count)
{
    // const boost::uint64_t newPos64 = getIndex() + count;
    // 
    // // The liblas reader's seek() call only supports size_t, so we might
    // // not be able to satisfy this request...
    // 
    // if (newPos64 > std::numeric_limits<size_t>::max())
    // {
    //     throw pdal_error("cannot support seek offsets greater than 32-bits");
    // }
    // 
    // // safe cast, since we just handled the overflow case
    // size_t newPos = static_cast<size_t>(newPos64);
    // 
    // getExternalReader().Seek(newPos);

    return 0;
}


bool SequentialIterator::atEndImpl() const
{
    return m_at_end;
}


boost::uint32_t SequentialIterator::readBufferImpl(PointBuffer& data)
{
    return myReadBuffer(data);
}


//---------------------------------------------------------------------------
//
// RandomIterator
//
//---------------------------------------------------------------------------


} } } // namespaces
