/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <libpc/drivers/oci/Iterator.hpp>
#include <libpc/Utils.hpp>

#include <liblas/factory.hpp>

#include <libpc/exceptions.hpp>
#include <libpc/PointBuffer.hpp>
#include <libpc/Utils.hpp>
#include <libpc/drivers/oci/Reader.hpp>
#include <libpc/Vector.hpp>

#include <sstream>
#include <map>
#include <algorithm>


namespace libpc { namespace drivers { namespace oci {

IteratorBase::IteratorBase(const Reader& reader)
    : m_statement(reader.getStatement())
    , m_at_end(false)
    , m_block(reader.getBlock())
    , m_reader(reader)

{
    // oci::Options& options = m_reader.getOptions();

    // m_reader.getConnection()->CreateType(&m_pc);
    
    // m_querytype = describeQueryType();

    // if (m_querytype == QUERY_SDO_PC)
    // {
    //     // m_statement->Define(&m_pc);
    //     // Unpack SDO_PC object to get at block 
    //     // table, select that stuff, and unpack the blocks
    // } 
    // else if (m_querytype == QUERY_BLK_TABLE)
    // {
    //     doBlockTableDefine();
    // }

    return;
}


IteratorBase::~IteratorBase()
{
}



const Reader& IteratorBase::getReader() const
{
    return m_reader;
}

boost::uint32_t IteratorBase::unpackOracleData(PointBuffer& data)
{
    boost::uint32_t capacity = data.getCapacity();
    boost::uint32_t numPoints = data.getNumPoints();
    
    boost::uint32_t space = capacity - numPoints;
    boost::uint32_t point_position = numPoints;
    
    if (m_block->num_points < 0)
    {
        std::ostringstream oss;
        oss << "This oracle block has a num_points that is negative (" << m_block->num_points <<")!";
        throw libpc_error(oss.str());
    }
    
    if (space < static_cast<boost::uint32_t>(m_block->num_points))
    {
        std::ostringstream oss;
        oss << "Not enough space to store this block!  The space left in the buffer is ";
        oss << space << ", the Oracle block has " << m_block->num_points << " points ";
        throw libpc_error(oss.str());
    }

    std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
    std::cout.precision(2);

    const Schema& schema = data.getSchema();
    const int indexX = schema.getDimensionIndex(Dimension::Field_X, Dimension::Int32);
    const int indexY = schema.getDimensionIndex(Dimension::Field_Y, Dimension::Int32);
    const int indexZ = schema.getDimensionIndex(Dimension::Field_Z, Dimension::Int32);
    const int indexTime = schema.getDimensionIndex(Dimension::Field_Time, Dimension::Double);
    const int indexClassification = schema.getDimensionIndex(Dimension::Field_Classification, Dimension::Uint8);

    const Dimension& dimX = schema.getDimension(indexX);
    const Dimension& dimY = schema.getDimension(indexY);
    const Dimension& dimZ = schema.getDimension(indexZ);
    
    double scalex = dimX.getNumericScale();
    double scaley = dimY.getNumericScale();
    double scalez = dimZ.getNumericScale();
    
    double offsetx = dimX.getNumericOffset();
    double offsety = dimY.getNumericOffset();
    double offsetz = dimZ.getNumericOffset();
     
    FiveDimensionOCI* d;
    for (boost::uint32_t i = 0; i < static_cast<boost::uint32_t>(m_block->num_points); i++)
    {
        boost::uint32_t byte_position = i*sizeof(FiveDimensionOCI);
        
        d = (FiveDimensionOCI*)(&(*m_block->chunk)[byte_position]);
        SWAP_BE_TO_LE(d->x);
        SWAP_BE_TO_LE(d->y);
        SWAP_BE_TO_LE(d->z);
        SWAP_BE_TO_LE(d->t);
        SWAP_BE_TO_LE(d->c);
        SWAP_BE_TO_LE(d->blk_id);
        SWAP_BE_TO_LE(d->pc_id);
        
        boost::int32_t x = static_cast<boost::int32_t>(
                         Utils::sround((d->x - offsetx) / scalex));
        boost::int32_t y = static_cast<boost::int32_t>(
                         Utils::sround((d->y - offsety) / scaley));
        boost::int32_t z = static_cast<boost::int32_t>(
                         Utils::sround((d->z - offsetz) / scalez));
        data.setField(point_position, indexX, x);
        data.setField(point_position, indexY, y);
        data.setField(point_position, indexZ, z);
        
        data.setField(point_position, indexTime, d->t);
        data.setField(point_position, indexClassification, static_cast<boost::uint8_t>(d->c));
        
        point_position++;
        data.setNumPoints(point_position);
    
    }
    return 0;
}

boost::uint32_t IteratorBase::readBuffer(PointBuffer& data)
{
    boost::uint32_t numPointsRead = 0;

    data.setNumPoints(0);
    
    bool bDidRead = false;


    if (!m_block->num_points) 
    {
        // We still have a block of data from the last readBuffer call
        // that was partially read. 
        // std::cout << "reading because we have no points" << std::endl;
        bDidRead = m_statement->Fetch();
        if (!bDidRead)
        {
            m_at_end = true;
            return 0;
        }
    
    } else 
    {
        // Our read was already "done" last readBuffer call, but if we're done,
        // we're done
        if (m_at_end) return 0;
        bDidRead = true;

    }
    
    while (bDidRead)
    {
        boost::uint32_t numReadThisBlock = m_block->num_points;
        if ((numPointsRead + numReadThisBlock) > (data.getCapacity() - data.getNumPoints()))
        {
            // We're done.  We still have more data, but the 
            // user is going to have to request another buffer.
            // We're not going to fill the buffer up to *exactly* 
            // the number of points the user requested.  
            // If the buffer's capacity isn't large enough to hold 
            // an oracle block, they're just not going to get anything 
            // back right now (FIXME)
            break;
        }

        numPointsRead = numPointsRead + numReadThisBlock;
        boost::uint32_t nAmountRead = 0;
    
    
        boost::uint32_t blob_length = m_statement->GetBlobLength(m_block->locator);
    
        if (m_block->chunk->size() < blob_length)
        {
            m_block->chunk->resize(blob_length);
        }
        
        bool read_all_data = m_statement->ReadBlob( m_block->locator,
                                         (void*)(&(*m_block->chunk)[0]),
                                         m_block->chunk->size() , 
                                         &nAmountRead);
        if (!read_all_data) throw libpc_error("Did not read all blob data!");

        unpackOracleData(data);
        bDidRead = m_statement->Fetch();
        if (!bDidRead)
        {
            m_at_end = true;
            return numPointsRead;
        }
    }    
    



    // double x, y, z;
    // 
    // boost::int32_t elem1, elem2, elem3;
    // m_statement->GetElement(&(m_block->blk_extent->sdo_elem_info), 0, &elem1);
    // m_statement->GetElement(&(m_block->blk_extent->sdo_elem_info), 1, &elem2);
    // m_statement->GetElement(&(m_block->blk_extent->sdo_elem_info), 2, &elem3);
    // 
    // boost::int32_t gtype, srid;
    // gtype= m_statement->GetInteger(&(m_block->blk_extent->sdo_gtype));
    // srid =m_statement->GetInteger(&(m_block->blk_extent->sdo_srid));

    
    // See http://download.oracle.com/docs/cd/B28359_01/appdev.111/b28400/sdo_objrelschema.htm#g1013735

    // boost::uint32_t dimension = gtype / 1000;
    // boost::uint32_t geom_type = gtype % 100;
    // boost::uint32_t referencing= ((gtype % 1000) / 100);
    // std::cout << "dimension: " << dimension << " geometry type: " << geom_type << " referencing " << referencing << std::endl;
    
    libpc::Vector<double> mins;
    libpc::Vector<double> maxs;
    
    boost::int32_t bounds_length = m_statement->GetArrayLength(&(m_block->blk_extent->sdo_ordinates));
    
    for (boost::int32_t i = 0; i < bounds_length; i = i + 2)
    {
        double v;
        m_statement->GetElement(&(m_block->blk_extent->sdo_ordinates), i, &v);
        mins.add(v);
        m_statement->GetElement(&(m_block->blk_extent->sdo_ordinates), i+1, &v);
        maxs.add(v);
    }
    
    libpc::Bounds<double> block_bounds(mins, maxs);
    
    data.setSpatialBounds(block_bounds);
    
    // std::cout << "srid: " << srid << std::endl;
    
    // std::cout << "elem1, elem2, elem3 " << elem1 << " " << elem2 << " " << elem3 << std::endl;
    
    // std::cout << "elem_info size " << m_statement->GetArrayLength(&(m_block->blk_extent->sdo_elem_info)) << std::endl;
    // std::cout << "sdo_ordinates size " << m_statement->GetArrayLength(&(m_block->blk_extent->sdo_ordinates)) << std::endl;

    // m_statement->GetElement(&(m_block->blk_extent->sdo_ordinates), 0, &x);
    // m_statement->GetElement(&(m_block->blk_extent->sdo_ordinates), 1, &y);
    // m_statement->GetElement(&(m_block->blk_extent->sdo_ordinates), 2, &z);

    // std::cout << "x, y, z " << x << " " << y << " " << z << std::endl;


    return numPointsRead;
}





//---------------------------------------------------------------------------
//
// SequentialIterator
//
//---------------------------------------------------------------------------

SequentialIterator::SequentialIterator(const Reader& reader)
    : IteratorBase(reader)
    , libpc::SequentialIterator(reader)
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
    //     throw libpc_error("cannot support seek offsets greater than 32-bits");
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
    // return getIndex() >= getStage().getNumPoints();
}


boost::uint32_t SequentialIterator::readImpl(PointBuffer& data)
{
    return readBuffer(data);
}


//---------------------------------------------------------------------------
//
// RandomIterator
//
//---------------------------------------------------------------------------


} } } // namespaces
