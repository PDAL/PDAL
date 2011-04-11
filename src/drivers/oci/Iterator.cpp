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
    : m_at_end(false)
    , m_block(new Block(reader.getConnection()))
    , m_reader(reader)

{
    // oci::Options& options = m_reader.getOptions();

    // m_reader.getConnection()->CreateType(&m_pc);
    
    m_statement = Statement(m_reader.getConnection()->CreateStatement(m_reader.getQuery().c_str()));
    
    m_statement->Execute(0);
    m_querytype = describeQueryType();

    if (m_querytype == QUERY_SDO_PC)
    {
        m_statement->Define(&m_pc);
        // Unpack SDO_PC object to get at block 
        // table, select that stuff, and unpack the blocks
    } 
    else if (m_querytype == QUERY_BLK_TABLE)
    {
        doBlockTableDefine();
    }

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

    // if (getReader().isVerbose())
    //     std::cout << " Existing block has " << m_block->num_points << " points" << std::endl;
    
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

        unpackOracleData(data);
    }
    
    while (bDidRead)
    {
        boost::uint32_t numReadThisBlock = m_block->num_points;
        if (numPointsRead + numReadThisBlock > (data.getCapacity() - data.getNumPoints()))
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
    
    
        boost::uint32_t blob_length = m_statement->GetBlobLength(m_locator);
    
        if (m_block->chunk->size() < blob_length)
        {
            m_block->chunk->resize(blob_length);
        }
        
        bool read_all_data = m_statement->ReadBlob( m_locator,
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


void IteratorBase::doBlockTableDefine()
{

    int   iCol = 0;
    char  szFieldName[OWNAME];
    int   hType = 0;
    int   nSize = 0;
    int   nPrecision = 0;
    signed short nScale = 0;
    char szTypeName[OWNAME];
    





    // block_columns[0] = "OBJ_ID";
    // block_columns[1] = "BLK_ID";
    // block_columns[2] = "BLK_EXTENT";
    // block_columns[3] = "BLK_DOMAIN";
    // block_columns[4] = "PCBLK_MIN_RES";
    // block_columns[5] = "PCBLK_MAX_RES";
    // block_columns[6] = "NUM_POINTS";
    // block_columns[7] = "NUM_UNSORTED_POINTS";
    // block_columns[8] = "PT_SORT_DIM";
    // block_columns[9] = "POINTS";
    

    
    while( m_statement->GetNextField(iCol, szFieldName, &hType, &nSize, &nPrecision, &nScale, szTypeName) )
    {
        std::string name = to_upper(std::string(szFieldName));

        if (compare_no_case(szFieldName, "OBJ_ID", 6) == 0)
        {
            m_statement->Define(&(m_block->obj_id));
        }

        if (compare_no_case(szFieldName, "BLK_ID", 6) == 0)
        {
            m_statement->Define(&(m_block->blk_id));
        }

        if (compare_no_case(szFieldName, "BLK_EXTENT", 10) == 0)
        {
            m_statement->Define(&(m_block->blk_extent));
        }

        if (compare_no_case(szFieldName, "BLK_DOMAIN", 10) == 0)
        {
            m_statement->Define(&(m_block->blk_domain));
        }
        
        if (compare_no_case(szFieldName, "PCBLK_MIN_RES", 13) == 0)
        {
            m_statement->Define(&(m_block->pcblk_min_res));
        }

        if (compare_no_case(szFieldName, "PCBLK_MAX_RES", 13) == 0)
        {
            m_statement->Define(&(m_block->pcblk_max_res));
        }

        if (compare_no_case(szFieldName, "NUM_POINTS", 10) == 0)
        {
            m_statement->Define(&(m_block->num_points));
        }

        if (compare_no_case(szFieldName, "NUM_UNSORTED_POINTS", 19) == 0)
        {
            m_statement->Define(&(m_block->num_unsorted_points));
        }

        if (compare_no_case(szFieldName, "PT_SORT_DIM", 11) == 0)
        {
            m_statement->Define(&(m_block->pt_sort_dim));
        }

        if (compare_no_case(szFieldName, "POINTS", 6) == 0)
        {
            std::cout << "Defined POINTS as BLOB" << std::endl;
            m_statement->Define( &m_locator ); 
        }
        iCol++;
    }    
}

QueryType IteratorBase::describeQueryType() const
{


    int   iCol = 0;
    char  szFieldName[OWNAME];
    int   hType = 0;
    int   nSize = 0;
    int   nPrecision = 0;
    signed short nScale = 0;
    char szTypeName[OWNAME];
    
    bool isPCObject = false;
    bool isBlockTableQuery = false;
    bool isBlockTableType = false;
    
    
    const int columns_size = 10;
    std::string block_columns[columns_size];
    block_columns[0] = "OBJ_ID";
    block_columns[1] = "BLK_ID";
    block_columns[2] = "BLK_EXTENT";
    block_columns[3] = "BLK_DOMAIN";
    block_columns[4] = "PCBLK_MIN_RES";
    block_columns[5] = "PCBLK_MAX_RES";
    block_columns[6] = "NUM_POINTS";
    block_columns[7] = "NUM_UNSORTED_POINTS";
    block_columns[8] = "PT_SORT_DIM";
    block_columns[9] = "POINTS";
    
    std::map<std::string, bool> columns_map;

    for(int i = 0; i < columns_size; ++i)
    {
        columns_map.insert(std::pair<std::string, bool>(block_columns[i], false));
    }
    
    while( m_statement->GetNextField(iCol, szFieldName, &hType, &nSize, &nPrecision, &nScale, szTypeName) )
    {
        std::string name = to_upper(std::string(szFieldName));
        
        std::map<std::string, bool>::iterator it = columns_map.find(name);
        if (it != columns_map.end())
        {
            // std::cout << "setting columns to true for " << it->first << std::endl;
            (*it).second = true;
            
        }

        switch( hType )
        {
            case SQLT_FLT:
                std::cout << "Field " << szFieldName << " is SQL_FLT" << std::endl;
                break;
            case SQLT_NUM:
                if( nPrecision == 0 )
                {
                    std::cout << "Field " << szFieldName << " is SQLT_NUM with precision 0" << std::endl;
                }
                else
                {
                    std::cout << "Field " << szFieldName << " is SQLT_NUM with precision " << nPrecision << std::endl;

                }
                break;
            
            case SQLT_BLOB:
                std::cout << "Field " << szFieldName << " is SQLT_BLOB" << std::endl;
                break;
                
            case SQLT_NTY:
                if (compare_no_case(szTypeName, "SDO_PC", 6) == 0)
                    isPCObject = true;
                if (compare_no_case(szTypeName, "SDO_PC_BLK_TYPE", 6) == 0)
                    isBlockTableType = true;
                std::cout << "Field " << szFieldName << " is SQLT_NTY with type name " << szTypeName  << std::endl;
                break;
                
            case SQLT_AFC:
            case SQLT_CHR:
                std::cout << "Field " << szFieldName << " is SQLT_CHR" << std::endl;
                break;
                


            case SQLT_DAT:
            case SQLT_DATE:
            case SQLT_TIMESTAMP:
            case SQLT_TIMESTAMP_TZ:
            case SQLT_TIMESTAMP_LTZ:
            case SQLT_TIME:
            case SQLT_TIME_TZ:
                std::cout << "Field " << szFieldName << " is some kind of time type" << std::endl;

                break;
            default:
                std::ostringstream oss;
                oss << "Field " << szFieldName << " with type " << hType << " is not handled by fetchPCFields";
                throw libpc_error(oss.str());
        }

        iCol++;
    }

    // Assume we're a block table until we say we aren't.  Loop through all of 
    // the required columns that make up a block table and if we find one that 
    // wasn't marked in the loop above, we're not a block table.
    isBlockTableQuery = true;
    std::map<std::string, bool>::iterator it = columns_map.begin();
    while (it != columns_map.end())
    {   
        if (it->second == false) 
        {
            isBlockTableQuery = false; 
            break;
        }
        ++it;
    }
    
    // If we have all of the block table columns + some extras, we aren't a block table for now
    if (iCol != 10 && isBlockTableQuery) {
        isBlockTableQuery = false;
    }
    
    if (!isBlockTableQuery && !isPCObject) 
    {
        std::ostringstream oss;
        oss << "Select statement '" << m_reader.getQuery() << "' does not fetch an SDO_PC object" 
              " or one that is equivalent to SDO_PC_BLK_TYPE";
        throw libpc_error(oss.str());
    }

    if (isBlockTableQuery) 
        return QUERY_BLK_TABLE;
    
    if (isPCObject)
        return QUERY_SDO_PC;
    
    if (isBlockTableType)
        return QUERY_SDO_PC_BLK;
    
    return QUERY_UNKNOWN;
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
