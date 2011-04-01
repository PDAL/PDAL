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

#include <liblas/factory.hpp>

#include <libpc/exceptions.hpp>
#include <libpc/PointBuffer.hpp>
#include <libpc/Utils.hpp>
#include <libpc/drivers/oci/Reader.hpp>

#include <sstream>
#include <map>


namespace libpc { namespace drivers { namespace oci {

IteratorBase::IteratorBase(const Reader& reader)
    : m_at_end(false)
    , m_reader(reader)

{
    // oci::Options& options = m_reader.getOptions();

    m_reader.getConnection()->CreateType(&m_pc);
    
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
    
    // setNumPoints(1000);
    
    return;
}


IteratorBase::~IteratorBase()
{
}



const Reader& IteratorBase::getReader() const
{
    return m_reader;
}


boost::uint32_t IteratorBase::readBuffer(PointBuffer& data)
{
    boost::uint32_t numPoints = data.getCapacity();

    bool bDidRead = m_statement->Fetch();
    if (!bDidRead)
    {
        m_at_end = true;
        return 0;
    }
    std::cout << "This block has " << m_block->num_points << " points" << std::endl;
    
    boost::uint32_t nAmountRead;
    
    std::vector<boost::uint8_t> chunk;// = iterator.getChunk();
    
    boost::uint32_t blob_length = m_statement->GetBlobLength(m_locator);
    
    if (chunk.size() < blob_length)
    {
        chunk.resize(blob_length);
    }
    bool read_all_data = m_statement->ReadBlob( m_locator,
                                     (void*)(&chunk[0]),
                                     chunk.size(), 
                                     &nAmountRead);
    if (!read_all_data) throw libpc_error("Did not read all blob data!");
    std::cout << "nAmountRead: " << nAmountRead << std::endl;



    
    // bool read = m_reader.fetchNext();
    // Block* block = m_reader.getBlock();
    // 
    // if (!read)
    // {
    //     m_at_end = true;
    //     return 0;
    // }
        std::cout << "fetched" << std::endl;
    // boost::uint32_t i = 0;
    // 
    // const Schema& schema = data.getSchema();
    // 
    // const int indexX = schema.getDimensionIndex(Dimension::Field_X, Dimension::Int32);
    // const int indexY = schema.getDimensionIndex(Dimension::Field_Y, Dimension::Int32);
    // const int indexZ = schema.getDimensionIndex(Dimension::Field_Z, Dimension::Int32);
    // 
    // const int indexIntensity = schema.getDimensionIndex(Dimension::Field_Intensity, Dimension::Uint16);
    // const int indexReturnNumber = schema.getDimensionIndex(Dimension::Field_ReturnNumber, Dimension::Uint8);
    // const int indexNumberOfReturns = schema.getDimensionIndex(Dimension::Field_NumberOfReturns, Dimension::Uint8);
    // const int indexScanDirectionFlag = schema.getDimensionIndex(Dimension::Field_ScanDirectionFlag, Dimension::Uint8);
    // const int indexEdgeOfFlightLine = schema.getDimensionIndex(Dimension::Field_EdgeOfFlightLine, Dimension::Uint8);
    // const int indexClassification = schema.getDimensionIndex(Dimension::Field_Classification, Dimension::Uint8);
    // const int indexScanAngleRank = schema.getDimensionIndex(Dimension::Field_ScanAngleRank, Dimension::Int8);
    // const int indexUserData = schema.getDimensionIndex(Dimension::Field_UserData, Dimension::Uint8);
    // const int indexPointSourceId = schema.getDimensionIndex(Dimension::Field_PointSourceId, Dimension::Uint16);
    // 
    // const int indexTime = (getReader().hasTimeData() ? schema.getDimensionIndex(Dimension::Field_Time, Dimension::Double) : 0);
    // 
    // const int indexRed = (getReader().hasColorData() ? schema.getDimensionIndex(Dimension::Field_Red, Dimension::Uint16) : 0);
    // const int indexGreen = (getReader().hasColorData() ? schema.getDimensionIndex(Dimension::Field_Green, Dimension::Uint16) : 0);
    // const int indexBlue = (getReader().hasColorData() ? schema.getDimensionIndex(Dimension::Field_Blue, Dimension::Uint16) : 0);
    // 
    // //const int indexWavePacketDescriptorIndex = (m_hasWaveData ? schema.getDimensionIndex(Dimension::Field_WavePacketDescriptorIndex) : 0);
    // //const int indexWaveformDataOffset = (m_hasWaveData ? schema.getDimensionIndex(Dimension::Field_WaveformDataOffset) : 0);
    // //const int indexReturnPointWaveformLocation = (m_hasWaveData ? schema.getDimensionIndex(Dimension::Field_ReturnPointWaveformLocation) : 0);
    // //const int indexWaveformXt = (m_hasWaveData ? schema.getDimensionIndex(Dimension::Field_WaveformXt) : 0);
    // //const int indexWaveformYt = (m_hasWaveData ? schema.getDimensionIndex(Dimension::Field_WaveformYt) : 0);
    // //const t indexWaveformZt = (m_hasWaveData ? schema.getDimensionIndex(Dimension::Field_WaveformZt) : 0);
    // 
    // for (i=0; i<numPoints; i++)
    // {
    //     bool ok = getExternalReader().ReadNextPoint();
    //     if (!ok)
    //     {
    //         throw libpc_error("liblas reader failed to retrieve point");
    //     }
    // 
    //     const ::liblas::Point& pt = getExternalReader().GetPoint();
    // 
    //     const boost::int32_t x = pt.GetRawX();
    //     const boost::int32_t y = pt.GetRawY();
    //     const boost::int32_t z = pt.GetRawZ();
    // 
    //     const boost::uint16_t intensity = pt.GetIntensity();
    //     const boost::int8_t returnNumber = (boost::int8_t)pt.GetReturnNumber();
    //     const boost::int8_t numberOfReturns = (boost::int8_t)pt.GetNumberOfReturns();
    //     const boost::int8_t scanDirFlag = (boost::int8_t)pt.GetScanDirection();
    //     const boost::int8_t edgeOfFlightLine = (boost::int8_t)pt.GetFlightLineEdge();
    //     const boost::uint8_t classification = pt.GetClassification().GetClass();
    //     const boost::int8_t scanAngleRank = pt.GetScanAngleRank();
    //     const boost::uint8_t userData = pt.GetUserData();
    //     const boost::uint16_t pointSourceId = pt.GetPointSourceID();
    //     
    //     data.setField(i, indexX, x);
    //     data.setField(i, indexY, y);
    //     data.setField(i, indexZ, z);
    // 
    //     data.setField(i, indexIntensity, intensity);
    //     data.setField(i, indexReturnNumber, returnNumber);
    //     data.setField(i, indexNumberOfReturns, numberOfReturns);
    //     data.setField(i, indexScanDirectionFlag, scanDirFlag);
    //     data.setField(i, indexEdgeOfFlightLine, edgeOfFlightLine);
    //     data.setField(i, indexClassification, classification);
    //     data.setField(i, indexScanAngleRank, scanAngleRank);
    //     data.setField(i, indexUserData, userData);
    //     data.setField(i, indexPointSourceId, pointSourceId);
    // 
    //     if (getReader().hasTimeData())
    //     {
    //         const double time = pt.GetTime();
    //         
    //         data.setField(i, indexTime, time);
    //     }
    // 
    //     if (getReader().hasColorData())
    //     {
    //         const ::liblas::Color color = pt.GetColor();
    //         const boost::uint16_t red = color.GetRed();
    //         const boost::uint16_t green = color.GetGreen();
    //         const boost::uint16_t blue = color.GetBlue();
    // 
    //         data.setField(i, indexRed, red);
    //         data.setField(i, indexGreen, green);
    //         data.setField(i, indexBlue, blue);
    //     }
    //     
    //     data.setNumPoints(i+1);
    //     if (getReader().hasWaveData())
    //     {
    //         throw not_yet_implemented("Waveform data (types 4 and 5) not supported");
    //     }
    //     
    // }

    return numPoints;
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

    m_block = new Block;

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
    
    m_reader.getConnection()->CreateType(&(m_block->blk_extent));
    m_reader.getConnection()->CreateType(&(m_block->blk_domain));
    
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
    if (iCol != 11 && isBlockTableQuery) {
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
