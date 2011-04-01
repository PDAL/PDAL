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

#include <libpc/drivers/oci/Reader.hpp>
#include <libpc/drivers/oci/Iterator.hpp>

#include <libpc/exceptions.hpp>

#include <iostream>
#include <sstream>
#include <map>

namespace libpc { namespace drivers { namespace oci {


Reader::Reader(Options& options)
    : libpc::Stage()
    , m_options(options)
    , m_verbose(false)
    , m_qtype(QUERY_UNKNOWN)
{

    Debug();
    
    m_connection = Connect(m_options);

    
    std::string sql = options.GetPTree().get<std::string>("select_sql");
    
    if (sql.size() == 0 )
        throw libpc_error("'select_sql' statement is empty. No data can be read from libpc::drivers::oci::Reader");
    
    m_statement = Statement(m_connection->CreateStatement(sql.c_str()));
    
    registerFields();
    
    
    
    m_statement->Execute(0);

    m_qtype = describeQueryType();
    
    // int foo;
    // int block_id;
    // 
    // m_statement->Define(&foo);
    // m_statement->Define(&block_id);

    m_connection->CreateType(&m_pc);
    

    if (m_qtype == QUERY_SDO_PC)
    {
        m_statement->Define(&m_pc);
        // Unpack SDO_PC object to get at block 
        // table, select that stuff, and unpack the blocks
    } 
    else if (m_qtype == QUERY_BLK_TABLE)
    {
        doBlockTableDefine();
    }

    setNumPoints(1000);



    
}    

bool Reader::fetchNext() const
{
    bool bDidRead = m_statement->Fetch();
    if (!bDidRead) return false;
    
    std::cout << "This block has " << m_block_table->num_points << " points" << std::endl;
    
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
    
    return bDidRead;
}

void Reader::doBlockTableDefine()
{

    int   iCol = 0;
    char  szFieldName[OWNAME];
    int   hType = 0;
    int   nSize = 0;
    int   nPrecision = 0;
    signed short nScale = 0;
    char szTypeName[OWNAME];

    m_block_table = new Block;

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
    
    m_connection->CreateType(&(m_block_table->blk_extent));
    m_connection->CreateType(&(m_block_table->blk_domain));
    
    while( m_statement->GetNextField(iCol, szFieldName, &hType, &nSize, &nPrecision, &nScale, szTypeName) )
    {
        std::string name = to_upper(std::string(szFieldName));

        if (compare_no_case(szFieldName, "OBJ_ID", 6) == 0)
        {
            m_statement->Define(&(m_block_table->obj_id));
        }

        if (compare_no_case(szFieldName, "BLK_ID", 6) == 0)
        {
            m_statement->Define(&(m_block_table->blk_id));
        }

        if (compare_no_case(szFieldName, "BLK_EXTENT", 10) == 0)
        {
            m_statement->Define(&(m_block_table->blk_extent));
        }

        if (compare_no_case(szFieldName, "BLK_DOMAIN", 10) == 0)
        {
            m_statement->Define(&(m_block_table->blk_domain));
        }
        
        if (compare_no_case(szFieldName, "PCBLK_MIN_RES", 13) == 0)
        {
            m_statement->Define(&(m_block_table->pcblk_min_res));
        }

        if (compare_no_case(szFieldName, "PCBLK_MAX_RES", 13) == 0)
        {
            m_statement->Define(&(m_block_table->pcblk_max_res));
        }

        if (compare_no_case(szFieldName, "NUM_POINTS", 10) == 0)
        {
            m_statement->Define(&(m_block_table->num_points));
        }

        if (compare_no_case(szFieldName, "NUM_UNSORTED_POINTS", 19) == 0)
        {
            m_statement->Define(&(m_block_table->num_unsorted_points));
        }

        if (compare_no_case(szFieldName, "PT_SORT_DIM", 11) == 0)
        {
            m_statement->Define(&(m_block_table->pt_sort_dim));
        }

        if (compare_no_case(szFieldName, "POINTS", 6) == 0)
        {
            std::cout << "Defined POINTS as BLOB" << std::endl;
            m_statement->Define( &m_locator ); 
        }
        iCol++;
    }    
}


Reader::QueryType Reader::describeQueryType()
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
        std::string sql = m_options.GetPTree().get<std::string>("select_sql");
        oss << "Select statement '" << sql << "' does not fetch an SDO_PC object" 
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
void Reader::Debug()
{
    bool debug = m_options.IsDebug();

    if (debug)
    {
        m_verbose = true;
    }

    CPLPopErrorHandler();

    if (debug)
    {
        const char* gdal_debug = Utils::getenv("CPL_DEBUG");
        if (gdal_debug == 0)
        {
            Utils::putenv("CPL_DEBUG=ON");
        }
        
        const char* gdal_debug2 = getenv("CPL_DEBUG");
        std::cout << "Setting GDAL debug handler CPL_DEBUG=" << gdal_debug2 << std::endl;
        CPLPushErrorHandler(OCIGDALDebugErrorHandler);
        
    }
    else 
    {
        CPLPushErrorHandler(OCIGDALErrorHandler);        
    }
}

void Reader::registerFields()
{
    Schema& schema = getSchemaRef();

    Dimension xDim(Dimension::Field_X, Dimension::Int32);
    Dimension yDim(Dimension::Field_Y, Dimension::Int32);
    Dimension zDim(Dimension::Field_Z, Dimension::Int32);
    
    boost::property_tree::ptree tree = m_options.GetPTree();
    double scalex = tree.get<double>("scale.x");
    double scaley = tree.get<double>("scale.y");
    double scalez = tree.get<double>("scale.z");

    double offsetx = tree.get<double>("offset.x");
    double offsety = tree.get<double>("offset.y");
    double offsetz = tree.get<double>("offset.z");
        
    xDim.setNumericScale(scalex);
    yDim.setNumericScale(scaley);
    zDim.setNumericScale(scalez);
    xDim.setNumericOffset(offsetx);
    yDim.setNumericOffset(offsety);
    zDim.setNumericOffset(offsetz);

    schema.addDimension(xDim);
    schema.addDimension(yDim);
    schema.addDimension(zDim);

    schema.addDimension(Dimension(Dimension::Field_Time, Dimension::Double));
    schema.addDimension(Dimension(Dimension::Field_Classification, Dimension::Uint8));

    
    return;
}

const std::string& Reader::getName() const
{
    static std::string name("OCI Reader");
    return name;
}

Reader::~Reader()
{

    m_connection->DestroyType(&m_pc);
    return;
}

libpc::SequentialIterator* Reader::createSequentialIterator() const
{
    return new libpc::drivers::oci::SequentialIterator(*this);
}


}}} // namespace libpc::driver::oci
