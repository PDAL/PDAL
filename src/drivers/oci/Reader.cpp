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

#include <pdal/drivers/oci/Reader.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/FileUtils.hpp>

#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>
#include <boost/tokenizer.hpp>
#include <boost/make_shared.hpp>
#include <boost/algorithm/string.hpp>

#include <iostream>
#include <map>


namespace pdal { namespace drivers { namespace oci {


Reader::Reader(const Options& options)
    : pdal::Reader(options)
    , OracleDriver(getOptions())
    , m_querytype(QUERY_UNKNOWN)
    , m_capacity(0)
{

}

void Reader::initialize()
{
    pdal::Reader::initialize();

    m_gdal_debug = boost::shared_ptr<pdal::gdal::Debug>( new pdal::gdal::Debug(isDebug(), log()));
    m_connection = connect();
    m_block = BlockPtr(new Block(m_connection));

    if (getQuery().size() == 0 )
        throw pdal_error("'query' statement is empty. No data can be read from pdal::drivers::oci::Reader");

    m_statement = Statement(m_connection->CreateStatement(getQuery().c_str()));
    
    m_statement->Execute(0);

    m_querytype = describeQueryType();

    if (m_querytype == QUERY_SDO_PC)
    {
        defineBlock(m_statement, m_block);
    
        bool bDidRead = m_statement->Fetch(); 
    
        if (!bDidRead) throw pdal_error("Unable to fetch a point cloud entry entry!");
        
        try 
        {
            setSpatialReference(getOptions().getValueOrThrow<pdal::SpatialReference>("spatialreference"));
        
        }
        catch (pdal_error const&) 
        {
            // If one wasn't set on the options, we'll ignore at this 
            setSpatialReference(fetchSpatialReference(m_statement, m_block->pc));

        }

        Schema& schema = getSchemaRef(); 
        schema = fetchSchema(m_statement, m_block->pc, m_capacity);
    }

    else if (m_querytype == QUERY_SDO_BLK_PC_VIEW)
    {
        
        defineBlock(m_statement, m_block);
        bool bDidRead = m_statement->Fetch(); 
    
        if (!bDidRead) throw pdal_error("Unable to fetch a point cloud entry entry!");
        Schema& schema = getSchemaRef(); 

        try 
        {
            setSpatialReference(getOptions().getValueOrThrow<pdal::SpatialReference>("spatialreference"));
        
        }
        catch (pdal_error const&) 
        {
            // If one wasn't set on the options, we'll ignore at this 
            setSpatialReference(fetchSpatialReference(m_statement, m_block->pc));

        }

        schema = fetchSchema(m_statement, m_block->pc, m_capacity);
        
    }
    
    else 
        throw pdal_error("SQL statement does not define a SDO_PC or CLIP_PC block");

}



void Reader::defineBlock(Statement statement, BlockPtr block) const
{

    int   iCol = 0;
    char  szFieldName[OWNAME];
    int   hType = 0;
    int   nSize = 0;
    int   nPrecision = 0;
    signed short nScale = 0;
    char szTypeName[OWNAME];
    

    while( statement->GetNextField(iCol, szFieldName, &hType, &nSize, &nPrecision, &nScale, szTypeName) )
    {
        std::string name = boost::to_upper_copy(std::string(szFieldName));

        if ( hType == SQLT_NTY)
        {
            if (boost::iequals(szTypeName, "SDO_PC"))
            {
                statement->Define(&(block->pc));
            }
        }
        if (boost::iequals(szFieldName, "OBJ_ID"))
        {
            statement->Define(&(block->obj_id));
        }

        if (boost::iequals(szFieldName, "BLK_ID"))
        {
            statement->Define(&(block->blk_id));
        }

        if (boost::iequals(szFieldName, "BLK_EXTENT"))
        {
            statement->Define(&(block->blk_extent));
        }

        if (boost::iequals(szFieldName, "BLK_DOMAIN"))
        {
            statement->Define(&(block->blk_domain));
        }
        
        if (boost::iequals(szFieldName, "PCBLK_MIN_RES"))
        {
            statement->Define(&(block->pcblk_min_res));
        }

        if (boost::iequals(szFieldName, "PCBLK_MAX_RES"))
        {
            statement->Define(&(block->pcblk_max_res));
        }

        if (boost::iequals(szFieldName, "NUM_POINTS"))
        {
            statement->Define(&(block->num_points));
        }

        if (boost::iequals(szFieldName, "NUM_UNSORTED_POINTS"))
        {
            statement->Define(&(block->num_unsorted_points));
        }

        if (boost::iequals(szFieldName, "PT_SORT_DIM"))
        {
            statement->Define(&(block->pt_sort_dim));
        }

        if (boost::iequals(szFieldName, "POINTS"))
        {
            statement->Define( &(block->locator) ); 
        }
        iCol++;
    }
    
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



std::string Reader::getQuery() const
{
    return getOptions().getValueOrThrow<std::string>("query");
}

Reader::~Reader()
{
    return;
}

QueryType Reader::describeQueryType()
{

    int   iCol = 0;
    char  szFieldName[OWNAME];
    int   hType = 0;
    int   nSize = 0;
    int   nPrecision = 0;
    signed short nScale = 0;
    char szTypeName[OWNAME];
    

    std::ostringstream oss;

    std::map<std::string, std::string> objects;
    while( m_statement->GetNextField(iCol, szFieldName, &hType, &nSize, &nPrecision, &nScale, szTypeName) )
    {
        
        std::pair<std::string, int> p(szFieldName, hType);
        m_fields.insert(p);

        if ( hType == SQLT_NTY)
        {
            std::pair<std::string, std::string> p(szTypeName, szFieldName);
            oss << "Typed Field " << std::string(szFieldName) << " with type " << hType << " and typename " << std::string(szTypeName) << std::endl;

            objects.insert(p);
        } else
        {
            oss << "Field " << std::string(szFieldName) << " with type " << hType << std::endl;
            
        }
        iCol++;
    }
    log()->get(logDEBUG)  << oss.str() << std::endl;
        // if ( hType == SQLT_NTY)
        // {
        //     std::cout << "Field " << szFieldName << " is SQLT_NTY with type name " << szTypeName  << std::endl;
        //     if (boost::iequals(szTypeName, "SDO_PC"))
        //         return QUERY_SDO_PC;
        //     if (boost::iequals(szTypeName, "SDO_PC_BLK_TYPE"))
        //         return QUERY_SDO_PC_BLK_TYPE;
        // }
    bool bHaveSDO_PC(false);
    if (objects.find("SDO_PC") != objects.end()) bHaveSDO_PC = true;
    bool bHaveSDO_PC_BLK_TYPE(false);
    if (objects.find("SDO_PC_BLK_TYPE") != objects.end()) bHaveSDO_PC_BLK_TYPE = true;
    if ( !bHaveSDO_PC && !bHaveSDO_PC_BLK_TYPE)
    {
        std::ostringstream oss;
        oss << "Select statement '" << getQuery() << "' does not fetch a SDO_PC object" 
              " or SDO_PC_BLK_TYPE";
        throw pdal_error(oss.str());
        
    }
    
    typedef std::pair<std::string, bool> SB;
    std::map<std::string, bool> block_fields;
    
    // We must have all of these field names present to be considered block 
    // data.
    block_fields.insert(SB("OBJ_ID", false));
    block_fields.insert(SB("BLK_ID", false));
    block_fields.insert(SB("BLK_EXTENT", false)); block_fields.insert(SB("BLK_DOMAIN", false));
    block_fields.insert(SB("PCBLK_MIN_RES", false)); block_fields.insert(SB("PCBLK_MAX_RES", false));
    block_fields.insert(SB("NUM_POINTS", false)); block_fields.insert(SB("NUM_UNSORTED_POINTS", false));
    block_fields.insert(SB("PT_SORT_DIM", false)); block_fields.insert(SB("POINTS", false));
    
    std::map<std::string, int>::const_iterator i = m_fields.begin();
    
    bool bHasBlockFields(false);
    for (i = m_fields.begin(); i != m_fields.end(); ++i)
    {
        std::map<std::string, bool>::iterator t = block_fields.find(i->first);
        if (t != block_fields.end())
            t->second = true;
    }
    
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
    
    if (bHaveSDO_PC && bHasBlockFields) 
    {
        log()->get(logDEBUG) << "Query type is QUERY_SDO_BLK_PC_VIEW" << std::endl;
        return QUERY_SDO_BLK_PC_VIEW;
    }
    if (bHaveSDO_PC) 
    {
        log()->get(logDEBUG) << "Query type is QUERY_SDO_PC" << std::endl;
        return QUERY_SDO_PC;
    }

    log()->get(logDEBUG) << "Query type is QUERY_UNKNOWN" << std::endl;
    return QUERY_UNKNOWN;
    
}

pdal::SpatialReference Reader::fetchSpatialReference(Statement statement, sdo_pc* pc) const
{
    // Fetch the WKT for the SRID to set the coordinate system of this stage
    int srid = statement->GetInteger(&(pc->pc_geometry.sdo_srid));
    
    std::ostringstream select_wkt;
    select_wkt
        << "SELECT WKTEXT3D from MDSYS.CS_SRS WHERE SRID = " << srid;

    int wkt_length = 3999;
    char* wkt = (char*) malloc (sizeof(char*) * wkt_length);
    Statement get_wkt(m_connection->CreateStatement(select_wkt.str().c_str()));
    get_wkt->Define( wkt, wkt_length );    
    get_wkt->Execute();    
    std::string s_wkt(wkt);
    free(wkt);
    
    return pdal::SpatialReference(s_wkt);
}

pdal::Schema Reader::fetchSchema(Statement statement, sdo_pc* pc, boost::uint32_t& capacity) const
{
    log()->get(logDEBUG) << "Fetching schema from SDO_PC object" << std::endl;
    
    // Fetch the XML that defines the schema for this point cloud
    std::ostringstream select_schema;
    OCILobLocator* metadata = NULL;    
    select_schema
        << "DECLARE" << std::endl
        << "PC_TABLE VARCHAR2(32) := '" << m_statement->GetString(pc->base_table) << "';" << std::endl
        << "PC_ID NUMBER := " << m_statement->GetInteger(&(pc->pc_id)) << ";" << std::endl
        << "PC_COLUMN VARCHAR2(32) := '" << m_statement->GetString(pc->base_column) << "';" << std::endl
        << "BEGIN" << std::endl
        << std::endl 
        << "EXECUTE IMMEDIATE" << std::endl
        << " 'SELECT T.'||PC_COLUMN||'.PC_OTHER_ATTRS.getClobVal(), T.'||PC_COLUMN||'.PTN_PARAMS FROM '||pc_table||' T WHERE T.ID='||PC_ID INTO :metadata, :capacity;"
        << std::endl
        << "END;"
        << std::endl;
    Statement get_schema(m_connection->CreateStatement(select_schema.str().c_str()));
    get_schema->BindName( ":metadata", &metadata );
    
    int ptn_params_length = 1024;
    char* ptn_params = (char*) malloc (sizeof(char*) * ptn_params_length);
    ptn_params[ptn_params_length-1] = '\0'; //added trailing null to fix ORA-01480
    get_schema->BindName( ":capacity", ptn_params, ptn_params_length );
    get_schema->Execute();
    
    char* pc_schema = get_schema->ReadCLob(metadata);
    
    std::string write_schema_file = getOptions().getValueOrDefault<std::string>("xml_schema_dump", std::string(""));
    if (write_schema_file.size() > 0)
    {
        std::ostream* out = FileUtils::createFile("schema-xml.xml");
        out->write(pc_schema, strlen(pc_schema));
        delete out;
    }
    

    // Fetch the block capacity from the point cloud object so we 
    // can use it later.
    // PTN_PARAMS is like:
    // 'blk_capacity=1000,work_tablespace=my_work_ts'
    int block_capacity = 0;
    boost::char_separator<char> sep_space(" ");
    boost::char_separator<char> sep_equal("=");

    std::string s_cap(ptn_params);
    tokenizer parameters(s_cap, sep_space);
    for (tokenizer::iterator t = parameters.begin(); t != parameters.end(); ++t) {
        tokenizer parameter((*t), sep_equal);

        for(tokenizer::iterator c = parameter.begin(); c != parameter.end(); ++c)
        {
            if (boost::iequals(c->c_str(), "blk_capacity"))
            {
                tokenizer::iterator d = ++c;
                block_capacity = atoi(d->c_str());
            }
        }
    }    

    if (block_capacity < 1) 
    {
        std::ostringstream oss;
        oss << "Invalid block capacity for point cloud object in Oracle: " << block_capacity;
        throw pdal_error(oss.str());
    }
    
    capacity = block_capacity;
    
    std::string pc_schema_xml(pc_schema);
    if (pc_schema)
        CPLFree(pc_schema);
    
    if (ptn_params)
        free(ptn_params);

    Schema schema = Schema::from_xml(pc_schema_xml);

    return schema;
}

void Reader::addDefaultDimensions()
{

}

pdal::StageSequentialIterator* Reader::createSequentialIterator() const
{
    return new pdal::drivers::oci::iterators::sequential::Reader(*this);
}


boost::property_tree::ptree Reader::toPTree() const
{
    boost::property_tree::ptree tree = pdal::Reader::toPTree();

    return tree;
}


namespace iterators { namespace sequential {


IteratorBase::IteratorBase(const pdal::drivers::oci::Reader& reader)
    : bBlockStatementComplete(false)
    , bCloudStatementComplete(false)
    , m_block(BlockPtr(new Block(reader.getConnection())))
    , m_active_cloud_id(0)
    , m_active_buffer(BufferPtr())
    , bBufferHasNewSchema(false)
    , bReadFirstCloud(true)
    , m_current_blob_position(0)
    , m_current_point_position(0)
    , m_reader(reader)

{
    
    m_querytype = reader.getQueryType();
    
    if (m_querytype == QUERY_SDO_PC)
    {
        m_block_statement = getNextCloud(m_block, m_active_cloud_id);
    }

    if (m_querytype == QUERY_SDO_BLK_PC_VIEW)
    {
        m_block_statement = reader.getStatement();
        m_block = reader.getBlock();
        
        m_active_cloud_id = m_block_statement->GetInteger(&m_block->pc->pc_id);
    }
    
    return;
}

Statement IteratorBase::getNextCloud(BlockPtr block, boost::int32_t& cloud_id)
{

    std::ostringstream select_blocks;
    BlockPtr cloud_block = m_reader.getBlock();
    Statement cloud_statement = m_reader.getStatement();
    
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



const pdal::drivers::oci::Reader& IteratorBase::getReader() const
{
    return m_reader;
}

void IteratorBase::fillBufferWithSDO_PC_BlockData(PointBuffer& data,
                        Statement statement,
                        BlockPtr block,
                        boost::uint32_t howMany, 
                        boost::uint32_t whichPointPosition, 
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
    
    boost::uint32_t howMuchToRead = howMany * data.getSchema().getByteSize();
    data.setDataStride(&(*block->chunk)[whichBlobPosition], whichPointPosition, howMuchToRead);

    data.setNumPoints(data.getNumPoints() + howMany);

}

boost::uint32_t IteratorBase::myReadBuffer(PointBuffer& data)
{
    // if (m_querytype == QUERY_SDO_PC)
    //     return myReadClouds(data);
    if (m_querytype == QUERY_SDO_BLK_PC_VIEW)
        return myReadBlocks(data);
    
    return 0;
}

// boost::uint32_t IteratorBase::myReadClouds(PointBuffer& data)
// {
//     boost::uint32_t numReadThisCloud(0);
//     boost::uint32_t numToRead = data.getCapacity();
//     
//     getReader().log()->get(logDEBUG2) << "Fetched buffer with cloud id: " << m_active_cloud_id << " for myReadClouds" << std::endl;
//     getReader().log()->get(logDEBUG2) << "Initial read request size: " << numToRead << std::endl;
//     
//     bool bReadCloud(true);
//     while( bReadCloud) 
//     {
//         m_active_buffer = createPointBufferFromSDO_PC(m_statement, getReader().getBlock()->pc, data.getCapacity());
//         
//         bBufferHasNewSchema = true;
//             
//         boost::uint32_t numRead = myReadBlocks(data);
//         
//         numReadThisCloud = numRead + numReadThisCloud;
// 
//         
//         if (!m_have_more_blocks)
//         {
// 
//             getReader().log()->get(logDEBUG2) << "Read " << numReadThisCloud << " points from myReadBlocks. Total read this cloud " << numRead << std::endl;
//         
//             bReadCloud = getReader().getStatement()->Fetch();
//             m_block = BlockPtr(new Block(getReader().getConnection()));
//             m_statement = getNextCloud(m_block, m_active_cloud_id);
//             if (m_at_end == true) 
//             {
//                 getReader().log()->get(logDEBUG2) << "At end of current block and trying to fetch another cloud " << std::endl;
//             
//                 if (bReadCloud) 
//                 {
//                     getReader().log()->get(logDEBUG2) << "Fetched another cloud " << std::endl;
//                     m_block = BlockPtr(new Block(getReader().getConnection()));
//                     m_statement = getNextCloud(m_block, m_active_cloud_id);
//                     m_at_end = false;
// 
//                 } else
//                 {
//                     getReader().log()->get(logDEBUG2) << "Another cloud not found " << std::endl;
//                     
//                 }
//                 return numReadThisCloud;
//             }
//             else
//             {
// 
//                 getReader().log()->get(logDEBUG2) << "At end of current block and have more blocks to fetch" << std::endl;                
//                 return numReadThisCloud;
//             }
// 
//         } 
//     }
// 
//     return numReadThisCloud;
// }

BufferPtr IteratorBase::createPointBufferFromSDO_PC(Statement statement, sdo_pc* pc, boost::uint32_t capacity)
{
    boost::int32_t id = m_block_statement->GetInteger(&pc->pc_id);
    BufferMap::const_iterator i = m_buffers.find(id);
    
    if (i != m_buffers.end())
    {
        getReader().log()->get(logDEBUG2) << "IteratorBase::fetchPointBuffer: found existing PointBuffer with id " << id << std::endl;
        return i->second;
    } else {
        boost::uint32_t block_capacity(0);
        Schema schema = m_reader.fetchSchema(statement, pc, block_capacity);
        
        if (block_capacity > capacity)
        {
            std::ostringstream oss;
            oss << "Block size is larger, " << block_capacity << ", than buffer capacity, " << capacity << ". Try increasing your 'chunk_size' writer option or increasing the size of the buffer being requested";
            throw pdal_error(oss.str());
        }
        
        BufferPtr output  = BufferPtr(new PointBuffer(schema, capacity));
        std::pair<int, BufferPtr> p(id, output);
        m_buffers.insert(p);
        getReader().log()->get(logDEBUG2) << "IteratorBase::fetchPointBuffer: creating new PointBuffer with id " << id << std::endl;

        return p.second;
    }    
}



boost::uint32_t IteratorBase::myReadBlocks(PointBuffer& data)
{
    boost::uint32_t numPointsRead = 0;
    if (bBufferHasNewSchema)
    {
        getReader().log()->get(logDEBUG2) << "IteratorBase::myReadBlocks: Switching buffer with id " << m_active_cloud_id << std::endl;
        data = *m_active_buffer;
        bBufferHasNewSchema = false;
    }
    data.setNumPoints(0);


    if (!m_block->num_points) 
    {

        getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: fetching first block" << std::endl;
        bBlockStatementComplete = !m_block_statement->Fetch();        
        if (bBlockStatementComplete)
        {
            return 0;
        }
        
    } 
    else 
    {
        // Our read was already "done" last readBuffer call, but if we're done,
        // we're done
        getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: we have points left to read on this block" << std::endl;


        if (bBlockStatementComplete)
            getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: we are at end of the blocks;" << std::endl;
        else
        {

            getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: Reading leftover data at blob position " << m_current_blob_position << " and point position " << m_current_point_position <<std::endl;
            
            bool bDoneWithOldBlock(false);
            boost::uint32_t howMany = (std::min)(m_current_point_position, data.getCapacity());
            if (howMany < data.getCapacity() )
                bDoneWithOldBlock = true;
            fillBufferWithSDO_PC_BlockData( data, m_block_statement, 
                                        m_block, howMany, 
                                        data.getNumPoints(), m_current_blob_position);
            if (bDoneWithOldBlock)
            {
                m_current_blob_position = 0;
                m_current_point_position = 0;
            } else
            {
                m_current_blob_position = howMany * data.getSchema().getByteSize();
                m_current_point_position = howMany;
                return howMany;
            }

        }
        
        if (bBlockStatementComplete) return 0;
    
    }

    pdal::Bounds<double> old_bounds = data.getSpatialBounds();
    pdal::Bounds<double> new_bounds = getBounds(m_block_statement, m_block);
    new_bounds.grow(old_bounds);
    data.setSpatialBounds(new_bounds);
    
    while (!bBlockStatementComplete)
    {
        boost::uint32_t numReadThisBlock = m_block->num_points;
        boost::uint32_t numSpaceLeftThisBlock = data.getCapacity() - data.getNumPoints();
        
        getReader().log()->get(logDEBUG4) << "IteratorBase::myReadBlocks:" " numReadThisBlock: " 
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
            // 
            m_current_point_position = numSpaceLeftThisBlock;
            fillBufferWithSDO_PC_BlockData( data, m_block_statement, 
                                        m_block, m_current_point_position, 
                                        data.getNumPoints(), 0);
            m_current_blob_position = numSpaceLeftThisBlock * data.getSchema().getByteSize();
                                        
            getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: numReadThisBlock > numSpaceLeftThisBlock. Filling to end of buffer and storing block position " << m_current_blob_position << std::endl;
            return data.getNumPoints();
            
        }

        numPointsRead = numPointsRead + numReadThisBlock;
        
        fillBufferWithSDO_PC_BlockData( data, m_block_statement, 
                                        m_block, (std::min)(numReadThisBlock, data.getCapacity()), 
                                        data.getNumPoints(), 0);
        
        bBlockStatementComplete = !m_block_statement->Fetch();
        if (bBlockStatementComplete)
        {
            getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: done reading block. Read " << numPointsRead << " points" << std::endl;
            m_current_blob_position = 0;
            return numPointsRead;
        }

       
        if (m_querytype == QUERY_SDO_BLK_PC_VIEW)
        {
            boost::int32_t current_cloud_id(0);
            current_cloud_id  = m_block_statement->GetInteger(&m_block->pc->pc_id);

            getReader().log()->get(logDEBUG3) << "IteratorBase::myReadBlocks: current_cloud_id: " 
                                              << current_cloud_id << " m_active_cloud_id: " 
                                              << m_active_cloud_id << std::endl;
            
            if (current_cloud_id != m_active_cloud_id)
            {
                m_active_buffer = createPointBufferFromSDO_PC(m_block_statement, m_block->pc, data.getCapacity());

                bBufferHasNewSchema = true;
                m_active_cloud_id = current_cloud_id;
                m_current_blob_position = 0;
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

Reader::Reader(const pdal::drivers::oci::Reader& reader)
    : IteratorBase(reader)
    , pdal::StageSequentialIterator(reader)
{
    return;
}


Reader::~Reader()
{
    return;
}


boost::uint64_t Reader::skipImpl(boost::uint64_t count)
{

    return 0;
}


bool Reader::atEndImpl() const
{
    return bCloudStatementComplete;
}


boost::uint32_t Reader::readBufferImpl(PointBuffer& data)
{
    return myReadBuffer(data);
}    



}} // iterators::sequential::

}}} // namespace pdal::driver::oci
