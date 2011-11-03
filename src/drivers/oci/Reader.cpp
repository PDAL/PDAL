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
#include <pdal/drivers/oci/Iterator.hpp>

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

    if (isDebug())
    {
        const char* gdal_debug = pdal::Utils::getenv("CPL_DEBUG");
        if (gdal_debug == 0)
        {
            pdal::Utils::putenv("CPL_DEBUG=ON");
        }        
        m_gdal_callback = boost::bind(&Reader::GDAL_log, this, _1, _2, _3);
    }
    else
    {
        m_gdal_callback = boost::bind(&Reader::GDAL_error, this, _1, _2, _3);
    }


#if GDAL_VERSION_MAJOR == 1 && GDAL_VERSION_MINOR >= 9
    CPLPushErrorHandlerEx(&Reader::trampoline, this);
#else
    CPLPushErrorHandler(&Reader::trampoline);
#endif
}

void Reader::GDAL_log(::CPLErr code, int num, char const* msg)
{
    std::ostringstream oss;
    
    if (code == CE_Failure || code == CE_Fatal) {
        oss <<"GDAL Failure number=" << num << ": " << msg;
        throw gdal_error(oss.str());
    } else if (code == CE_Debug) {
        oss << "GDAL debug: " << msg;
        log(oss);
        return;
    } else {
        return;
    }
}

void Reader::GDAL_error(::CPLErr code, int num, char const* msg)
{
    std::ostringstream oss;
    if (code == CE_Failure || code == CE_Fatal) {
        oss <<"GDAL Failure number=" << num << ": " << msg;
        throw gdal_error(oss.str());
    } else {
        return;
    }
}

void Reader::initialize()
{
    pdal::Reader::initialize();

    m_connection = connect();
    m_block = BlockPtr(new Block(m_connection));

    if (getQuery().size() == 0 )
        throw pdal_error("'query' statement is empty. No data can be read from pdal::drivers::oci::Reader");

    m_statement = Statement(m_connection->CreateStatement(getQuery().c_str()));
    
    m_statement->Execute(0);

    m_querytype = describeQueryType();

    std::ostringstream oss;
    oss << "Query type is: " << m_querytype;
    log(oss);
    
    if (m_querytype == QUERY_SDO_PC)
    {
        defineBlock(m_statement, m_block);
    
        bool bDidRead = m_statement->Fetch(); 
    
        if (!bDidRead) throw pdal_error("Unable to fetch a point cloud entry entry!");
        
        setSpatialReference(fetchSpatialReference(m_statement, m_block->pc));

        Schema& schema = getSchemaRef(); 
        schema = fetchSchema(m_statement, m_block->pc, m_capacity);
    }
    // else if (m_querytype == QUERY_SDO_PC_BLK_TYPE)
    // {
    //     m_connection->CreateType(&m_pc_block);
    //     // m_connection->CreateType(&(m_pc_block->inp));
    //     m_statement->Define(&m_pc_block);
    // 
    //     bool bDidRead = m_statement->Fetch(); 
    // 
    //     if (!bDidRead) throw pdal_error("Unable to fetch a point cloud entry entry!");
    //     setSpatialReference(fetchSpatialReference(m_statement, m_block->pc));
    //     
    //     Schema& schema = getSchemaRef(); 
    //     schema = fetchSchema(m_statement, m_block->pc, m_capacity);
    //     
    // }
    else if (m_querytype == QUERY_SDO_BLK_PC_VIEW)
    {
        
        defineBlock(m_statement, m_block);
        bool bDidRead = m_statement->Fetch(); 
    
        if (!bDidRead) throw pdal_error("Unable to fetch a point cloud entry entry!");
        Schema& schema = getSchemaRef(); 
        setSpatialReference(fetchSpatialReference(m_statement, m_block->pc));

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
    CPLPopErrorHandler();

    pdal::Utils::putenv("CPL_DEBUG=OFF");    
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
    log(oss);
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
            std::ostringstream oss;
            oss << "Unable to find block field " << q->first << "in describeQueryType()";
            log(oss);
            break;
        }
        bHasBlockFields = true;
    }
    
    if (bHaveSDO_PC && bHasBlockFields) return QUERY_SDO_BLK_PC_VIEW;
    if (bHaveSDO_PC) return QUERY_SDO_PC;
    
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
    std::ostringstream oss;
    oss << "Fetching schema from SDO_PC object";
    log(oss);
    
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

CloudPtr Reader::getCloud() const
{

    CloudPtr output(new Cloud(getConnection()));
    output->base_table = m_statement->GetString(m_block->pc->base_table);
    output->base_column = m_statement->GetString(m_block->pc->base_column);
    output->pc_id = m_statement->GetInteger(&(m_block->pc->pc_id));
    output->blk_table = m_statement->GetString(m_block->pc->blk_table);
    return output;

}

pdal::StageSequentialIterator* Reader::createSequentialIterator() const
{
    return new pdal::drivers::oci::SequentialIterator(*this);
}


boost::property_tree::ptree Reader::toPTree() const
{
    boost::property_tree::ptree tree = pdal::Reader::toPTree();

    return tree;
}

}}} // namespace pdal::driver::oci
