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
    , m_querytype(QUERY_UNKNOWN)
    , m_capacity(0)
{



}


void Reader::initialize()
{
    pdal::Reader::initialize();

    m_connection = Connect(getOptions(), isDebug(), getVerboseLevel());


    if (getQuery().size() == 0 )
        throw pdal_error("'select_sql' statement is empty. No data can be read from pdal::drivers::oci::Reader");

    m_statement = Statement(m_connection->CreateStatement(getQuery().c_str()));
    
    m_statement->Execute(0);

    m_querytype = describeQueryType();


    
    if (m_querytype == QUERY_SDO_PC)
    {
        m_connection->CreateType(&m_pc);

        m_statement->Define(&(m_pc));
    
        bool bDidRead = m_statement->Fetch(); 
    
        if (!bDidRead) throw pdal_error("Unable to fetch a point cloud entry entry!");
        Schema& schema = getSchemaRef(); 
        schema = fetchSchema(m_pc);
    }
    else if (m_querytype == QUERY_SDO_PC_BLK_TYPE)
    {
        m_connection->CreateType(&m_pc_block);
        // m_connection->CreateType(&(m_pc_block->inp));
        m_statement->Define(&m_pc_block);

        bool bDidRead = m_statement->Fetch(); 
    
        if (!bDidRead) throw pdal_error("Unable to fetch a point cloud entry entry!");
        Schema& schema = getSchemaRef(); 
        schema = fetchSchema(m_pc);
        
    }
    
    else 
        throw pdal_error("SQL statement does not define a SDO_PC or CLIP_PC block");

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


    options.add(connection);
    options.add(query);
    options.add(capacity);
    
    return options;
}



std::string Reader::getQuery() const
{
    return getOptions().getValueOrThrow<std::string>("query");
}


Reader::~Reader()
{

    // m_connection->DestroyType(&m_pc);
    return;
}


QueryType Reader::describeQueryType() const
{


    int   iCol = 0;
    char  szFieldName[OWNAME];
    int   hType = 0;
    int   nSize = 0;
    int   nPrecision = 0;
    signed short nScale = 0;
    char szTypeName[OWNAME];

    while( m_statement->GetNextField(iCol, szFieldName, &hType, &nSize, &nPrecision, &nScale, szTypeName) )
    {

        if ( hType == SQLT_NTY)
        {
                // std::cout << "Field " << szFieldName << " is SQLT_NTY with type name " << szTypeName  << std::endl;
                if (boost::iequals(szTypeName, "SDO_PC"))
                    return QUERY_SDO_PC;
                if (boost::iequals(szTypeName, "SDO_PC_BLK_TYPE"))
                    return QUERY_SDO_PC_BLK_TYPE;
        }
    }

    std::ostringstream oss;
    oss << "Select statement '" << getQuery() << "' does not fetch an SDO_PC object" 
          " or SDO_PC_BLK_TYPE";
    throw pdal_error(oss.str());
}


pdal::Schema Reader::fetchSchema(sdo_pc* pc) 
{
    
    // Fetch the WKT for the SRID to set the coordinate system of this stage
    int srid = m_statement->GetInteger(&(pc->pc_geometry.sdo_srid));
    
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
    
    setSpatialReference(pdal::SpatialReference(s_wkt));
    
    
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
    
    std::ostream* out = FileUtils::createFile("schema-xml.xml");
    out->write(pc_schema, strlen(pc_schema));
    delete out;
    
    
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
    
    m_capacity = block_capacity;
    
    std::string pc_schema_xml(pc_schema);
    CPLFree(pc_schema);
    free(ptn_params);
    Schema schema = Schema::from_xml(pc_schema_xml);

    return schema;
}

CloudPtr Reader::getCloud() const
{

    CloudPtr output(new Cloud(getConnection()));
    output->base_table = m_statement->GetString(m_pc->base_table);
    output->base_column = m_statement->GetString(m_pc->base_column);
    output->pc_id = m_statement->GetInteger(&(m_pc->pc_id));
    output->blk_table = m_statement->GetString(m_pc->blk_table);
    return output;

}

pdal::StageSequentialIterator* Reader::createSequentialIterator() const
{
    return new pdal::drivers::oci::SequentialIterator(*this);
}


boost::property_tree::ptree Reader::toPTree() const
{
    boost::property_tree::ptree tree = pdal::Reader::toPTree();

    // add stuff here specific to this stage type

    return tree;
}

}}} // namespace pdal::driver::oci
