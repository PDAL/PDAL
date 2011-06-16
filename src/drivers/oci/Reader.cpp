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
#include <pdal/Utils.hpp>

#include <pdal/exceptions.hpp>

#include <iostream>
#include <map>


namespace pdal { namespace drivers { namespace oci {


Reader::Reader(Options& options)
    : pdal::Stage()
    , m_options(options)
    , m_querytype(QUERY_UNKNOWN)
{

    Debug();
    
    m_connection = Connect(m_options);


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



    

    
    // setNumPoints(1000);
}    

bool Reader::isVerbose() const
{
    return m_options.GetPTree().get<bool>("verbose");
}

bool Reader::isDebug() const
{
    return m_options.GetPTree().get<bool>("debug");
}

std::string Reader::getQuery() const
{
    return m_options.GetPTree().get<std::string>("select_sql");
}

void Reader::Debug()
{
    bool debug = isDebug();

    CPLPopErrorHandler();

    if (debug)
    {
        const char* gdal_debug = Utils::getenv("CPL_DEBUG");
        if (gdal_debug == 0)
        {
            Utils::putenv("CPL_DEBUG=ON");
        }
        
        // const char* gdal_debug2 = getenv("CPL_DEBUG");
        // std::cout << "Setting GDAL debug handler CPL_DEBUG=" << gdal_debug2 << std::endl;
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

    Dimension x(Dimension::Field_X, Dimension::Int32);
    Dimension y(Dimension::Field_Y, Dimension::Int32);
    Dimension z(Dimension::Field_Z, Dimension::Int32);
    
    boost::property_tree::ptree tree = m_options.GetPTree();
    double scalex = tree.get<double>("scale.x");
    double scaley = tree.get<double>("scale.y");
    double scalez = tree.get<double>("scale.z");

    double offsetx = tree.get<double>("offset.x");
    double offsety = tree.get<double>("offset.y");
    double offsetz = tree.get<double>("offset.z");
        
    x.setNumericScale(scalex);
    y.setNumericScale(scaley);
    z.setNumericScale(scalez);
    x.setNumericOffset(offsetx);
    y.setNumericOffset(offsety);
    z.setNumericOffset(offsetz);


    std::ostringstream text;

    text << "x coordinate as a long integer.  You must use the scale and "
         << "offset information of the header to determine the double value.";
    x.setDescription(text.str());
    schema.addDimension(x);
    text.str("");

    text << "y coordinate as a long integer.  You must use the scale and "
         << "offset information of the header to determine the double value.";
    y.setDescription(text.str());
    schema.addDimension(y);
    text.str("");

    text << "z coordinate as a long integer.  You must use the scale and "
         << "offset information of the header to determine the double value.";
    z.setDescription(text.str());
    schema.addDimension(z);
    text.str("");

    Dimension intensity(Dimension::Field_Intensity, Dimension::Uint16);
    text << "The intensity value is the integer representation of the pulse "
         "return magnitude. This value is optional and system specific. "
         "However, it should always be included if available.";
    intensity.setDescription(text.str());
    schema.addDimension(intensity);
    text.str("");

    Dimension return_no(Dimension::Field_ReturnNumber, Dimension::Uint8); // 3 bits only
    text << "Return Number: The Return Number is the pulse return number for "
         "a given output pulse. A given output laser pulse can have many "
         "returns, and they must be marked in sequence of return. The first "
         "return will have a Return Number of one, the second a Return "
         "Number of two, and so on up to five returns.";
    return_no.setDescription(text.str());
    schema.addDimension(return_no);
    text.str("");

    Dimension no_returns(Dimension::Field_NumberOfReturns, Dimension::Uint8); // 3 bits only
    text << "Number of Returns (for this emitted pulse): The Number of Returns "
         "is the total number of returns for a given pulse. For example, "
         "a laser data point may be return two (Return Number) within a "
         "total number of five returns.";
    no_returns.setDescription(text.str());
    schema.addDimension(no_returns);
    text.str("");

    Dimension scan_dir(Dimension::Field_ScanDirectionFlag, Dimension::Uint8); // 1 bit only
    text << "The Scan Direction Flag denotes the direction at which the "
         "scanner mirror was traveling at the time of the output pulse. "
         "A bit value of 1 is a positive scan direction, and a bit value "
         "of 0 is a negative scan direction (where positive scan direction "
         "is a scan moving from the left side of the in-track direction to "
         "the right side and negative the opposite). ";
    scan_dir.setDescription(text.str());
    schema.addDimension(scan_dir);
    text.str("");

    Dimension edge(Dimension::Field_EdgeOfFlightLine, Dimension::Uint8); // 1 bit only
    text << "The Edge of Flight Line data bit has a value of 1 only when "
         "the point is at the end of a scan. It is the last point on "
         "a given scan line before it changes direction.";
    edge.setDescription(text.str());
    schema.addDimension(edge);
    text.str("");

    Dimension classification(Dimension::Field_Classification, Dimension::Uint8);
    text << "Classification in LAS 1.0 was essentially user defined and optional. "
         "LAS 1.1 defines a standard set of ASPRS classifications. In addition, "
         "the field is now mandatory. If a point has never been classified, this "
         "byte must be set to zero. There are no user defined classes since "
         "both point format 0 and point format 1 supply 8 bits per point for "
         "user defined operations. Note that the format for classification is a "
         "bit encoded field with the lower five bits used for class and the "
         "three high bits used for flags.";
    classification.setDescription(text.str());
    schema.addDimension(classification);
    text.str("");

    Dimension scan_angle(Dimension::Field_ScanAngleRank, Dimension::Int8);
    text << "The Scan Angle Rank is a signed one-byte number with a "
         "valid range from -90 to +90. The Scan Angle Rank is the "
         "angle (rounded to the nearest integer in the absolute "
         "value sense) at which the laser point was output from the "
         "laser system including the roll of the aircraft. The scan "
         "angle is within 1 degree of accuracy from +90 to ñ90 degrees. "
         "The scan angle is an angle based on 0 degrees being nadir, "
         "and ñ90 degrees to the left side of the aircraft in the "
         "direction of flight.";
    scan_angle.setDescription(text.str());
    schema.addDimension(scan_angle);
    text.str("");

    Dimension user_data(Dimension::Field_UserData, Dimension::Uint8);
    text << "This field may be used at the userís discretion";
    user_data.setDescription(text.str());
    schema.addDimension(user_data);
    text.str("");

    Dimension point_source_id(Dimension::Field_PointSourceId, Dimension::Uint16);
    text << "This value indicates the file from which this point originated. "
         "Valid values for this field are 1 to 65,535 inclusive with zero "
         "being used for a special case discussed below. The numerical value "
         "corresponds to the File Source ID from which this point originated. "
         "Zero is reserved as a convenience to system implementers. A Point "
         "Source ID of zero implies that this point originated in this file. "
         "This implies that processing software should set the Point Source "
         "ID equal to the File Source ID of the file containing this point "
         "at some time during processing. ";
    point_source_id.setDescription(text.str());
    schema.addDimension(point_source_id);
    text.str("");

    Dimension t(Dimension::Field_Time, Dimension::Double);
    text << "The GPS Time is the double floating point time tag value at "
        "which the point was acquired. It is GPS Week Time if the "
        "Global Encoding low bit is clear and Adjusted Standard GPS "
        "Time if the Global Encoding low bit is set (see Global Encoding "
        "in the Public Header Block description).";
    t.setDescription(text.str());
    schema.addDimension(t);
    text.str("");

    Dimension red(Dimension::Field_Red, Dimension::Uint16);
    text << "The red image channel value associated with this point";
    red.setDescription(text.str());
    schema.addDimension(red);
    text.str("");

    Dimension green(Dimension::Field_Green, Dimension::Uint16);
    text << "The green image channel value associated with this point";
    green.setDescription(text.str());
    schema.addDimension(green);
    text.str("");

    Dimension blue(Dimension::Field_Blue, Dimension::Uint16);
    text << "The blue image channel value associated with this point";
    blue.setDescription(text.str());
    schema.addDimension(blue);
    text.str("");


    
    return;
}

const std::string& Reader::getDescription() const
{
    static std::string name("OCI Reader");
    return name;
}

const std::string& Reader::getName() const
{
    static std::string name("drivers.oci.reader");
    return name;
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
                if (compare_no_case(szTypeName, "SDO_PC") == 0)
                    return QUERY_SDO_PC;
                if (compare_no_case(szTypeName, "SDO_PC_BLK_TYPE") == 0)
                    return QUERY_SDO_PC_BLK_TYPE;
        }
    }

    std::ostringstream oss;
    oss << "Select statement '" << getQuery() << "' does not fetch an SDO_PC object" 
          " or SDO_PC_BLK_TYPE";
    throw pdal_error(oss.str());
}

BlockPtr Reader::defineBlock() const
{

    int   iCol = 0;
    char  szFieldName[OWNAME];
    int   hType = 0;
    int   nSize = 0;
    int   nPrecision = 0;
    signed short nScale = 0;
    char szTypeName[OWNAME];
    
    BlockPtr block = BlockPtr(new Block(m_connection));




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

        if (compare_no_case(szFieldName, "OBJ_ID") == 0)
        {
            m_statement->Define(&(block->obj_id));
        }

        if (compare_no_case(szFieldName, "BLK_ID") == 0)
        {
            m_statement->Define(&(block->blk_id));
        }

        if (compare_no_case(szFieldName, "BLK_EXTENT") == 0)
        {
            m_statement->Define(&(block->blk_extent));
        }

        if (compare_no_case(szFieldName, "BLK_DOMAIN") == 0)
        {
            m_statement->Define(&(block->blk_domain));
        }
        
        if (compare_no_case(szFieldName, "PCBLK_MIN_RES") == 0)
        {
            m_statement->Define(&(block->pcblk_min_res));
        }

        if (compare_no_case(szFieldName, "PCBLK_MAX_RES") == 0)
        {
            m_statement->Define(&(block->pcblk_max_res));
        }

        if (compare_no_case(szFieldName, "NUM_POINTS") == 0)
        {
            m_statement->Define(&(block->num_points));
        }

        if (compare_no_case(szFieldName, "NUM_UNSORTED_POINTS") == 0)
        {
            m_statement->Define(&(block->num_unsorted_points));
        }

        if (compare_no_case(szFieldName, "PT_SORT_DIM") == 0)
        {
            m_statement->Define(&(block->pt_sort_dim));
        }

        if (compare_no_case(szFieldName, "POINTS") == 0)
        {
            // std::cout << "Defined POINTS as BLOB" << std::endl;
            m_statement->Define( &(block->locator) ); 
        }
        iCol++;
    }
    
    return block;
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
    get_schema->BindName( ":capacity", ptn_params, ptn_params_length );
    get_schema->Execute();
    
    char* pc_schema = get_schema->ReadCLob(metadata);
    
    std::ostream* out = Utils::createFile("schema-xml.xml");
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
            if (compare_no_case(c->c_str(), "blk_capacity") == 0)
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
    
    m_options.GetPTree().put("capacity", block_capacity);
    
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

pdal::SequentialIterator* Reader::createSequentialIterator() const
{
    return new pdal::drivers::oci::SequentialIterator(*this);
}


}}} // namespace pdal::driver::oci
