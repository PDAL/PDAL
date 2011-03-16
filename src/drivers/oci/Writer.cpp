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


#include <cassert>
#include <sstream>


#include <libpc/drivers/oci/Writer.hpp>
#include <libpc/drivers/oci/Header.hpp>

#include <libpc/exceptions.hpp>
#include <libpc/libpc_config.hpp>

#include <cstdlib>
#include <iostream>

#include <boost/make_shared.hpp>

namespace libpc { namespace driver { namespace oci {


Writer::Writer(Stage& prevStage, Options& options)
    : libpc::Writer(prevStage)
    , m_stage(prevStage)
    , m_chipper(m_stage, options.GetPTree().get<boost::uint32_t>("capacity") )
    , m_options(options)
    , m_verbose(false)
{

    
    return;
}

Connection Writer::Connect()
{
    std::string connection = m_options.GetPTree().get<std::string>("connection");
    if (connection.empty())
        throw libpc_error("Oracle connection string empty! Unable to connect");

    
    std::string::size_type slash_pos = connection.find("/",0);
    std::string username = connection.substr(0,slash_pos);
    std::string::size_type at_pos = connection.find("@",slash_pos);

    std::string password = connection.substr(slash_pos+1, at_pos-slash_pos-1);
    std::string instance = connection.substr(at_pos+1);
    
    Connection con = boost::make_shared<OWConnection>(username.c_str(),password.c_str(),instance.c_str());
    
    if (con->Succeeded())
        if (m_verbose)
            std::cout << "Oracle connection succeeded" << std::endl;
    else
        throw libpc_error("Oracle connection failed");
        
    return con;
    
}

Writer::~Writer()
{
    m_connection->Commit();

    return;
}


const std::string& Writer::getName() const
{
    static std::string name("OCI Writer");
    return name;
}






void Writer::run(std::ostringstream const& command)
{
    Statement statement = Statement(m_connection->CreateStatement(command.str().c_str()));
    
    // Because of OCIGDALErrorHandler, this is going to throw if there is a 
    // problem.  When it does, the statement should go out of scope and 
    // be destroyed without leaking.
    statement->Execute();
}

void Writer::WipeBlockTable()
{
    std::ostringstream oss;
    
    std::string block_table_name = m_options.GetPTree().get<std::string>("block_table_name");
    std::string base_table_name = m_options.GetPTree().get<std::string>("base_table_name");
    std::string cloud_column_name = m_options.GetPTree().get<std::string>("cloud_column_name");
    
    oss << "DELETE FROM " << block_table_name;
    try {
        run(oss);    
    } catch (libpc_error const&) 
    {
        // if we failed, let's try dropping the spatial index for the block_table_name
        oss.str("");
        if (m_options.IsDebug()) 
            std::cout << "Dropping index " << block_table_name 
                      << "_cloud_idx for block_table_name" 
                      << std::endl;
        oss << "DROP INDEX " << block_table_name << "_cloud_idx" ;
        run(oss);
        oss.str("");
        
        oss << "DELETE FROM " << block_table_name;
        run(oss);
        oss.str("");
    }
    
    oss.str("");

    // These need to be uppercase to satisfy the PLSQL function
    cloud_column_name = to_upper(cloud_column_name);
    base_table_name = to_upper(base_table_name);
    
    oss << "declare\n"
           "begin \n"
           "  mdsys.sdo_pc_pkg.drop_dependencies('" 
           << base_table_name <<
           "', '" 
           << cloud_column_name <<
           "'); end;";
    run(oss);
    oss.str("");
    
    oss << "DROP TABLE" << block_table_name;
    run(oss);
    oss.str("");

    // Oracle upper cases the table name when inserting it in the 
    // USER_SDO_GEOM_METADATA.  We'll use std::transform to do it. 
    // See http://forums.devx.com/showthread.php?t=83058 for the 
    // technique
    
    block_table_name = to_upper(block_table_name);
    oss << "DELETE FROM USER_SDO_GEOM_METADATA WHERE TABLE_NAME='" << block_table_name << "'" ;
    run(oss);
}

void Writer::CreateBlockIndex()
{
    std::ostringstream oss;
    std::string block_table_name = m_options.GetPTree().get<std::string>("block_table_name");
    
    bool is3d = m_options.Is3d();
    oss << "CREATE INDEX "<< block_table_name << "_cloud_idx on "
        << block_table_name << "(blk_extent) INDEXTYPE IS MDSYS.SPATIAL_INDEX";
    
    if (is3d)
    {
        oss <<" PARAMETERS('sdo_indx_dims=3')";
    }
    
    run(oss);
    oss.str("");

    oss << "CREATE INDEX " << block_table_name <<"_objectid_idx on " 
        << block_table_name << "(OBJ_ID,BLK_ID) COMPRESS 2" ;
    run(oss);
    oss.str("");

    
}

void Writer::CreateSDOEntry()
{
    boost::property_tree::ptree  tree = m_options.GetPTree();    
    std::string block_table_name = tree.get<std::string>("block_table_name");

    boost::uint32_t srid = tree.get<boost::uint32_t>("srid");
    boost::uint32_t precision = tree.get<boost::uint32_t>("precision");
    
    bool bUse3d = m_options.Is3d();
    

    std::ostringstream oss;

    std::ostringstream oss_geom;
    
    oss.setf(std::ios_base::fixed, std::ios_base::floatfield);
    oss.precision(precision);

    std::ostringstream s_srid;
    

    if (srid == 0) {
        s_srid << "NULL";
    }
    else {
        s_srid << srid;
    }

    double tolerance = 0.05;
    libpc::Bounds<double> e = m_bounds;

    if (IsGeographic(srid)) {
        // FIXME: This should be overrideable
        e.setMinimum(0,-180.0); e.setMaximum(0,180.0);
        e.setMinimum(1,-90.0); e.setMaximum(1,90.0);
        e.setMinimum(2,0.0); e.setMaximum(2,20000.0);

        tolerance = 0.000000005;
    }

 
    oss <<  "INSERT INTO user_sdo_geom_metadata VALUES ('" << block_table_name <<
        "','blk_extent', MDSYS.SDO_DIM_ARRAY(";
    
    oss << "MDSYS.SDO_DIM_ELEMENT('X', " << e.getMinimum(0) << "," << e.getMaximum(0) <<"," << tolerance << "),"
           "MDSYS.SDO_DIM_ELEMENT('Y', " << e.getMinimum(1) << "," << e.getMaximum(1) <<"," << tolerance << ")";
           
    if (bUse3d) {
        oss << ",";
        oss <<"MDSYS.SDO_DIM_ELEMENT('Z', "<< e.getMinimum(2) << "," << e.getMaximum(2) << "," << tolerance << ")";
    }
    oss << ")," << s_srid.str() << ")";
    
    run(oss);
    oss.str("");
    
}

bool Writer::BlockTableExists()
{

    std::ostringstream oss;
    std::string block_table_name = m_options.GetPTree().get<std::string>("block_table_name");
    
    char szTable[OWNAME]= "";
    oss << "select table_name from user_tables where table_name like upper('%%"<< block_table_name <<"%%') ";


    Statement statement = Statement(m_connection->CreateStatement(oss.str().c_str()));
    
    // Because of OCIGDALErrorHandler, this is going to throw if there is a 
    // problem.  When it does, the statement should go out of scope and 
    // be destroyed without leaking.
    statement->Define(szTable);

    statement->Execute();
    
    
    try {
        statement->Execute();
    } catch (libpc_error const& ) {
        // Assume for now that an error returned here is OCI_NODATA, which means 
        // the table doesn't exist.  If this really isn't the case, we're going 
        // to get more legit message further down the line.
        return false;
    }  
    
    return true;
    
}

void Writer::CreateBlockTable()
{
    std::ostringstream oss;
    std::string block_table_name = m_options.GetPTree().get<std::string>("block_table_name");
    
    oss << "CREATE TABLE " << block_table_name << " AS SELECT * FROM MDSYS.SDO_PC_BLK_TABLE";
    
    run(oss);
    m_connection->Commit();
    
}

bool Writer::IsGeographic(boost::int32_t srid)

{
    typedef boost::shared_ptr<long> shared_long;
    typedef boost::shared_ptr<char> shared_char;
    
    std::ostringstream oss;
    // char* kind = (char* ) malloc (OWNAME * sizeof(char));
    shared_char kind = boost::shared_ptr<char>(new char[OWNAME]);
    oss << "SELECT COORD_REF_SYS_KIND from MDSYS.SDO_COORD_REF_SYSTEM WHERE SRID = :1";
    
    Statement statement = Statement(m_connection->CreateStatement(oss.str().c_str()));

    shared_long p_srid = boost::shared_ptr<long>(new long);
    *p_srid = srid;
    // p_srid[0] = srid;
    
    statement->Bind(p_srid.get());
    statement->Define(kind.get());    
    
    try {
        statement->Execute();
    } catch (libpc_error const& e) {
        std::ostringstream oss;
        oss << "Failed to fetch geographicness of srid " << srid << std::endl << e.what() << std::endl;
        throw std::runtime_error(oss.str());
    }  
    
    if (compare_no_case(kind.get(), "GEOGRAPHIC2D",12) == 0) {
        return true;
    }
    if (compare_no_case(kind.get(), "GEOGRAPHIC3D",12) == 0) {
        return true;
    }


    return false;
    
}

std::string Writer::LoadSQLData(std::string const& filename)
{
    if (!Utils::fileExists(filename))
    {
        std::ostringstream oss;
        oss << filename << " does not exist";
        throw libpc_error(oss.str());
    }

    std::istream::pos_type size;    
    std::istream* input = Utils::openFile(filename);
    input->seekg(std::ios::end);
    
    char* data;
    if (input->good()){
        size = input->tellg();
        data = new char [static_cast<boost::uint32_t>(size)];
        input->seekg (0, std::ios::beg);
        input->read (data, size);
        // infile->close();

        std::string output = std::string(data, (std::size_t) size);
        delete[] data;
        Utils::closeFile(input);
        return output;
    } 
    else 
    {   
        Utils::closeFile(input);
        return std::string("");
    }    
    
}

void Writer::RunFileSQL(std::string const& filename)
{
    std::ostringstream oss;
    std::string sql = m_options.GetPTree().get<std::string>(filename);
        
    if (!sql.size()) return;

    if (!Utils::fileExists(sql))
    {
        oss << sql;
    } else {
        oss << LoadSQLData(sql);  // Our "sql" is really the filename in the ptree
    }


   

    if (m_options.IsDebug())
        std::cout << "running "<< filename << " ..." <<std::endl;

    run(oss);
    
}

long Writer::GetGType()
{
    boost::property_tree::ptree  tree = m_options.GetPTree();    
    bool bUse3d = tree.get<bool>("is3d");
    bool bUseSolidGeometry = tree.get<bool>("solid");
    long gtype = 0;
    if (bUse3d) {
        if (bUseSolidGeometry == true) {
            gtype = 3008;

        } else {
            gtype = 3003;
        }
    } else {
        if (bUseSolidGeometry == true) {
            gtype = 2008;
        } else {
            gtype = 2003;
        }
    }
    
    return gtype;   
}


std::string Writer::CreatePCElemInfo()
{
    boost::property_tree::ptree  tree = m_options.GetPTree();    
    bool bUse3d = tree.get<bool>("is3d");
    bool bUseSolidGeometry = tree.get<bool>("solid");
    
    std::ostringstream s_eleminfo;
    if (bUse3d) {
        if (bUseSolidGeometry == true) {
            // s_gtype << "3008";
            s_eleminfo << "(1,1007,3)";

        } else {
            // s_gtype << "3003";
            s_eleminfo  << "(1,1003,3)";

        }
    } else {
        if (bUseSolidGeometry == true) {
            // s_gtype << "2008";
            s_eleminfo << "(1,1007,3)";

        } else {
            // s_gtype << "2003";
            s_eleminfo  << "(1,1003,3)";

        }
    }
    
    return s_eleminfo.str();
      
}

void Writer::CreatePCEntry(std::vector<boost::uint8_t> const* header_data)
{
    boost::property_tree::ptree&  tree = m_options.GetPTree();
    
    std::string block_table_name = to_upper(tree.get<std::string>("block_table_name"));
    std::string base_table_name = to_upper(tree.get<std::string>("base_table_name"));
    std::string cloud_column_name = to_upper(tree.get<std::string>("cloud_column_name"));
    std::string base_table_aux_columns = to_upper(tree.get<std::string>("base_table_aux_columns"));
    std::string base_table_aux_values = to_upper(tree.get<std::string>("base_table_aux_values"));
    std::string header_blob_column_name = to_upper(tree.get<std::string>("header_blob_column_name"));
    boost::uint32_t srid = tree.get<boost::uint32_t>("srid");
    boost::uint32_t precision = tree.get<boost::uint32_t>("precision");
    boost::uint32_t capacity = tree.get<boost::uint32_t>("capacity");
    boost::uint32_t dimensions = tree.get<boost::uint32_t>("dimensions");
    bool bUse3d = tree.get<bool>("is3d");
    
    std::ostringstream oss;


    oss.setf(std::ios_base::fixed, std::ios_base::floatfield);
    oss.precision(precision);

    std::ostringstream columns;
    std::ostringstream values;
    
    if (!base_table_aux_columns.empty() ) {
        columns << cloud_column_name << "," << base_table_aux_columns;
    
        values << "pc," << base_table_aux_values;
    } else {
        columns << cloud_column_name;
        values << "pc";
    }
    
    if (!header_blob_column_name.empty()){
        columns << "," << header_blob_column_name;
        values <<", :2";
    }


    std::ostringstream s_srid;
    std::ostringstream s_gtype;
    std::ostringstream s_eleminfo;
    std::ostringstream s_geom;


    // IsGeographic(srid);

    if (srid == 0)
    {
        s_srid << "NULL";
    }
    else
    {
        s_srid << srid;
    }

    long gtype = GetGType();
    s_gtype << gtype;
    
    std::string eleminfo = CreatePCElemInfo();

    libpc::Bounds<double> e = m_bounds;

    s_geom << "           mdsys.sdo_geometry("<<s_gtype.str() <<", "<<s_srid.str()<<", null,\n"
"              mdsys.sdo_elem_info_array"<< s_eleminfo.str() <<",\n"
"              mdsys.sdo_ordinate_array(\n";

    s_geom << e.getMinimum(0) << "," << e.getMinimum(1) << ",";

    if (bUse3d) {
        s_geom << e.getMinimum(2) << ",";
    }
    
    s_geom << e.getMaximum(0) << "," << e.getMaximum(1);

    if (bUse3d) {
        s_geom << "," << e.getMaximum(2);
    }

    s_geom << "))";

    
    
oss << "declare\n"
"  pc_id NUMBER := :1;\n"
"  pc sdo_pc;\n"

"begin\n"
"  -- Initialize the Point Cloud object.\n"
"  pc := sdo_pc_pkg.init( \n"
"          '"<< base_table_name<<"', -- Table that has the SDO_POINT_CLOUD column defined\n"
"          '"<< cloud_column_name<<"',   -- Column name of the SDO_POINT_CLOUD object\n"
"          '"<< block_table_name <<"', -- Table to store blocks of the point cloud\n"
"           'blk_capacity="<< capacity <<"', -- max # of points per block\n"
<< s_geom.str() <<
",  -- Extent\n"
"     0.5, -- Tolerance for point cloud\n"
"           "<<dimensions<<", -- Total number of dimensions\n"
"           null);\n"
"  :1 := pc.pc_id;\n"

"  -- Insert the Point Cloud object  into the \"base\" table.\n"
"  insert into " << base_table_name << " ( ID, "<< columns.str() <<
        ") values ( pc.pc_id, " << values.str() << ");\n"

"  "
"end;\n";


    int pc_id = 0;
    long output = 0;
    Statement statement = Statement(m_connection->CreateStatement(oss.str().c_str()));

    statement->Bind(&pc_id);
    if (header_data->size() != 0) {
        OCILobLocator** locator =(OCILobLocator**) VSIMalloc( sizeof(OCILobLocator*) * 1 );
        statement->Define( locator, 1 ); 

        statement->Bind((char*)&(header_data[0]),(long)header_data->size());
    
    }
    try {
        statement->Execute();
    } catch (std::runtime_error const& e) {
        std::ostringstream oss;
        oss << "Failed at creating Point Cloud entry into " << base_table_name << " table. Does the table exist? "  << e.what();
        throw std::runtime_error(oss.str());
    }
    output = pc_id;

    tree.put("cloud_id", pc_id);
    
}
void Writer::writeBegin()
{
    
    m_chipper.Chip();

    // cumulate a global bounds for the dataset
    for ( boost::uint32_t i = 0; i < m_chipper.GetBlockCount(); ++i )
    {
        const chipper::Block& b = m_chipper.GetBlock(i);
        
        // FIXME: This only gets called once!
        if (m_bounds.empty()) // If the user already set the bounds for this writer, we're using that
            m_bounds.grow(b.GetBounds());        
    }

    // Set up debugging info
    Debug();
    
    m_connection = Connect();
    
    RunFileSQL("pre_sql");
    if (!BlockTableExists())
        CreateBlockTable();
    
    m_stage.seekToPoint(0);
    return;
}


void Writer::writeEnd()
{

    return;
}

bool Writer::FillOraclePointData(PointData const& buffer, 
                                 std::vector<boost::uint8_t>& point_data,
                                 chipper::Block const& block,
                                 boost::uint32_t block_id)
{


    libpc::Schema const& schema = buffer.getSchema();
    std::vector<boost::uint32_t> ids = block.GetIDs();

    bool hasTimeData = schema.hasDimension(Dimension::Field_Time);
    
    boost::uint64_t count = buffer.getNumPoints();

    point_data.clear(); // wipe whatever was there    
    boost::uint32_t oracle_record_size = 8+8+8+8+8+4+4;
    // point_data.resize(count*oracle_record_size);
    
    // assert(count*oracle_record_size == point_data.size());

    const int indexX = schema.getDimensionIndex(Dimension::Field_X);
    const int indexY = schema.getDimensionIndex(Dimension::Field_Y);
    const int indexZ = schema.getDimensionIndex(Dimension::Field_Z);
    const int indexClassification = schema.getDimensionIndex(Dimension::Field_Classification);
    const int indexTime = schema.getDimensionIndex(Dimension::Field_Time);
    
    Dimension const& dimX = schema.getDimension(indexX);
    Dimension const& dimY = schema.getDimension(indexY);
    Dimension const& dimZ = schema.getDimension(indexZ);

    double xscale = dimX.getNumericScale();
    double yscale = dimY.getNumericScale();
    double zscale = dimY.getNumericScale();
        
    double xoffset = dimX.getNumericOffset();
    double yoffset = dimY.getNumericOffset();
    double zoffset = dimZ.getNumericOffset();
    
    boost::uint32_t counter = 0;
    std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
    std::cout.precision(6);

    while (counter < count)
    {

        double x = (buffer.getField<boost::int32_t>(counter, indexX) * xscale) + xoffset;
        double y = (buffer.getField<boost::int32_t>(counter, indexY) * yscale) + yoffset;
        double z = (buffer.getField<boost::int32_t>(counter, indexZ) * zscale) + zoffset;
        boost::uint8_t classification = buffer.getField<boost::uint8_t>(counter, indexClassification);
        double c = static_cast<double>(classification);
        boost::uint32_t id = ids[counter];
        std::cout << x <<" "<< y  <<" "<< z << " "<< id << " " << c <<std::endl;
        
        double t = 0.0;
        if (hasTimeData)
            t = buffer.getField<double>(counter, indexTime);

        boost::uint8_t* x_b =  reinterpret_cast<boost::uint8_t*>(&x);
        boost::uint8_t* y_b =  reinterpret_cast<boost::uint8_t*>(&y);
        boost::uint8_t* z_b =  reinterpret_cast<boost::uint8_t*>(&z);
        boost::uint8_t* t_b =  reinterpret_cast<boost::uint8_t*>(&t);
        boost::uint8_t* c_b =  reinterpret_cast<boost::uint8_t*>(&c);

        // big-endian
        for (int i = sizeof(double) - 1; i >= 0; i--) {
            point_data.push_back(x_b[i]);
        }

        for (int i = sizeof(double) - 1; i >= 0; i--) {
            point_data.push_back(y_b[i]);
        }   

        for (int i = sizeof(double) - 1; i >= 0; i--) {
            point_data.push_back(z_b[i]);
        }

    
        for (int i = sizeof(double) - 1; i >= 0; i--) {
            point_data.push_back(t_b[i]);
        }

        // Classification is only a single byte, but 
        // we need to store it in an 8 byte big-endian 
        // double to satisfy OPC
        for (int i = sizeof(double) - 1; i >= 0; i--) {
            point_data.push_back(c_b[i]);
        }

        boost::uint8_t* id_b = reinterpret_cast<boost::uint8_t*>(&id);
        boost::uint8_t* block_b = reinterpret_cast<boost::uint8_t*>(&block_id);
        
        // 4-byte big-endian integer for the BLK_ID value
        for (int i =  sizeof(boost::uint32_t) - 1; i >= 0; i--) {
            point_data.push_back(block_b[i]);
        }
        
        // 4-byte big-endian integer for the PT_ID value
        for (int i =  sizeof(boost::uint32_t) - 1; i >= 0; i--) {
            point_data.push_back(id_b[i]);
        }

        counter++;
    }

    assert(count*oracle_record_size == point_data.size());
    
    return true;
}

void Writer::SetElements(   Statement statement,
                            OCIArray* elem_info)
{
    

    statement->AddElement(elem_info, 1);
    bool bUseSolidGeometry = m_options.IsSolid();
    if (bUseSolidGeometry == true) {
        //"(1,1007,3)";
        statement->AddElement(elem_info, 1007);
    } else {
        //"(1,1003,3)";
        statement->AddElement(elem_info, 1003);
    }

    statement->AddElement(elem_info, 3);
 
}

void Writer::SetOrdinates(Statement statement,
                          OCIArray* ordinates, 
                          libpc::Bounds<double> const& extent)
{
    
    statement->AddElement(ordinates, extent.getMinimum(0));
    statement->AddElement(ordinates, extent.getMaximum(1));
    if (extent.dimensions().size() > 2)
        statement->AddElement(ordinates, extent.getMinimum(2));
    
    statement->AddElement(ordinates, extent.getMaximum(0));
    statement->AddElement(ordinates, extent.getMaximum(1));
    if (extent.dimensions().size() > 2)
        statement->AddElement(ordinates, extent.getMaximum(2));
        

}

bool Writer::WriteBlock(PointData const& buffer, 
                                 std::vector<boost::uint8_t>& point_data,
                                 chipper::Block const& block,
                                 boost::uint32_t block_id)
{

    boost::property_tree::ptree&  tree = m_options.GetPTree();
    
    std::string block_table_name = to_upper(tree.get<std::string>("block_table_name"));
    std::string block_table_partition_column = to_upper(tree.get<std::string>("block_table_partition_column"));
    boost::int32_t block_table_partition_value = tree.get<boost::uint32_t>("block_table_partition_value");
    boost::uint32_t srid = tree.get<boost::uint32_t>("srid");

    bool bUsePartition = block_table_partition_column.size() != 0;
    
    std::vector<boost::uint32_t> ids = block.GetIDs();

    

    std::ostringstream oss;
    std::ostringstream partition;
    
    if (bUsePartition)
    {
        partition << "," << block_table_partition_column;
    }



    // EnableTracing(connection);
    
    long gtype = GetGType();

    
    oss << "INSERT INTO "<< block_table_name << 
            "(OBJ_ID, BLK_ID, NUM_POINTS, POINTS,   "
            "PCBLK_MIN_RES, BLK_EXTENT, PCBLK_MAX_RES, NUM_UNSORTED_POINTS, PT_SORT_DIM";
    if (bUsePartition)
        oss << partition.str();
    oss << ") "
            "VALUES ( :1, :2, :3, :4, 1, mdsys.sdo_geometry(:5, :6, null,:7, :8)" 
            ", 1, 0, 1";
    if (bUsePartition)
        oss << ", :9";
            
    oss <<")";
          
    // TODO: If gotdata == false below, this memory probably leaks --mloskot
    OCILobLocator** locator =(OCILobLocator**) VSIMalloc( sizeof(OCILobLocator*) * 1 );

    Statement statement = Statement(m_connection->CreateStatement(oss.str().c_str()));
    

    long pc_id = tree.get<boost::int32_t>("cloud_id");
    long* p_pc_id = (long*) malloc( 1 * sizeof(long));
    p_pc_id[0] = pc_id;

    long* p_result_id = (long*) malloc( 1 * sizeof(long));
    p_result_id[0] = (long)block_id;
    
    long* p_num_points = (long*) malloc (1 * sizeof(long));
    p_num_points[0] = (long)ids.size();
    
    
    // :1
    statement->Bind( p_pc_id );
    
    // :2
    statement->Bind( p_result_id );

    // :3
    statement->Bind( p_num_points );
       
    // :4
    statement->Define( locator, 1 ); 

        
    // std::vector<liblas::uint8_t> data;
    // bool gotdata = GetResultData(result, reader, data, 3);
    // if (! gotdata) throw std::runtime_error("unable to fetch point data byte array");

    statement->Bind((char*)&(point_data[0]),(long)point_data.size());

    // :5
    long* p_gtype = (long*) malloc (1 * sizeof(long));
    p_gtype[0] = gtype;

    statement->Bind(p_gtype);
    
    // :6
    long* p_srid  = 0;
    
    
    if (srid != 0) {
        p_srid = (long*) malloc (1 * sizeof(long));
        p_srid[0] = srid;
    }
    statement->Bind(p_srid);
    
    // :7
    OCIArray* sdo_elem_info=0;
    m_connection->CreateType(&sdo_elem_info, m_connection->GetElemInfoType());
    SetElements(statement, sdo_elem_info);    
    statement->Bind(&sdo_elem_info, m_connection->GetElemInfoType());
    
    // :8
    OCIArray* sdo_ordinates=0;
    m_connection->CreateType(&sdo_ordinates, m_connection->GetOrdinateType());
    
     // x0, x1, y0, y1, z0, z1, bUse3d
    SetOrdinates(statement, sdo_ordinates, block.GetBounds());
    statement->Bind(&sdo_ordinates, m_connection->GetOrdinateType());
    
    // :9
    long* p_partition_d = 0;
    if (bUsePartition) {
        p_partition_d = (long*) malloc (1 * sizeof(long));
        p_partition_d[0] = block_table_partition_value;
        statement->Bind(p_partition_d);        
    }
    
    try {
        statement->Execute();
    } catch (std::runtime_error const& e) {
        std::ostringstream oss;
        oss << "Failed to insert block # into '" << block_table_name << "' table. Does the table exist? "  << std::endl << e.what() << std::endl;
        throw std::runtime_error(oss.str());
    }
    
    oss.str("");
    

    OWStatement::Free(locator, 1);

    if (p_pc_id != 0) free(p_pc_id);
    if (p_result_id != 0) free(p_result_id);
    if (p_num_points != 0) free(p_num_points);
    if (p_gtype != 0) free(p_gtype);
    if (p_srid != 0) free(p_srid);    
    if (p_partition_d != 0) free(p_partition_d);
    
    m_connection->DestroyType(&sdo_elem_info);
    m_connection->DestroyType(&sdo_ordinates);
    


    
    return true;
}

boost::uint32_t Writer::writeBuffer(const PointData& pointData)
{
    boost::uint32_t numPoints = pointData.getNumPoints();

    libpc::Header const& header = m_stage.getHeader();
    libpc::Schema const& schema = header.getSchema();

    PointData buffer(schema, 1);

    
    for ( boost::uint32_t i = 0; i < m_chipper.GetBlockCount(); ++i )
    {
        const chipper::Block& b = m_chipper.GetBlock(i);
        
        // FIXME: This should be std::min(capacity, block_id.size()) (I think)
        PointData block(schema, m_options.GetPTree().get<boost::uint32_t>("capacity"));
        std::vector<boost::uint32_t> ids = b.GetIDs();
        
        std::vector<boost::uint32_t>::const_iterator it;
        std::vector<boost::uint8_t> oracle_buffer;
        boost::uint32_t count = 0;
        for (it = ids.begin(); it != ids.end(); it++)
        {
            m_stage.seekToPoint(*it);
            m_stage.read(buffer);
            
            block.copyPointsFast(static_cast<std::size_t>(count), static_cast<std::size_t>(0), buffer, 1); // put single point onto our block

            count++;

        }
        FillOraclePointData(block, oracle_buffer, b, i);
        WriteBlock(block, oracle_buffer, b, i);
    }
    // bool hasTimeData = false;
    // bool hasColorData = false;
    // bool hasWaveData = false;
    // 
    // const liblas::PointFormatName pointFormat = m_externalHeader->GetDataFormatId();
    // switch (pointFormat)
    // {
    // case liblas::ePointFormat0:
    //     break;
    // case liblas::ePointFormat1:
    //     hasTimeData = true;
    //     break;
    // case liblas::ePointFormat2:
    //     hasColorData = true;
    //     break;
    // case liblas::ePointFormat3:
    //     hasTimeData = true;
    //     hasColorData = true;
    //     break;
    // case liblas::ePointFormat4:
    //     hasTimeData = true;
    //     hasWaveData = true;
    //     break;
    // case liblas::ePointFormat5:
    //     hasColorData = true;
    //     hasTimeData = true;
    //     hasWaveData = true;
    //     break;
    // case liblas::ePointFormatUnknown:
    //     throw not_yet_implemented("Unknown point format encountered");
    // }
    // 
    // if (hasWaveData)
    // {
    //     throw not_yet_implemented("Waveform data (types 4 and 5) not supported");
    // }
    // 
    // const Schema& schema = pointData.getSchema();
    // 
    // const int indexX = schema.getDimensionIndex(Dimension::Field_X);
    // const int indexY = schema.getDimensionIndex(Dimension::Field_Y);
    // const int indexZ = schema.getDimensionIndex(Dimension::Field_Z);
    // 
    // const int indexIntensity = schema.getDimensionIndex(Dimension::Field_Intensity);
    // const int indexReturnNumber = schema.getDimensionIndex(Dimension::Field_ReturnNumber);
    // const int indexNumberOfReturns = schema.getDimensionIndex(Dimension::Field_NumberOfReturns);
    // const int indexScanDirectionFlag = schema.getDimensionIndex(Dimension::Field_ScanDirectionFlag);
    // const int indexEdgeOfFlightLine = schema.getDimensionIndex(Dimension::Field_EdgeOfFlightLine);
    // const int indexClassification = schema.getDimensionIndex(Dimension::Field_Classification);
    // const int indexScanAngleRank = schema.getDimensionIndex(Dimension::Field_ScanAngleRank);
    // const int indexUserData = schema.getDimensionIndex(Dimension::Field_UserData);
    // const int indexPointSourceId = schema.getDimensionIndex(Dimension::Field_PointSourceId);
    // 
    // const int indexGpsTime = (hasTimeData ? schema.getDimensionIndex(Dimension::Field_GpsTime) : 0);
    // 
    // const int indexRed = (hasColorData ? schema.getDimensionIndex(Dimension::Field_Red) : 0);
    // const int indexGreen = (hasColorData ? schema.getDimensionIndex(Dimension::Field_Green) : 0);
    // const int indexBlue = (hasColorData ? schema.getDimensionIndex(Dimension::Field_Blue) : 0);
    // 
    // //const int indexWavePacketDescriptorIndex = (hasWaveData ? schema.getDimensionIndex(Dimension::Field_WavePacketDescriptorIndex) : 0);
    // //const int indexWaveformDataOffset = (hasWaveData ? schema.getDimensionIndex(Dimension::Field_WaveformDataOffset) : 0);
    // //const int indexReturnPointWaveformLocation = (hasWaveData ? schema.getDimensionIndex(Dimension::Field_ReturnPointWaveformLocation) : 0);
    // //const int indexWaveformXt = (hasWaveData ? schema.getDimensionIndex(Dimension::Field_WaveformXt) : 0);
    // //const int indexWaveformYt = (hasWaveData ? schema.getDimensionIndex(Dimension::Field_WaveformYt) : 0);
    // //const int indexWaveformZt = (hasWaveData ? schema.getDimensionIndex(Dimension::Field_WaveformZt) : 0);
    // 
    // liblas::Point pt;
    // 
    // boost::uint32_t numPoints = pointData.getNumPoints();
    // for (boost::uint32_t i=0; i<numPoints; i++)
    // {
    //     const boost::int32_t x = pointData.getField<boost::int32_t>(i, indexX);
    //     const boost::int32_t y = pointData.getField<boost::int32_t>(i, indexY);
    //     const boost::int32_t z = pointData.getField<boost::int32_t>(i, indexZ);
    //     pt.SetRawX(x);
    //     pt.SetRawY(y);
    //     pt.SetRawZ(z);
    // 
    //     const boost::uint16_t intensity = pointData.getField<boost::uint16_t>(i, indexIntensity);
    //     const boost::int8_t returnNumber = pointData.getField<boost::int8_t>(i, indexReturnNumber);
    //     const boost::int8_t numberOfReturns = pointData.getField<boost::int8_t>(i, indexNumberOfReturns);
    //     const boost::int8_t scanDirFlag = pointData.getField<boost::int8_t>(i, indexScanDirectionFlag);
    //     const boost::int8_t edgeOfFlightLine = pointData.getField<boost::int8_t>(i, indexEdgeOfFlightLine);
    //     const boost::uint8_t classification = pointData.getField<boost::uint8_t>(i, indexClassification);
    //     const boost::int8_t scanAngleRank = pointData.getField<boost::int8_t>(i, indexScanAngleRank);
    //     const boost::uint8_t userData = pointData.getField<boost::uint8_t>(i, indexUserData);
    //     const boost::uint16_t pointSourceId = pointData.getField<boost::uint16_t>(i, indexPointSourceId);
    //     pt.SetIntensity(intensity);
    //     pt.SetReturnNumber(returnNumber);
    //     pt.SetNumberOfReturns(numberOfReturns);
    //     pt.SetScanDirection(scanDirFlag);
    //     pt.SetFlightLineEdge(edgeOfFlightLine);
    //     pt.SetClassification(classification);
    //     pt.SetScanAngleRank(scanAngleRank);
    //     pt.SetUserData(userData);
    //     pt.SetPointSourceID(pointSourceId);
    // 
    //     if (hasTimeData)
    //     {
    //         const double gpsTime = pointData.getField<double>(i, indexGpsTime);
    //         pt.SetTime(gpsTime);
    //     }
    // 
    //     if (hasColorData)
    //     {
    //         const boost::uint16_t red = pointData.getField<boost::uint16_t>(i, indexRed);
    //         const boost::uint16_t green = pointData.getField<boost::uint16_t>(i, indexGreen);
    //         const boost::uint16_t blue = pointData.getField<boost::uint16_t>(i, indexBlue);
    //         liblas::Color color(red, green, blue);
    //         pt.SetColor(color);
    //     }
    // 
    //     if (hasWaveData)
    //     {
    //         assert(false);
    //     }
    // 
    //     bool ok = m_externalWriter->WritePoint(pt);
    //     assert(ok); // BUG
    // }
    // 
    // return numPoints;
    return numPoints;
}

void Writer::Debug()
{
    bool debug = m_options.IsDebug();
    

    
    if (debug)
    {
        m_verbose = true;
    }

    CPLPopErrorHandler();

    if (debug)
    {
        const char* gdal_debug = getenv("CPL_DEBUG");
        if (gdal_debug == 0)
        {
            // FIXME: this leaks
            const std::string* d = new std::string("CPL_DEBUG=ON");
            putenv(const_cast<char*>(d->c_str()));
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





}}} // namespace libpc::driver::oci
