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


#include <pdal/drivers/oci/Writer.hpp>

#include <iostream>


#include <pdal/exceptions.hpp>

#include <fstream>

namespace pdal { namespace drivers { namespace oci {


pdal::Schema Get8DimensionFixedSchema()
{
    std::ostringstream text;
    pdal::Schema schema;
    
    Dimension x(Dimension::Field_X, Dimension::Double);
    text << "x coordinate as a long integer.  You must use the scale and "
         << "offset information of the header to determine the double value.";
    x.setDescription(text.str());
    x.setEndianness(pdal::Endian_Big);
    schema.addDimension(x);
    text.str("");

    Dimension y(Dimension::Field_Y, Dimension::Double);
    text << "y coordinate as a long integer.  You must use the scale and "
         << "offset information of the header to determine the double value.";
    y.setDescription(text.str());
    y.setEndianness(pdal::Endian_Big);
    schema.addDimension(y);
    text.str("");

    Dimension z(Dimension::Field_Z, Dimension::Double);
    text << "z coordinate as a long integer.  You must use the scale and "
         << "offset information of the header to determine the double value.";
    z.setDescription(text.str());
    z.setEndianness(pdal::Endian_Big);
    schema.addDimension(z);
    text.str("");

    Dimension t(Dimension::Field_Time, Dimension::Double);
    text << "The GPS Time is the double floating point time tag value at "
        "which the point was acquired. It is GPS Week Time if the "
        "Global Encoding low bit is clear and Adjusted Standard GPS "
        "Time if the Global Encoding low bit is set (see Global Encoding "
        "in the Public Header Block description).";
    t.setDescription(text.str());
    t.setEndianness(pdal::Endian_Big);
    schema.addDimension(t);
    text.str("");

    Dimension classification(Dimension::Field_Classification, Dimension::Double);
    text << "Classification in LAS 1.0 was essentially user defined and optional. "
         "LAS 1.1 defines a standard set of ASPRS classifications. In addition, "
         "the field is now mandatory. If a point has never been classified, this "
         "byte must be set to zero. There are no user defined classes since "
         "both point format 0 and point format 1 supply 8 bits per point for "
         "user defined operations. Note that the format for classification is a "
         "bit encoded field with the lower five bits used for class and the "
         "three high bits used for flags.";
    classification.setDescription(text.str());
    classification.setEndianness(pdal::Endian_Big);
    schema.addDimension(classification);
    text.str("");
    
    Dimension intensity(Dimension::Field_Intensity, Dimension::Double);
    text << "The intensity value is the integer representation of the pulse "
         "return magnitude. This value is optional and system specific. "
         "However, it should always be included if available.";
    intensity.setDescription(text.str());
    intensity.setEndianness(pdal::Endian_Big);
    schema.addDimension(intensity);
    text.str("");

    Dimension return_no(Dimension::Field_ReturnNumber, Dimension::Uint8); // 3 bits only
    text << "Return Number: The Return Number is the pulse return number for "
         "a given output pulse. A given output laser pulse can have many "
         "returns, and they must be marked in sequence of return. The first "
         "return will have a Return Number of one, the second a Return "
         "Number of two, and so on up to five returns.";
    return_no.setDescription(text.str());
    return_no.setEndianness(pdal::Endian_Big);
    schema.addDimension(return_no);
    text.str("");

    Dimension no_returns(Dimension::Field_NumberOfReturns, Dimension::Uint8); // 3 bits only
    text << "Number of Returns (for this emitted pulse): The Number of Returns "
         "is the total number of returns for a given pulse. For example, "
         "a laser data point may be return two (Return Number) within a "
         "total number of five returns.";
    no_returns.setDescription(text.str());
    no_returns.setEndianness(pdal::Endian_Big);
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
    scan_dir.setEndianness(pdal::Endian_Big);
    schema.addDimension(scan_dir);
    text.str("");

    Dimension edge(Dimension::Field_EdgeOfFlightLine, Dimension::Uint8); // 1 bit only
    text << "The Edge of Flight Line data bit has a value of 1 only when "
         "the point is at the end of a scan. It is the last point on "
         "a given scan line before it changes direction.";
    edge.setDescription(text.str());
    edge.setEndianness(pdal::Endian_Big);
    schema.addDimension(edge);
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
    scan_angle.setEndianness(pdal::Endian_Big);
    schema.addDimension(scan_angle);
    text.str("");

    Dimension user_data(Dimension::Field_UserData, Dimension::Uint8);
    text << "This field may be used at the userís discretion";
    user_data.setDescription(text.str());
    user_data.setEndianness(pdal::Endian_Big);
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
    point_source_id.setEndianness(pdal::Endian_Big);
    schema.addDimension(point_source_id);
    text.str("");



    Dimension red(Dimension::Field_Red, Dimension::Uint16);
    text << "The red image channel value associated with this point";
    red.setDescription(text.str());
    red.setEndianness(pdal::Endian_Big);
    schema.addDimension(red);
    text.str("");

    Dimension green(Dimension::Field_Green, Dimension::Uint16);
    text << "The green image channel value associated with this point";
    green.setDescription(text.str());
    green.setEndianness(pdal::Endian_Big);
    schema.addDimension(green);
    text.str("");

    Dimension blue(Dimension::Field_Blue, Dimension::Uint16);
    text << "The blue image channel value associated with this point";
    blue.setDescription(text.str());
    blue.setEndianness(pdal::Endian_Big);
    schema.addDimension(blue);
    text.str("");

    Dimension alpha(Dimension::Field_Alpha, Dimension::Uint16);
    text << "The alpha image channel value associated with this point";
    alpha.setDescription(text.str());
    alpha.setEndianness(pdal::Endian_Big);
    schema.addDimension(alpha);
    text.str("");

    Dimension blk_id(Dimension::Field_User1, Dimension::Uint32);
    text << "The block id for this point";
    blk_id.setDescription(text.str());
    blk_id.setEndianness(pdal::Endian_Big);
    schema.addDimension(blk_id);
    text.str("");

    Dimension pt_id(Dimension::Field_User2, Dimension::Uint32);
    text << "The point id for this point";
    pt_id.setDescription(text.str());
    pt_id.setEndianness(pdal::Endian_Big);
    schema.addDimension(pt_id);
    text.str("");
        
    return schema;
}

std::string ReadFile(std::string filename)
{

    std::istream* infile = Utils::openFile(filename, true);
    std::ifstream::pos_type size;
    // char* data;
    std::vector<char> data;
    if (infile->good()){
        infile->seekg(0, std::ios::end);
        size = infile->tellg();
        data.resize(static_cast<std::vector<char>::size_type>(size));
        // data = new char [size];
        infile->seekg (0, std::ios::beg);
        infile->read (&data.front(), size);
        // infile->close();

        // delete[] data;
        delete infile;
        return std::string(&data[0], data.size());
        // return data; 
    } 
    else 
    {   
        throw pdal_error("unable to open file!");
        // return data;
    }
    
}

Writer::Writer(Stage& prevStage, OptionsOld& optionsOld)
    : pdal::Writer(prevStage)
    , m_stage(prevStage)
    , m_optionsOld(optionsOld)
    , m_verbose(false)
    , m_doCreateIndex(false)
{

    Debug();
    
    m_connection = Connect(m_optionsOld);
    
    boost::uint32_t capacity = m_optionsOld.GetPTree().get<boost::uint32_t>("capacity");
    setChunkSize(capacity);
    return;
}



Writer::~Writer()
{
    m_connection->Commit();

    return;
}


const std::string& Writer::getDescription() const
{
    static std::string name("OCI Writer");
    return name;
}

const std::string& Writer::getName() const
{
    static std::string name("drivers.oci.writer");
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
    
    std::string block_table_name = m_optionsOld.GetPTree().get<std::string>("block_table_name");
    std::string base_table_name = m_optionsOld.GetPTree().get<std::string>("base_table_name");
    std::string cloud_column_name = m_optionsOld.GetPTree().get<std::string>("cloud_column_name");
    
    oss << "DELETE FROM " << block_table_name;
    try {
        run(oss);    
    } catch (pdal_error const&) 
    {
        // if we failed, let's try dropping the spatial index for the block_table_name
        oss.str("");
        if (isDebug()) 
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
    
    oss << "DROP TABLE " << block_table_name;
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

bool Writer::isVerbose() const
{
    return m_optionsOld.GetPTree().get<bool>("verbose");
}

bool Writer::isDebug() const
{
    return m_optionsOld.GetPTree().get<bool>("debug");
}

bool Writer::is3d() const
{
    return m_optionsOld.GetPTree().get<bool>("is3d");
}

bool Writer::isSolid() const
{
    return m_optionsOld.GetPTree().get<bool>("solid");
}


void Writer::CreateBlockIndex()
{
    std::ostringstream oss;
    std::string block_table_name = m_optionsOld.GetPTree().get<std::string>("block_table_name");
    
    bool bUse3d = is3d();
    oss << "CREATE INDEX "<< block_table_name << "_cloud_idx on "
        << block_table_name << "(blk_extent) INDEXTYPE IS MDSYS.SPATIAL_INDEX";
    
    if (bUse3d)
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
    boost::property_tree::ptree  tree = m_optionsOld.GetPTree();    
    std::string block_table_name = tree.get<std::string>("block_table_name");

    boost::uint32_t srid = tree.get<boost::uint32_t>("srid");
    boost::uint32_t precision = tree.get<boost::uint32_t>("precision");
    
    bool bUse3d = is3d();
    

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
    

    pdal::Bounds<double> e = m_bounds;

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
    std::string block_table_name = m_optionsOld.GetPTree().get<std::string>("block_table_name");
    
    char szTable[OWNAME]= "";
    oss << "select table_name from user_tables where table_name like upper('%%"<< block_table_name <<"%%') ";


    Statement statement = Statement(m_connection->CreateStatement(oss.str().c_str()));
    
    // Because of OCIGDALErrorHandler, this is going to throw if there is a 
    // problem.  When it does, the statement should go out of scope and 
    // be destroyed without leaking.
    statement->Define(szTable);    
    
    try {
        statement->Execute();
    } catch (pdal_error const& ) {
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
    std::string block_table_name = m_optionsOld.GetPTree().get<std::string>("block_table_name");
    
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
    } catch (pdal_error const& e) {
        std::ostringstream oss;
        oss << "Failed to fetch geographicness of srid " << srid << std::endl << e.what() << std::endl;
        throw std::runtime_error(oss.str());
    }  
    
    if (compare_no_case(kind.get(), "GEOGRAPHIC2D") == 0) {
        return true;
    }
    if (compare_no_case(kind.get(), "GEOGRAPHIC3D") == 0) {
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
        throw pdal_error(oss.str());
    }

    std::istream::pos_type size;    
    std::istream* input = Utils::openFile(filename, true);
    

    if (input->good()) {
        std::string output;
        std::string line;
        while ( input->good() )
        {
            getline(*input, line);
            if (output.size())
            {
                output = output + "\n" + line;            
            }
            else {
                output = line;
            }
        }
    
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
    std::string sql = m_optionsOld.GetPTree().get<std::string>(filename);
        
    if (!sql.size()) return;

    if (!Utils::fileExists(sql))
    {
        oss << sql;
    } else {
        oss << LoadSQLData(sql);  // Our "sql" is really the filename in the ptree
    }


   

    if (isDebug())
        std::cout << "running "<< filename << " ..." <<std::endl;

    run(oss);
    
}

long Writer::GetGType()
{
    boost::property_tree::ptree  tree = m_optionsOld.GetPTree();    
    bool bUse3d = is3d();
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
    boost::property_tree::ptree  tree = m_optionsOld.GetPTree();    
    bool bUse3d = is3d();
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
    boost::property_tree::ptree&  tree = m_optionsOld.GetPTree();
    
    std::string block_table_name = to_upper(tree.get<std::string>("block_table_name"));
    std::string base_table_name = to_upper(tree.get<std::string>("base_table_name"));
    std::string cloud_column_name = to_upper(tree.get<std::string>("cloud_column_name"));
    std::string base_table_aux_columns = to_upper(tree.get<std::string>("base_table_aux_columns"));
    std::string base_table_aux_values = to_upper(tree.get<std::string>("base_table_aux_values"));
    std::string header_blob_column_name = to_upper(tree.get<std::string>("header_blob_column_name"));
    std::string base_table_boundary_column = to_upper(tree.get<std::string>("base_table_boundary_column"));
    std::string base_table_boundary_wkt = tree.get<std::string>("base_table_boundary_wkt");
    std::string point_schema_override = tree.get<std::string>("point_schema_override");
    
    boost::uint32_t srid = tree.get<boost::uint32_t>("srid");
    boost::uint32_t precision = tree.get<boost::uint32_t>("precision");
    boost::uint32_t capacity = tree.get<boost::uint32_t>("capacity");
    boost::uint32_t dimensions = tree.get<boost::uint32_t>("dimensions");
    bool bUse3d = is3d();

    bool bHaveSchemaOverride = (point_schema_override.size() > 0);
    
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

    int nPCPos = 1;
    int nSchemaPos = 1;
    // if (bHaveSchemaOverride)
        nSchemaPos++;

    int nPos = nSchemaPos; // Bind column position    
    // if (!header_blob_column_name.empty()){
    //     columns << "," << header_blob_column_name;
    //     values <<", :" << nPos;
    // }

    if (!base_table_boundary_column.empty()){
        columns << "," << base_table_boundary_column;
        nPos++;
        values <<", SDO_GEOMETRY(:"<<nPos;
        nPos++;
        values <<", :"<<nPos<<")";
    }
    


    std::ostringstream s_srid;
    std::ostringstream s_geom;
    std::ostringstream s_schema;
    

    // IsGeographic(srid);

    if (srid == 0)
    {
        s_srid << "NULL";
    }
    else
    {
        s_srid << srid;
    }

    // if (bHaveSchemaOverride)
    // {
        s_schema << "xmltype(:"<<nSchemaPos<<")";
    // } else
    // {
    //     s_schema << "NULL";
    // }
    
    long gtype = GetGType();
    
    std::string eleminfo = CreatePCElemInfo();

    std::string bounds_string  = tree.get<std::string>("base_table_bounds");
    std::stringstream ss(bounds_string, std::stringstream::in | std::stringstream::out);
    pdal::Bounds<double> e;
    ss >> e;

    s_geom << "           mdsys.sdo_geometry("<< gtype <<", "<<s_srid.str()<<", null,\n"
"              mdsys.sdo_elem_info_array"<< eleminfo <<",\n"
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
"  pc_id NUMBER := :"<<nPCPos<<";\n"
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
"           NULL,"
"            NULL,"
"            "<< s_schema.str() <<");\n"
"  :"<<nPCPos<<" := pc.pc_id;\n"

"  -- Insert the Point Cloud object  into the \"base\" table.\n"
"  insert into " << base_table_name << " ( ID, "<< columns.str() <<
        ") values ( pc.pc_id, " << values.str() << ");\n"

"  "
"end;\n";


    int pc_id = 0;
    long output = 0;
    Statement statement = Statement(m_connection->CreateStatement(oss.str().c_str()));

    statement->Bind(&pc_id);


    OCILobLocator* schema_locator ; 
    OCILobLocator* boundary_locator ; 

    std::string schema_data;
    if (bHaveSchemaOverride)
    {
        schema_data = ReadFile(point_schema_override);
    } else {
        schema_data = pdal::Schema::to_xml(m_stage.getSchema());
    }

    char* schema = (char*) malloc(schema_data.size() * sizeof(char) + 1);
    strncpy(schema, schema_data.c_str(), schema_data.size());
    schema[schema_data.size()] = '\0';
    statement->WriteCLob( &schema_locator, schema ); 
    statement->Bind(&schema_locator);


    // if (header_data->size() != 0) 
    // {
    //     OCILobLocator** locator =(OCILobLocator**) VSIMalloc( sizeof(OCILobLocator*) * 1 );
    //     statement->Define( locator, 1 ); 
    //     statement->Bind((char*)&(header_data[0]),(long)header_data->size());
    // }

    std::ostringstream wkt_s;

    if (!Utils::fileExists(base_table_boundary_wkt))
    {
        wkt_s << base_table_boundary_wkt;
    } else {
        wkt_s << LoadSQLData(base_table_boundary_wkt);
    }
    
    std::string wkt_string = wkt_s.str();
    char* wkt = (char*) malloc(wkt_string.size() * sizeof(char)+1);
    strncpy(wkt, wkt_string.c_str(), wkt_string.size());
    wkt[wkt_string.size()] = '\0';
    if (!base_table_boundary_column.empty())
    {
        statement->WriteCLob( &boundary_locator, wkt ); 
        statement->Bind(&boundary_locator);
        statement->Bind((long int*)&srid);

    }

    try {
        statement->Execute();
    } catch (std::runtime_error const& e) {
        std::ostringstream oss;
        oss << "Failed at creating Point Cloud entry into " << base_table_name << " table. Does the table exist? "  << e.what();
        throw std::runtime_error(oss.str());
    }
    output = pc_id;
    
    free(wkt);
    tree.put("cloud_id", pc_id);
    
}
void Writer::writeBegin()
{
    
    // m_chipper.Chip();
    // 
    // // cumulate a global bounds for the dataset
    // for ( boost::uint32_t i = 0; i < m_chipper.GetBlockCount(); ++i )
    // {
    //     const filters::chipper::Block& b = m_chipper.GetBlock(i);
    //     
    //     // FIXME: This only gets called once!
    //     if (m_bounds.empty()) // If the user already set the bounds for this writer, we're using that
    //         m_bounds.grow(b.GetBounds());        
    // }

    // Set up debugging info

    
    if (m_optionsOld.GetPTree().get<bool>("overwrite"))
    {
        if (BlockTableExists())
        {
            WipeBlockTable();
        }
    }
    
    RunFileSQL("pre_sql");
    if (!BlockTableExists())
    {
        m_doCreateIndex = true;
        CreateBlockTable();
    }
        
    CreatePCEntry(0);
    
    return;
}


void Writer::writeEnd()
{

    if (m_doCreateIndex)
    {
        CreateSDOEntry();
        CreateBlockIndex();
    }
    RunFileSQL("post_block_sql");
    return;
}

bool Writer::FillOraclePointBuffer(PointBuffer const& buffer, 
                                 std::vector<boost::uint8_t>& point_data
)
{


    pdal::Schema const& schema = buffer.getSchema();
    // std::vector<boost::uint32_t> ids = block.GetIDs();

    bool hasTimeData = schema.hasDimension(Dimension::Field_Time, Dimension::Double);
    bool hasColorData = schema.hasDimension(Dimension::Field_Red, Dimension::Uint16);
    
    boost::uint64_t count = buffer.getNumPoints();

    point_data.clear(); // wipe whatever was there    
    boost::uint32_t oracle_record_size = (8*8)+4+4;
    // point_data.resize(count*oracle_record_size);
    
    // assert(count*oracle_record_size == point_data.size());

    const int indexX = schema.getDimensionIndex(Dimension::Field_X, Dimension::Int32);
    const int indexY = schema.getDimensionIndex(Dimension::Field_Y, Dimension::Int32);
    const int indexZ = schema.getDimensionIndex(Dimension::Field_Z, Dimension::Int32);
    const int indexClassification = schema.getDimensionIndex(Dimension::Field_Classification, Dimension::Uint8);
    const int indexTime = schema.getDimensionIndex(Dimension::Field_Time, Dimension::Double);
    const int indexIntensity = schema.getDimensionIndex(Dimension::Field_Intensity, Dimension::Uint16);
    const int indexReturnNumber = schema.getDimensionIndex(Dimension::Field_ReturnNumber, Dimension::Uint8);
    const int indexNumberOfReturns = schema.getDimensionIndex(Dimension::Field_NumberOfReturns, Dimension::Uint8);
    const int indexScanDirectionFlag = schema.getDimensionIndex(Dimension::Field_ScanDirectionFlag, Dimension::Uint8);
    const int indexEdgeOfFlightLine = schema.getDimensionIndex(Dimension::Field_EdgeOfFlightLine, Dimension::Uint8);
    const int indexUserData = schema.getDimensionIndex(Dimension::Field_UserData, Dimension::Uint8);
    const int indexPointSourceId = schema.getDimensionIndex(Dimension::Field_PointSourceId, Dimension::Uint16);
    const int indexScanAngleRank = schema.getDimensionIndex(Dimension::Field_ScanAngleRank, Dimension::Int8);
    const int indexRed = schema.getDimensionIndex(Dimension::Field_Red, Dimension::Uint16);
    const int indexGreen = schema.getDimensionIndex(Dimension::Field_Green, Dimension::Uint16);
    const int indexBlue = schema.getDimensionIndex(Dimension::Field_Blue, Dimension::Uint16);






    
    // "Global" ids from the chipper are also available here.
    // const int indexId = schema.getDimensionIndex(Dimension::Field_User1, Dimension::Int32);
    const int indexBlockId = schema.getDimensionIndex(Dimension::Field_User2, Dimension::Int32);
    
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
    // std::cout.setf(std::ios_base::fixed, std::ios_base::floatfield);
    // std::cout.precision(6);

    while (counter < count)
    {

        double x = (buffer.getField<boost::int32_t>(counter, indexX) * xscale) + xoffset;
        double y = (buffer.getField<boost::int32_t>(counter, indexY) * yscale) + yoffset;
        double z = (buffer.getField<boost::int32_t>(counter, indexZ) * zscale) + zoffset;
        boost::uint8_t classification = buffer.getField<boost::uint8_t>(counter, indexClassification);
        double c = static_cast<double>(classification);
        
        boost::uint16_t i = buffer.getField<boost::uint16_t>(counter, indexIntensity);
        double intensity = static_cast<double>(i);
        
        // boost::uint32_t id = buffer.getField<boost::uint32_t>(counter, indexId);
        boost::uint32_t block_id = buffer.getField<boost::uint32_t>(counter, indexBlockId);
        // boost::uint32_t id = ids[counter];
        // std::cout << x <<" "<< y  <<" "<< z << " "<< id << " " << block_id <<" " << c <<std::endl;

        boost::uint8_t returnNumber = buffer.getField<boost::uint8_t>(counter, indexReturnNumber);
        boost::uint8_t numberOfReturns = buffer.getField<boost::uint8_t>(counter, indexNumberOfReturns);
        boost::uint8_t scanDirFlag = buffer.getField<boost::uint8_t>(counter, indexScanDirectionFlag);
        boost::uint8_t edgeOfFlightLine = buffer.getField<boost::uint8_t>(counter, indexEdgeOfFlightLine);
        boost::int8_t scanAngleRank = buffer.getField<boost::int8_t>(counter, indexScanAngleRank);

        boost::uint8_t userData = buffer.getField<boost::uint8_t>(counter, indexUserData);
        boost::uint16_t pointSourceId = buffer.getField<boost::uint16_t>(counter, indexPointSourceId);

        
        double t = 0.0;
        if (hasTimeData)
            t = buffer.getField<double>(counter, indexTime);

        boost::uint16_t red(0), green(0), blue(0), alpha(0);
        if (hasColorData)
        {
            red = buffer.getField<boost::uint16_t>(counter, indexRed);
            green = buffer.getField<boost::uint16_t>(counter, indexGreen);
            blue = buffer.getField<boost::uint16_t>(counter, indexBlue);
            
        }

        boost::uint8_t* x_b =  reinterpret_cast<boost::uint8_t*>(&x);
        boost::uint8_t* y_b =  reinterpret_cast<boost::uint8_t*>(&y);
        boost::uint8_t* z_b =  reinterpret_cast<boost::uint8_t*>(&z);
        boost::uint8_t* t_b =  reinterpret_cast<boost::uint8_t*>(&t);
        boost::uint8_t* c_b =  reinterpret_cast<boost::uint8_t*>(&c);

        boost::uint8_t* intensity_b =  reinterpret_cast<boost::uint8_t*>(&intensity);
    
        boost::uint8_t* returnNumber_b =  reinterpret_cast<boost::uint8_t*>(&returnNumber);
        boost::uint8_t* numberOfReturns_b =  reinterpret_cast<boost::uint8_t*>(&numberOfReturns);
        boost::uint8_t* scanDirFlag_b =  reinterpret_cast<boost::uint8_t*>(&scanDirFlag);
        boost::uint8_t* edgeOfFlightLine_b =  reinterpret_cast<boost::uint8_t*>(&edgeOfFlightLine);
        boost::uint8_t* scanAngleRank_b =  reinterpret_cast<boost::uint8_t*>(&scanAngleRank);
        boost::uint8_t* userData_b =  reinterpret_cast<boost::uint8_t*>(&userData);
        boost::uint8_t* pointSourceId_b =  reinterpret_cast<boost::uint8_t*>(&pointSourceId);


        boost::uint8_t* red_b =  reinterpret_cast<boost::uint8_t*>(&red);
        boost::uint8_t* green_b =  reinterpret_cast<boost::uint8_t*>(&green);
        boost::uint8_t* blue_b =  reinterpret_cast<boost::uint8_t*>(&blue);
        boost::uint8_t* alpha_b =  reinterpret_cast<boost::uint8_t*>(&alpha);
 

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

        // Intensity is only two bytes, but 
        // we need to store it in an 8 byte big-endian 
        // double to satisfy OPC
        for (int i = sizeof(double) - 1; i >= 0; i--) {
            point_data.push_back(intensity_b[i]);
        }


        // Pack dimension with a number of fields totalling 8 bytes
        point_data.push_back(returnNumber_b[0]);
        point_data.push_back(numberOfReturns_b[0]);
        point_data.push_back(scanDirFlag_b[0]);
        point_data.push_back(edgeOfFlightLine_b[0]);
        point_data.push_back(scanAngleRank_b[0]);
        point_data.push_back(userData_b[0]);
        for (int i = sizeof(uint16_t) - 1; i >= 0; i--) {
            point_data.push_back(pointSourceId_b[i]);
        }

        // Pack dimension with RGBA for a total of 8 bytes
        for (int i = sizeof(uint16_t) - 1; i >= 0; i--) {
            point_data.push_back(red_b[i]);
        }    
        for (int i = sizeof(uint16_t) - 1; i >= 0; i--) {
            point_data.push_back(green_b[i]);
        }    
        for (int i = sizeof(uint16_t) - 1; i >= 0; i--) {
            point_data.push_back(blue_b[i]);
        }    
        for (int i = sizeof(uint16_t) - 1; i >= 0; i--) {
            point_data.push_back(alpha_b[i]);
        }    


        boost::uint8_t* id_b = reinterpret_cast<boost::uint8_t*>(&counter); 
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
    bool bUseSolidGeometry = isSolid();
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
                          pdal::Bounds<double> const& extent)
{
    
    // std::cout << extent << std::endl;
    statement->AddElement(ordinates, extent.getMinimum(0));
    statement->AddElement(ordinates, extent.getMaximum(1));
    if (extent.dimensions().size() > 2)
        statement->AddElement(ordinates, extent.getMinimum(2));
    
    statement->AddElement(ordinates, extent.getMaximum(0));
    statement->AddElement(ordinates, extent.getMaximum(1));
    if (extent.dimensions().size() > 2)
        statement->AddElement(ordinates, extent.getMaximum(2));
        

}

bool Writer::WriteBlock(PointBuffer const& buffer)
{
    
    boost::uint8_t* point_data = buffer.getData(0);
    
    boost::property_tree::ptree&  tree = m_optionsOld.GetPTree();
    
    std::string block_table_name = to_upper(tree.get<std::string>("block_table_name"));
    std::string block_table_partition_column = to_upper(tree.get<std::string>("block_table_partition_column"));
    boost::int32_t block_table_partition_value = tree.get<boost::uint32_t>("block_table_partition_value");
    boost::uint32_t srid = tree.get<boost::uint32_t>("srid");

    bool bUsePartition = block_table_partition_column.size() != 0;
    
    // std::vector<boost::uint32_t> ids = block.GetIDs();
    
    // Pluck the block id out of the first point in the buffer
    pdal::Schema const& schema = buffer.getSchema();
    const int indexBlockId = schema.getDimensionIndex(Dimension::Field_User2, Dimension::Int32);
    boost::int32_t block_id  = buffer.getField<boost::int32_t>(0, indexBlockId);
    
    SWAP_ENDIANNESS(block_id); //We've already swapped these data, but we need to write a real number here.
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
    p_num_points[0] = (long)buffer.getNumPoints();
    
    // std::cout << "point count on write: " << buffer.getNumPoints() << std::endl;
    
    
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

    statement->Bind((char*)point_data,(long)(buffer.getSchemaLayout().getByteSize()*buffer.getNumPoints()));

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
    SetOrdinates(statement, sdo_ordinates, buffer.getSpatialBounds());
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

boost::uint32_t Writer::writeBuffer(const PointBuffer& buffer)
{
    boost::uint32_t numPoints = buffer.getNumPoints();

    WriteBlock(buffer);

    return numPoints;
}

void Writer::Debug()
{
    bool debug = isDebug();
    

    
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
        
        // const char* gdal_debug2 = getenv("CPL_DEBUG");
        // std::cout << "Setting GDAL debug handler CPL_DEBUG=" << gdal_debug2 << std::endl;
        CPLPushErrorHandler(OCIGDALDebugErrorHandler);
        
    }
    else 
    {
        CPLPushErrorHandler(OCIGDALErrorHandler);        
    }
}





}}} // namespace pdal::driver::oci
