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


#include <pdal/Vector.hpp>
#include <pdal/Bounds.hpp>
#include <pdal/FileUtils.hpp>
#include <pdal/PointBuffer.hpp>
#include <pdal/StageFactory.hpp>

#include <pdal/drivers/oci/Writer.hpp>

#include <fstream>

#include <boost/algorithm/string.hpp>
#ifdef PDAL_HAVE_GDAL
#include <ogr_api.h>
#endif


#ifdef USE_PDAL_PLUGIN_OCI
MAKE_WRITER_CREATOR(ociWriter, pdal::drivers::oci::Writer)
CREATE_WRITER_PLUGIN(oci, pdal::drivers::oci::Writer)
#endif


namespace pdal
{
namespace drivers
{
namespace oci
{


Writer::Writer(Stage& prevStage, const Options& options)
    : pdal::Writer(prevStage, options)
    , OracleDriver(getOptions())
    , m_doCreateIndex(false)
    , m_bHaveOutputTable(false)
    , m_pc_id(0)
    , m_srid(0)
    , m_gtype(0)
    , m_is3d(false)
    , m_issolid(false)
    , m_sdo_pc_is_initialized(false)
{

    m_connection = connect();

    boost::uint32_t capacity = getOptions().getValueOrDefault<boost::uint32_t>("capacity", 0);

    m_block_table_name = boost::to_upper_copy(getOptions().getValueOrThrow<std::string>("block_table_name"));
    m_block_table_partition_column = boost::to_upper_copy(getDefaultedOption<std::string>("block_table_partition_column"));
    m_block_table_partition_value = getDefaultedOption<boost::uint32_t>("block_table_partition_value");
    m_srid = getOptions().getValueOrThrow<boost::uint32_t>("srid");
    m_gtype = GetGType();
    m_is3d = is3d();
    m_issolid = isSolid();
    m_base_table_name = boost::to_upper_copy(getOptions().getValueOrThrow<std::string>("base_table_name"));
    m_cloud_column_name = boost::to_upper_copy(getOptions().getValueOrThrow<std::string>("cloud_column_name"));
    m_base_table_aux_columns = getDefaultedOption<std::string>("base_table_aux_columns");
    m_base_table_aux_values = getDefaultedOption<std::string>("base_table_aux_values");

    m_base_table_boundary_column = getDefaultedOption<std::string>("base_table_boundary_column");
    m_base_table_boundary_wkt = getDefaultedOption<std::string>("base_table_boundary_wkt");


}

Writer::~Writer()
{
    try
    {
        m_connection->Commit();

    }
    catch (...)
    {
        // destructors shouldn't throw
    }

    return;
}



void Writer::initialize()
{
    pdal::Writer::initialize();
    m_gdal_debug = boost::shared_ptr<pdal::gdal::Debug>(new pdal::gdal::Debug(isDebug(), log()));

}

Options Writer::getDefaultOptions()
{
    Options options;

    Option is3d("is3d",  false,"Should we use 3D objects for SDO_PC PC_EXTENT, \
                                       BLK_EXTENT, and indexing");

    Option solid("solid",false,"Define the point cloud's PC_EXTENT geometry \
                                       gtype as (1,1007,3) instead of the normal \
                                       (1,1003,3), and use gtype 3008/2008 vs \
                                       3003/2003 for BLK_EXTENT geometry values.");

    Option overwrite("overwrite",false,"Wipe the block table and recreate it before loading data");
    Option verbose("verbose",false,"Wipe the block table and recreate it before loading data");
    Option srid("srid", 0, "The Oracle numerical SRID value to use \
                                             for PC_EXTENT, BLK_EXTENT, and indexing");
    Option capacity("capacity",
                    0,
                    "The block capacity or maximum number of \
                                     points a block can contain");
    Option stream_output_precision("stream_output_precision",
                                   8,
                                   "The number of digits past the decimal place for \
                                                    outputting floats/doubles to streams. This is used \
                                                    for creating the SDO_PC object and adding the \
                                                    index entry to the USER_SDO_GEOM_METADATA for the \
                                                    block table");

    Option cloud_id("cloud_id",
                    -1,
                    "The point cloud id that links the point cloud \
                                    object to the entries in the block table.");

    Option connection("connection",
                      "",
                      "Oracle connection string to connect to database");

    Option block_table_name("block_table_name",
                            "output",
                            "The table in which block data for the created SDO_PC will be placed");

    Option block_table_partition_column("block_table_partition_column",
                                        "",
                                        "The column name for which 'block_table_partition_value' \
                                                     will be placed in the 'block_table_name'");
    Option block_table_partition_value("block_table_partition_value",
                                       0,
                                       "Integer value to use to assing partition \
                                                        IDs in the block table. Used in conjunction \
                                                        with 'block_table_partition_column'");
    Option base_table_name("base_table_name",
                           "hobu",
                           "The name of the table which will contain the SDO_PC object");

    Option cloud_column_name("cloud_column_name",
                             "CLOUD",
                             "The column name in 'base_table_name' that will hold the SDO_PC object");

    Option base_table_aux_columns("base_table_aux_columns",
                                  "",
                                  "Quoted, comma-separated list of columns to \
                                                add to the SQL that gets executed as part of \
                                                the point cloud insertion into the \
                                                'base_table_name' table");
    Option base_table_aux_values("base_table_aux_values",
                                 "",
                                 "Quoted, comma-separated values that correspond \
                                              to 'base_table_aux_columns', entries that will \
                                              get inserted as part of the creation of the \
                                              SDO_PC entry in the 'base_table_name' table");

    Option base_table_boundary_column("base_table_boundary_column",
                                      "",
                                      "The SDO_GEOMETRY column in 'base_table_name' in which \
                                                   to insert the WKT in 'base_table_boundary_wkt' representing \
                                                   a boundary for the SDO_PC object. Note this is not \
                                                   the same as the 'base_table_bounds', which is just \
                                                   a bounding box that is placed on the SDO_PC object itself.");
    Option base_table_boundary_wkt("base_table_boundary_wkt",
                                   "",
                                   "WKT, in the form of a string or a file location, to insert \
                                                into the SDO_GEOMTRY column defined by 'base_table_boundary_column'");

    Option pre_block_sql("pre_block_sql",
                         "",
                         "SQL, in the form of a string or file location, that is executed \
                                      after the SDO_PC object has been created but before the block \
                                      data in 'block_table_name' are inserted into the database");

    Option pre_sql("pre_sql",
                   "",
                   "SQL, in the form of a string or file location, that is executed \
                                before the SDO_PC object is created.");

    Option post_block_sql("post_block_sql",
                          "",
                          "SQL, in the form of a string or file location, that is executed \
                                       after the block data in 'block_table_name' have been inserted");

    Option base_table_bounds("base_table_bounds",
                             Bounds<double>(),
                             "A bounding box, given in the Oracle SRID specified in 'srid' \
                                                    to set on the PC_EXTENT object of the SDO_PC. If none is specified, \
                                                    the cumulated bounds of all of the block data are used.");

    Option pc_id("pc_id", -1, "Point Cloud id");
    
    Option pack("pack_ignored_fields", true, "Pack ignored dimensions out of the data buffer that is written");

    options.add(is3d);
    options.add(solid);
    options.add(overwrite);
    options.add(verbose);
    options.add(srid);
    options.add(capacity);
    options.add(stream_output_precision);
    options.add(cloud_id);
    options.add(connection);
    options.add(block_table_name);
    options.add(block_table_partition_column);
    options.add(block_table_partition_value);
    options.add(base_table_name);
    options.add(cloud_column_name);
    options.add(base_table_aux_columns);
    options.add(base_table_aux_values);
    options.add(base_table_boundary_column);
    options.add(base_table_boundary_wkt);
    options.add(pre_block_sql);
    options.add(pre_sql);
    options.add(post_block_sql);
    options.add(base_table_bounds);
    options.add(pc_id);
    options.add(pack);
    return options;
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

    std::string block_table_name = getOptions().getValueOrThrow<std::string>("block_table_name");
    std::string base_table_name = getOptions().getValueOrThrow<std::string>("base_table_name");
    std::string cloud_column_name = getOptions().getValueOrThrow<std::string>("cloud_column_name");

    oss << "DELETE FROM " << block_table_name;
    try
    {
        run(oss);
    }
    catch (pdal_error const&)
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
    cloud_column_name = boost::to_upper_copy(cloud_column_name);
    base_table_name = boost::to_upper_copy(base_table_name);

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

    block_table_name = boost::to_upper_copy(block_table_name);
    oss << "DELETE FROM USER_SDO_GEOM_METADATA WHERE TABLE_NAME='" << block_table_name << "'" ;
    run(oss);
}

bool Writer::is3d() const
{
    return getDefaultedOption<bool>("is3d");
}
bool Writer::isSolid() const
{
    return getDefaultedOption<bool>("solid");
}

boost::int32_t Writer::getPCID() const
{
    return m_pc_id;
}

void Writer::CreateBlockIndex()
{
    std::ostringstream oss;
    std::string block_table_name = getOptions().getValueOrThrow<std::string>("block_table_name");

    std::ostringstream index_name;
    index_name << block_table_name << "_cloud_idx";
    std::string name;
    name = index_name.str().substr(0,29);

    oss << "CREATE INDEX "<< name << " on "
        << block_table_name << "(blk_extent) INDEXTYPE IS MDSYS.SPATIAL_INDEX";

    if (m_is3d)
    {
        oss <<" PARAMETERS('sdo_indx_dims=3')";
    }

    run(oss);
    oss.str("");

    index_name.str("");
    index_name <<  block_table_name <<"_objectid_idx";
    name = index_name.str().substr(0,29);
    oss << "ALTER TABLE "<< block_table_name <<  " ADD CONSTRAINT "<< name <<  
        "  PRIMARY KEY (OBJ_ID, BLK_ID) ENABLE VALIDATE";
    // oss << "CREATE INDEX " << name <<" on "
    //     << block_table_name << "(OBJ_ID,BLK_ID) COMPRESS 2" ;
    run(oss);
    oss.str("");


}

void Writer::CreateSDOEntry()
{

    boost::uint32_t precision = getDefaultedOption<boost::uint32_t>("stream_output_precision");


    std::ostringstream oss;

    std::ostringstream oss_geom;

    oss.setf(std::ios_base::fixed, std::ios_base::floatfield);
    oss.precision(precision);

    std::ostringstream s_srid;


    if (m_srid == 0)
    {
        s_srid << "NULL";
    }
    else
    {
        s_srid << m_srid;
    }

    double tolerance = 0.05;


    pdal::Bounds<double> e = m_bounds;

    if (IsGeographic(m_srid))
    {
        // FIXME: This should be overrideable
        e.setMinimum(0,-180.0);
        e.setMaximum(0,180.0);
        e.setMinimum(1,-90.0);
        e.setMaximum(1,90.0);
        e.setMinimum(2,0.0);
        e.setMaximum(2,20000.0);

        tolerance = 0.0005;
    }


    oss <<  "INSERT INTO user_sdo_geom_metadata VALUES ('" << m_block_table_name <<
        "','blk_extent', MDSYS.SDO_DIM_ARRAY(";

    oss << "MDSYS.SDO_DIM_ELEMENT('X', " << e.getMinimum(0) << "," << e.getMaximum(0) <<"," << tolerance << "),"
        "MDSYS.SDO_DIM_ELEMENT('Y', " << e.getMinimum(1) << "," << e.getMaximum(1) <<"," << tolerance << ")";

    if (m_is3d)
    {
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
    std::string block_table_name = getOptions().getValueOrThrow<std::string>("block_table_name");


    char szTable[OWNAME]= "";
    oss << "select table_name from user_tables";

    log()->get(logDEBUG) << "checking for " << block_table_name << " existence ... " ;

    Statement statement = Statement(m_connection->CreateStatement(oss.str().c_str()));

    // Because of OCIGDALErrorHandler, this is going to throw if there is a
    // problem.  When it does, the statement should go out of scope and
    // be destroyed without leaking.
    statement->Define(szTable);
    statement->Execute();

    log()->get(logDEBUG) << "checking ... " << szTable ;

    bool bDidRead(true);

    while (bDidRead)
    {
        log()->get(logDEBUG) << ", " << szTable;
        if (boost::iequals(szTable, block_table_name))
        {
            log()->get(logDEBUG) << " -- '" << block_table_name << "' found." <<std::endl;
            return true;
        }
        bDidRead = statement->Fetch();
    }

    log()->get(logDEBUG) << " -- '" << block_table_name << "' not found." << std::endl;

    return false;

}

void Writer::CreateBlockTable()
{
    std::ostringstream oss;

    oss << "CREATE TABLE " << m_block_table_name << " AS SELECT * FROM MDSYS.SDO_PC_BLK_TABLE";

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

    statement->Bind(p_srid.get());
    statement->Define(kind.get());

    try
    {
        statement->Execute();
    }
    catch (pdal_error const& e)
    {
        std::ostringstream oss;
        oss << "Failed to fetch geographicness of srid " << srid << std::endl << e.what() << std::endl;
        throw std::runtime_error(oss.str());
    }

    if (boost::iequals(kind.get(), "GEOGRAPHIC2D"))
    {
        return true;
    }
    if (boost::iequals(kind.get(), "GEOGRAPHIC3D"))
    {
        return true;
    }

    return false;

}

std::string Writer::LoadSQLData(std::string const& filename)
{
    if (!FileUtils::fileExists(filename))
    {
        std::ostringstream oss;
        oss << filename << " does not exist";
        throw pdal_error(oss.str());
    }

    std::istream::pos_type size;
    std::istream* input = FileUtils::openFile(filename, true);


    if (input->good())
    {
        std::string output;
        std::string line;
        while (input->good())
        {
            getline(*input, line);
            if (output.size())
            {
                output = output + "\n" + line;
            }
            else
            {
                output = line;
            }
        }

        return output;
    }
    else
    {
        FileUtils::closeFile(input);
        return std::string("");
    }

}

void Writer::RunFileSQL(std::string const& filename)
{
    std::ostringstream oss;
    std::string sql = getOptions().getValueOrDefault<std::string>(filename, "");

    if (!sql.size()) return;

    if (!FileUtils::fileExists(sql))
    {
        oss << sql;
    }
    else
    {
        oss << LoadSQLData(sql);  // Our "sql" is really the filename in the ptree
    }

    if (isDebug())
        std::cout << "running "<< filename << " ..." <<std::endl;

    run(oss);

}

long Writer::GetGType()
{
    bool bUse3d = is3d();
    bool bUseSolidGeometry = getDefaultedOption<bool>("solid");
    long gtype = 0;
    if (bUse3d)
    {
        if (bUseSolidGeometry == true)
        {
            gtype = 3008;

        }
        else
        {
            gtype = 3003;
        }
    }
    else
    {
        if (bUseSolidGeometry == true)
        {
            gtype = 2008;
        }
        else
        {
            gtype = 2003;
        }
    }

    return gtype;
}


std::string Writer::CreatePCElemInfo()
{
    bool bUse3d = is3d();
    bool bUseSolidGeometry = getDefaultedOption<bool>("solid");

    std::ostringstream s_eleminfo;
    if (bUse3d)
    {
        if (bUseSolidGeometry == true)
        {
            // s_gtype << "3008";
            s_eleminfo << "(1,1007,3)";

        }
        else
        {
            // s_gtype << "3003";
            s_eleminfo  << "(1,1003,3)";

        }
    }
    else
    {
        if (bUseSolidGeometry == true)
        {
            // s_gtype << "2008";
            s_eleminfo << "(1,1007,3)";

        }
        else
        {
            // s_gtype << "2003";
            s_eleminfo  << "(1,1003,3)";

        }
    }

    return s_eleminfo.str();

}

void Writer::CreatePCEntry(Schema const& buffer_schema)
{


    boost::uint32_t precision = getDefaultedOption<boost::uint32_t>("stream_output_precision");
    boost::uint64_t capacity = getDefaultedOption<boost::uint64_t>("capacity");
    
    if (capacity == 0)
    {
        capacity = getPrevStage().getNumPoints();
        if (capacity == 0 )
            throw pdal_error("blk_capacity for drivers.oci.writer cannot be 0!");
    }
    


    std::ostringstream oss;


    oss.setf(std::ios_base::fixed, std::ios_base::floatfield);
    oss.precision(precision);

    std::ostringstream columns;
    std::ostringstream values;

    if (!m_base_table_aux_columns.empty())
    {
        columns << m_cloud_column_name << "," << m_base_table_aux_columns;

        values << "pc," << m_base_table_aux_values;
    }
    else
    {
        columns << m_cloud_column_name;
        values << "pc";
    }

    int nPCPos = 1;
    int nSchemaPos = 1;
    nSchemaPos++;

    int nPos = nSchemaPos; // Bind column position

    if (!m_base_table_boundary_column.empty())
    {
        columns << "," << m_base_table_boundary_column;
        nPos++;
        values <<", SDO_GEOMETRY(:"<<nPos;
        nPos++;
        values <<", :"<<nPos<<")";
    }



    std::ostringstream s_srid;
    std::ostringstream s_geom;
    std::ostringstream s_schema;


    // IsGeographic(srid);

    if (m_srid == 0)
    {
        s_srid << "NULL";
    }
    else
    {
        s_srid << m_srid;
    }

    s_schema << "xmltype(:"<<nSchemaPos<<")";



    std::string eleminfo = CreatePCElemInfo();

    pdal::Bounds<double> base_table_bounds = getDefaultedOption<pdal::Bounds<double> >("base_table_bounds");

    if (base_table_bounds.empty())
    {
        if (IsGeographic(m_srid))
        {
            base_table_bounds.setMinimum(0, -179.99);
            base_table_bounds.setMinimum(1, -89.99);
            base_table_bounds.setMinimum(2, 0.0);
            base_table_bounds.setMaximum(0, 179.99);
            base_table_bounds.setMaximum(1, 89.99);
            base_table_bounds.setMaximum(2, 20000.0);
        }
        else
        {
            base_table_bounds.setMinimum(0, 0.0);
            base_table_bounds.setMinimum(1, 0.0);
            base_table_bounds.setMinimum(2, 0.0);
            base_table_bounds.setMaximum(0, 100.0);
            base_table_bounds.setMaximum(1, 100.0);
            base_table_bounds.setMaximum(2, 20000.0);
        }
    }
    s_geom << "           mdsys.sdo_geometry("<< m_gtype <<", "<<s_srid.str()<<", null,\n"
           "              mdsys.sdo_elem_info_array"<< eleminfo <<",\n"
           "              mdsys.sdo_ordinate_array(\n";

    s_geom << base_table_bounds.getMinimum(0) << "," << base_table_bounds.getMinimum(1) << ",";

    if (m_is3d)
    {
        s_geom << base_table_bounds.getMinimum(2) << ",";
    }

    s_geom << base_table_bounds.getMaximum(0) << "," << base_table_bounds.getMaximum(1);

    if (m_is3d)
    {
        s_geom << "," << base_table_bounds.getMaximum(2);
    }

    s_geom << "))";

    boost::uint32_t dimensions = 8;

    oss << "declare\n"
        "  pc_id NUMBER := :"<<nPCPos<<";\n"
        "  pc sdo_pc;\n"

        "begin\n"
        "  -- Initialize the Point Cloud object.\n"
        "  pc := sdo_pc_pkg.init( \n"
        "          '"<< m_base_table_name<<"', -- Table that has the SDO_POINT_CLOUD column defined\n"
        "          '"<< m_cloud_column_name<<"',   -- Column name of the SDO_POINT_CLOUD object\n"
        "          '"<< m_block_table_name <<"', -- Table to store blocks of the point cloud\n"
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
        "  insert into " << m_base_table_name << " ( ID, "<< columns.str() <<
        ") values ( pc.pc_id, " << values.str() << ");\n"

        "  "
        "end;\n";

    Statement statement = Statement(m_connection->CreateStatement(oss.str().c_str()));

    statement->Bind(&m_pc_id);


    OCILobLocator* schema_locator ;
    OCILobLocator* boundary_locator ;

    std::string schema_data;

    bool pack = getOptions().getValueOrDefault<bool>("pack_ignored_fields", true);
    if (pack)
    {
        schema::index_by_index const& idx = buffer_schema.getDimensions().get<schema::index>();
        log()->get(logDEBUG3) << "Packing ignored dimension from PointBuffer " << std::endl;

        boost::uint32_t position(0);
        
        pdal::Schema clean_schema;
        schema::index_by_index::size_type i(0);
        for (i = 0; i < idx.size(); ++i)
        {
            if (! idx[i].isIgnored())
            {
                
                Dimension d(idx[i]);
                d.setPosition(position);
                
                // Wipe off parent/child relationships if we're ignoring 
                // same-named dimensions
                d.setParent(boost::uuids::nil_uuid());
                clean_schema.appendDimension(d);
                position++;
            }
        }
        schema_data = pdal::Schema::to_xml(clean_schema);
            
    } else
    {
        schema_data = pdal::Schema::to_xml(buffer_schema);
        
    }

    char* schema = (char*) malloc(schema_data.size() * sizeof(char) + 1);
    strncpy(schema, schema_data.c_str(), schema_data.size());
    schema[schema_data.size()] = '\0';
    statement->WriteCLob(&schema_locator, schema);
    statement->Bind(&schema_locator);

    std::ostringstream wkt_s;

    if (!m_base_table_boundary_column.empty())
    {
        if (!FileUtils::fileExists(m_base_table_boundary_wkt))
        {
            if (!IsValidWKT(m_base_table_boundary_wkt))
            {
                std::ostringstream oss;
                oss << "WKT for base_table_boundary_wkt was not valid and '" << m_base_table_boundary_wkt
                    << "' doesn't exist as a file";
                throw pdal::pdal_error(oss.str());
            }
            wkt_s << m_base_table_boundary_wkt;
        }
        else
        {
            std::string wkt = LoadSQLData(m_base_table_boundary_wkt);
            if (!IsValidWKT(wkt))
            {
                std::ostringstream oss;
                oss << "WKT for base_table_boundary_wkt was from file '" << m_base_table_boundary_wkt
                    << "' is not valid";
                throw pdal::pdal_error(oss.str());
            }
            wkt_s << wkt;
        }
    }

    std::string wkt_string = wkt_s.str();
    char* wkt = (char*) malloc(wkt_string.size() * sizeof(char)+1);
    strncpy(wkt, wkt_string.c_str(), wkt_string.size());
    wkt[wkt_string.size()] = '\0';
    if (!m_base_table_boundary_column.empty())
    {
        statement->WriteCLob(&boundary_locator, wkt);
        statement->Bind(&boundary_locator);
        statement->Bind((int*)&m_srid);

    }

    try
    {
        statement->Execute();
    }
    catch (std::runtime_error const& e)
    {
        std::ostringstream oss;
        oss << "Failed at creating Point Cloud entry into " << m_base_table_name << " table. Does the table exist? "  << e.what();
        throw pdal_error(oss.str());
    }

    free(wkt);

    try
    {
        Option& pc_id = getOptions().getOptionByRef("pc_id");
        pc_id.setValue(m_pc_id);
    }
    catch (pdal::option_not_found&)
    {
        Option pc_id("pc_id", m_pc_id, "Point Cloud Id");
        getOptions().add(pc_id);
    }

}

bool Writer::IsValidWKT(std::string const& input)
{
#ifdef PDAL_HAVE_GDAL

    OGRGeometryH g;

    char* wkt = const_cast<char*>(input.c_str());
    OGRErr e = OGR_G_CreateFromWkt(&wkt, NULL, &g);
    OGR_G_DestroyGeometry(g);
    if (e != 0) return false;

    return true;
#else

    throw pdal_error("GDAL support not available for WKT validation");
#endif
}

void Writer::writeBufferBegin(PointBuffer const& data)
{
    if (m_sdo_pc_is_initialized) return;

    bool bTrace = getOptions().getValueOrDefault<bool>("do_trace", false);
    if (bTrace)
    {
        log()->get(logDEBUG) << "Setting database trace..." << std::endl;
        std::ostringstream oss;
        // oss << "ALTER SESSION SET SQL_TRACE=TRUE";
        oss << "BEGIN " << std::endl;
        oss << "DBMS_SESSION.set_sql_trace(sql_trace => TRUE);" << std::endl;
        oss << "DBMS_SESSION.SESSION_TRACE_ENABLE(waits => TRUE, binds => TRUE);" << std::endl;
        oss << "END;" << std::endl; 
        run(oss);
        oss.str("");
        
        // oss << "alter session set events ‘10046 trace name context forever, level 12’" << std::endl;
        // run(oss);
        // oss.str("");
        
        oss << "SELECT VALUE FROM V$DIAG_INFO WHERE NAME = 'Default Trace File'";
        Statement statement = Statement(m_connection->CreateStatement(oss.str().c_str()));
        int trace_table_name_length = 1024;
        char* trace_table_name = (char*) malloc(sizeof(char*) * trace_table_name_length);
        trace_table_name[trace_table_name_length-1] = '\0'; //added trailing null to fix ORA-01480        
        statement->Define(trace_table_name, trace_table_name_length);
        statement->Execute();
        // statement->Fetch();
        log()->get(logDEBUG) << "Trace location name:  " << trace_table_name << std::endl;
        

    }
    CreatePCEntry(data.getSchema());
    m_trigger_name = ShutOff_SDO_PC_Trigger();
    
    m_sdo_pc_is_initialized = true;

    return;

}

void Writer::writeBegin(boost::uint64_t)
{

    m_bHaveOutputTable = BlockTableExists();

    if (getDefaultedOption<bool>("overwrite"))
    {
        if (m_bHaveOutputTable)
        {
            WipeBlockTable();
        }
    }

    RunFileSQL("pre_sql");
    if (!m_bHaveOutputTable )
    {
        CreateBlockTable();
    }

    if (getOptions().getValueOrDefault<bool>("create_index", true))
        m_doCreateIndex = true;
    
    // 
    // CreatePCEntry();
    // m_trigger_name = ShutOff_SDO_PC_Trigger();
    return;
}


void Writer::writeEnd(boost::uint64_t)
{
    m_connection->Commit();

    if (m_doCreateIndex && !m_bHaveOutputTable)
    {
        CreateBlockIndex();
    }
    

    // Update extent of SDO_PC entry
    UpdatePCExtent();

    if (getOptions().getValueOrDefault<bool>("reenable_cloud_trigger", false))
        TurnOn_SDO_PC_Trigger(m_trigger_name);

    m_connection->Commit();

    RunFileSQL("post_block_sql");
    return;
}


void Writer::SetElements(Statement statement,
                         OCIArray* elem_info)
{


    statement->AddElement(elem_info, 1);
    if (m_issolid == true)
    {
        //"(1,1007,3)";
        statement->AddElement(elem_info, 1007);
    }
    else
    {
        //"(1,1003,3)";
        statement->AddElement(elem_info, 1003);
    }

    statement->AddElement(elem_info, 3);

}

void Writer::SetOrdinates(Statement statement,
                          OCIArray* ordinates,
                          pdal::Bounds<double> const& extent)
{

    statement->AddElement(ordinates, extent.getMinimum(0));
    statement->AddElement(ordinates, extent.getMinimum(1));
    if (is3d())
        statement->AddElement(ordinates, extent.getMinimum(2));

    statement->AddElement(ordinates, extent.getMaximum(0));
    statement->AddElement(ordinates, extent.getMaximum(1));
    if (is3d())
        statement->AddElement(ordinates, extent.getMaximum(2));


}

pdal::Bounds<double> Writer::CalculateBounds(PointBuffer const& buffer)
{
    pdal::Schema const& schema = buffer.getSchema();

    pdal::Bounds<double> output;

    boost::optional<Dimension const&> dimX = schema.getDimension("X");
    boost::optional<Dimension const&> dimY = schema.getDimension("Y");
    boost::optional<Dimension const&> dimZ = schema.getDimension("Z");


    bool first = true;
    Vector<double> v(0.0, 0.0, 0.0);    
    for (boost::uint32_t pointIndex=0; pointIndex<buffer.getNumPoints(); pointIndex++)
    {
        const boost::int32_t xi = buffer.getField<boost::int32_t>(*dimX, pointIndex);
        const boost::int32_t yi = buffer.getField<boost::int32_t>(*dimY, pointIndex);
        const boost::int32_t zi = buffer.getField<boost::int32_t>(*dimZ, pointIndex);

        const double xd = dimX->applyScaling(xi);
        const double yd = dimY->applyScaling(yi);
        const double zd = dimZ->applyScaling(zi);

        v[0] = xd;
        v[1] = yd;
        v[2] = zd;
        if (first)
        {
            output = pdal::Bounds<double>(xd, yd, zd, xd, yd, zd);
            if (m_pcExtent.empty())
                m_pcExtent = output;
            first = false;
        }
        output.grow(v);
    }

    m_pcExtent.grow(output);
    return output;

}

void Writer::PackPointData(PointBuffer const& buffer,
                           boost::uint8_t** point_data,
                           boost::uint32_t& point_data_len,
                           boost::uint32_t& schema_byte_size)

{
    // Creates a new buffer that has the ignored dimensions removed from 
    // it.
    
    schema::index_by_index const& idx = buffer.getSchema().getDimensions().get<schema::index>();

    schema_byte_size = 0;
    schema::index_by_index::size_type i(0);
    for (i = 0; i < idx.size(); ++i)
    {
        if (! idx[i].isIgnored())
            schema_byte_size = schema_byte_size+idx[i].getByteSize();
    }
    
    log()->get(logDEBUG) << "Packed schema byte size " << schema_byte_size;

    point_data_len = buffer.getNumPoints() * schema_byte_size;
    *point_data = new boost::uint8_t[point_data_len];
    
    boost::uint8_t* current_position = *point_data;
    
    for (boost::uint32_t i = 0; i < buffer.getNumPoints(); ++i)
    {
        boost::uint8_t* data = buffer.getData(i);
        for (boost::uint32_t d = 0; d < idx.size(); ++d)
        {
            if (! idx[d].isIgnored())
            {
                memcpy(current_position, data, idx[d].getByteSize());
                current_position = current_position+idx[d].getByteSize();
            }
            data = data + idx[d].getByteSize();
                
        }
    }

    // Create a vector of two element vectors that states how long of a byte
    // run to copy for the point based on whether or not the ignored/used 
    // flag of the dimension switches. This is to a) eliminate panning through 
    // the dimension multi_index repeatedly per point, and to b) eliminate 
    // tons of small memcpy in exchange for larger ones.
    // std::vector< std::vector< boost::uint32_t> > runs;
    //  // bool switched = idx[0].isIgnored(); // start at with the first dimension
    //  for (boost::uint32_t d = 0; d < idx.size(); ++d)
    //  {
    //      std::vector<boost::uint32_t> t;
    //      if ( idx[d].isIgnored())
    //      { 
    //          t.push_back(0);
    //          t.push_back(idx[d].getByteSize());
    //      } 
    //      else
    //      {
    //          t.push_back(1);
    //          t.push_back(idx[d].getByteSize());
    //      }
    //      
    //      runs.push_back(t);
    //  }
    //  
    //  for (boost::uint32_t i = 0; i < buffer.getNumPoints(); ++i)
    //  {
    //      boost::uint8_t* data = buffer.getData(i);
    //      for (std::vector< std::vector<boost::uint32_t> >::size_type d=0; d < runs.size(); d++)
    //      // for (boost::uint32_t d = 0; d < idx.size(); ++d)
    //      {
    //          boost::uint32_t isUsed = runs[d][0];
    //          boost::uint32_t byteSize = runs[d][1];
    //          if (isUsed != 0)
    //          {
    //              memcpy(current_position, data, byteSize);
    //              current_position = current_position+byteSize;
    //          }
    //          data = data + byteSize;
    //              
    //      }
    //  }
    
}

bool Writer::WriteBlock(PointBuffer const& buffer)
{
    bool bUsePartition = m_block_table_partition_column.size() != 0;

    // Pluck the block id out of the first point in the buffer
    pdal::Schema const& schema = buffer.getSchema();
    Dimension const& blockDim = schema.getDimension("BlockID");

    boost::int32_t block_id  = buffer.getField<boost::int32_t>(blockDim, 0);

    std::ostringstream oss;
    std::ostringstream partition;

    if (bUsePartition)
    {
        partition << "," << m_block_table_partition_column;
    }

    oss << "INSERT INTO "<< m_block_table_name <<
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
    OCILobLocator** locator =(OCILobLocator**) VSIMalloc(sizeof(OCILobLocator*) * 1);
    
    Statement statement = Statement(m_connection->CreateStatement(oss.str().c_str()));



    // :1
    statement->Bind(&m_pc_id);

    // :2
    long long_block_id =  static_cast<long>(block_id);
    statement->Bind(&(long_block_id));

    // :3
    long long_num_points = static_cast<long>(buffer.getNumPoints());
    statement->Bind(&long_num_points);

    // :4
    // statement->Define(locator, 1);


    // std::vector<liblas::uint8_t> data;
    // bool gotdata = GetResultData(result, reader, data, 3);
    // if (! gotdata) throw std::runtime_error("unable to fetch point data byte array");
    // boost::uint8_t* point_data = buffer.getData(0);
    boost::uint8_t* point_data;
    boost::uint32_t point_data_length;
    boost::uint32_t schema_byte_size;
    
    bool pack = getOptions().getValueOrDefault<bool>("pack_ignored_fields", true);
    if (pack)
        PackPointData(buffer, &point_data, point_data_length, schema_byte_size);
    else
    {
        point_data = buffer.getData(0);
        point_data_length = buffer.getSchema().getByteSize() * buffer.getNumPoints();
    }

    // statement->Bind((char*)point_data,(long)(buffer.getSchema().getByteSize()*buffer.getNumPoints()));
    statement->Bind((char*)point_data,(long)(point_data_length));
    // statement->EnableBuffering(locator);
    
    // :5
    long long_gtype = static_cast<long>(m_gtype);
    statement->Bind(&long_gtype);

    // :6
    long* p_srid  = 0;


    if (m_srid != 0)
    {
        p_srid = (long*) malloc(1 * sizeof(long));
        p_srid[0] = m_srid;
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
    pdal::Bounds<double> bounds = buffer.calculateBounds(true);
    SetOrdinates(statement, sdo_ordinates, bounds);
    statement->Bind(&sdo_ordinates, m_connection->GetOrdinateType());

    // :9
    if (bUsePartition)
    {
        long long_partition_id = static_cast<long>(m_block_table_partition_value);
        statement->Bind(&long_partition_id);
    }
    

    try
    {
        statement->Execute();
    }
    catch (std::runtime_error const& e)
    {
        std::ostringstream oss;
        oss << "Failed to insert block # into '" << m_block_table_name << "' table. Does the table exist? "  << std::endl << e.what() << std::endl;
        throw std::runtime_error(oss.str());
    }

    oss.str("");


    // OWStatement::Free(locator, 1);


    if (p_srid != 0) free(p_srid);


    m_connection->DestroyType(&sdo_elem_info);
    m_connection->DestroyType(&sdo_ordinates);




    return true;
}

std::string Writer::ShutOff_SDO_PC_Trigger()
{
    std::string base_table_name = boost::to_upper_copy(getOptions().getValueOrThrow<std::string>("base_table_name"));
    std::string cloud_column_name = boost::to_upper_copy(getOptions().getValueOrThrow<std::string>("cloud_column_name"));

    // Don't monkey with the trigger unless the user says to.
    if (!getOptions().getValueOrDefault<bool>("disable_cloud_trigger", false))
        return std::string("");

    std::ostringstream oss;

    char szTrigger[OWNAME] = "";
    char szStatus[OWNAME] = "";

    oss << "select trigger_name, status from all_triggers where table_name = '" << base_table_name << "' AND TRIGGER_NAME like upper('%%MDTNPC_%%') ";
    Statement statement = Statement(m_connection->CreateStatement(oss.str().c_str()));

    statement->Define(szTrigger);
    statement->Define(szStatus);

    statement->Execute();

    // Yes, we're assuming there's only one trigger that met these criteria.

    if (!strlen(szStatus))
    {
        // No rows returned, no trigger exists
        return std::string("");
    }


    if (boost::iequals(szStatus, "ENABLED"))
    {
        oss.str("");
        oss << "ALTER TRIGGER " << szTrigger << " DISABLE ";

        statement = Statement(m_connection->CreateStatement(oss.str().c_str()));
        statement->Execute();

        return std::string(szTrigger);
    }
    else
    {
        return std::string("");
    }



}

void Writer::TurnOn_SDO_PC_Trigger(std::string trigger_name)
{

    if (!trigger_name.size()) return;

    std::ostringstream oss;

    std::string base_table_name = boost::to_upper_copy(getOptions().getValueOrThrow<std::string>("base_table_name"));

    oss << "ALTER TRIGGER " << trigger_name << " ENABLE ";

    Statement statement = Statement(m_connection->CreateStatement(oss.str().c_str()));
    statement->Execute();


}

void Writer::UpdatePCExtent()
{

    std::string block_table_name = boost::to_upper_copy(getOptions().getValueOrThrow<std::string>("block_table_name"));
    std::string base_table_name = boost::to_upper_copy(getOptions().getValueOrThrow<std::string>("base_table_name"));
    std::string cloud_column_name = boost::to_upper_copy(getOptions().getValueOrThrow<std::string>("cloud_column_name"));

    boost::uint32_t srid = getOptions().getValueOrThrow<boost::uint32_t>("srid");

    pdal::Bounds<double> base_table_bounds = getDefaultedOption<pdal::Bounds<double> >("base_table_bounds");
    if (base_table_bounds.empty()) base_table_bounds = m_pcExtent;



    std::string eleminfo = CreatePCElemInfo();


    std::ostringstream s_geom;
    boost::uint32_t precision = getDefaultedOption<boost::uint32_t>("stream_output_precision");
    s_geom.setf(std::ios_base::fixed, std::ios_base::floatfield);
    s_geom.precision(precision);

    s_geom << "           mdsys.sdo_geometry("<< m_gtype <<", " << srid << ", null,\n"
           "              mdsys.sdo_elem_info_array"<< eleminfo <<",\n"
           "              mdsys.sdo_ordinate_array(\n";

    s_geom << base_table_bounds.getMinimum(0) << "," << base_table_bounds.getMinimum(1) << ",";

    if (m_is3d)
    {
        s_geom << base_table_bounds.getMinimum(2) << ",";
    }

    s_geom << base_table_bounds.getMaximum(0) << "," << base_table_bounds.getMaximum(1);

    if (m_is3d)
    {
        s_geom << "," << base_table_bounds.getMaximum(2);
    }

    s_geom << "))";


    std::ostringstream oss;

    oss << "UPDATE "<< base_table_name <<
        " A SET A." << cloud_column_name <<".PC_EXTENT = " << s_geom.str() <<
        " WHERE A.ID = " << getPCID();

    Statement statement = Statement(m_connection->CreateStatement(oss.str().c_str()));
    try
    {
        statement->Execute();
    }
    catch (std::runtime_error const& e)
    {
        std::ostringstream oss;
        oss << "Failed to update cloud extent in '" << base_table_name << "' table with id " << getPCID() << ". Does the table exist? "  << std::endl << e.what() << std::endl;
        throw std::runtime_error(oss.str());
    }
    m_connection->Commit();

}

boost::uint32_t Writer::writeBuffer(const PointBuffer& buffer)
{
    boost::uint32_t numPoints = buffer.getNumPoints();

    WriteBlock(buffer);

    return numPoints;
}


boost::property_tree::ptree Writer::toPTree() const
{
    boost::property_tree::ptree tree = pdal::Writer::toPTree();

    // add stuff here specific to this stage type

    return tree;
}


}
}
} // namespace pdal::driver::oci
