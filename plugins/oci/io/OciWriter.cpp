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

#include <pdal/pdal_features.hpp>
#include <pdal/PDALUtils.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/compression/LazPerfCompression.hpp>
#include <pdal/util/ProgramArgs.hpp>
#include <pdal/private/gdal/GDALUtils.hpp>

#include "OciWriter.hpp"

#include <fstream>

#include <ogr_api.h>

namespace pdal
{

static PluginInfo const s_info
{
    "writers.oci",
    "Write data using SDO_PC objects to Oracle.",
    "http://pdal.io/stages/writers.oci.html"
};

CREATE_SHARED_STAGE(OciWriter, s_info)

std::string OciWriter::getName() const { return s_info.name; }

OciWriter::OciWriter()
    : m_createIndex(false)
    , m_bDidCreateBlockTable(false)
    , m_pc_id(0)
    , m_srid(0)
    , m_3d(false)
    , m_solid(false)
    , m_sdo_pc_is_initialized(false)
    , m_chunkCount(16)
    , m_capacity(0)
    , m_streamChunks(false)
    , m_orientation(Orientation::PointMajor)
{}


void OciWriter::runCommand(std::ostringstream const& command)
{
    Statement statement(m_connection->CreateStatement(command.str().c_str()));

    // Because of OCIGDALErrorHandler, this is going to throw if there is a
    // problem.  When it does, the statement should go out of scope and
    // be destroyed without leaking.
    statement->Execute();
}


void OciWriter::wipeBlockTable()
{
    std::ostringstream oss;

    oss << "DELETE FROM " << m_blockTableName;
    try
    {
        runCommand(oss);
    }
    catch (pdal_error const&)
    {
        // if we failed, let's try dropping the spatial index for the
        // block_table_name
        oss.str("");
        if (isDebug())
            std::cout << "Dropping index " << m_blockTableName
                      << "_cloud_idx for block_table_name"
                      << std::endl;
        oss << "DROP INDEX " << m_blockTableName << "_cloud_idx" ;
        runCommand(oss);
        oss.str("");

        oss << "DELETE FROM " << m_blockTableName;
        runCommand(oss);
    }

    oss.str("");
    oss << "declare\n"
        "begin \n"
        "  mdsys.sdo_pc_pkg.drop_dependencies('"
        << m_baseTableName <<
        "', '"
        << m_cloudColumnName <<
        "'); end;";
    runCommand(oss);

    oss.str("");
    oss << "DROP TABLE " << m_blockTableName;
    runCommand(oss);

    // Oracle upper cases the table name when inserting it in the
    // USER_SDO_GEOM_METADATA.  We'll use std::transform to do it.
    // See http://forums.devx.com/showthread.php?t=83058 for the
    // technique

    oss.str("");
    oss << "DELETE FROM USER_SDO_GEOM_METADATA WHERE TABLE_NAME='" <<
        m_blockTableName << "'" ;
    runCommand(oss);
}


void OciWriter::createBlockIndex()
{
    std::ostringstream oss;

    std::ostringstream index_name;
    index_name << m_blockTableName << "_cloud_idx";
    std::string name = index_name.str().substr(0, 29);

    oss << "CREATE INDEX "<< name << " on "
        << m_blockTableName << "(blk_extent) INDEXTYPE IS MDSYS.SPATIAL_INDEX";

    if (m_3d)
        oss << " PARAMETERS('sdo_indx_dims=3')";

    runCommand(oss);

    index_name.str("");
    index_name << m_blockTableName << "_objectid_idx";
    name = index_name.str().substr(0, 29);

    oss.str("");
    oss << "ALTER TABLE "<< m_blockTableName <<  " ADD CONSTRAINT "<< name <<
        "  PRIMARY KEY (OBJ_ID, BLK_ID) ENABLE VALIDATE";
    runCommand(oss);
}


void OciWriter::createSDOEntry()
{
    std::ostringstream oss;
    oss.setf(std::ios_base::fixed, std::ios_base::floatfield);
    oss.precision(m_precision);

    std::ostringstream s_srid;
    if (m_srid == 0)
        s_srid << "NULL";
    else
        s_srid << m_srid;

    BOX3D e = m_bounds;
    if (isGeographic(m_srid))
    {
        //This should be overrideable
        e.minx = -180.0;
        e.maxx = 180.0;
        e.miny = -90.0;
        e.maxy = 90.0;
        e.minz = 0.0;
        e.maxz = 20000.0;

        m_tolerance = 0.0005;
    }

    oss <<  "INSERT INTO user_sdo_geom_metadata VALUES ('" <<
        m_blockTableName << "','blk_extent', MDSYS.SDO_DIM_ARRAY(";
    oss << "MDSYS.SDO_DIM_ELEMENT('X', " << e.minx << "," <<
        e.maxx <<"," << m_tolerance << "),"
        "MDSYS.SDO_DIM_ELEMENT('Y', " << e.miny << "," <<
        e.maxy <<"," << m_tolerance << ")";

    if (m_3d)
    {
        oss << ",";
        oss <<"MDSYS.SDO_DIM_ELEMENT('Z', "<< e.minz << "," <<
            e.maxz << "," << m_tolerance << ")";
    }
    oss << ")," << s_srid.str() << ")";

    runCommand(oss);
}


bool OciWriter::blockTableExists()
{
    std::ostringstream oss;

    char szTable[OWNAME] = "";
    oss << "select table_name from user_tables";

    log()->get(LogLevel::Debug) << "checking for " << m_blockTableName <<
        " existence ... " << std::endl;

    Statement statement(m_connection->CreateStatement(oss.str().c_str()));

    // Because of OCIGDALErrorHandler, this is going to throw if there is a
    // problem.  When it does, the statement should go out of scope and
    // be destroyed without leaking.
    statement->Define(szTable);
    statement->Execute();

    log()->get(LogLevel::Debug) << "checking ... " << szTable << std::endl;
    do
    {
        if (Utils::iequals(szTable, m_blockTableName))
            return true;
    } while (statement->Fetch());

    log()->get(LogLevel::Debug) << " -- '" << m_blockTableName <<
        "' not found." << std::endl;
    return false;
}


void OciWriter::createBlockTable()
{
    std::ostringstream oss;
    oss << "CREATE TABLE " << m_blockTableName <<
        " AS SELECT * FROM MDSYS.SDO_PC_BLK_TABLE";

    runCommand(oss);
    m_connection->Commit();
    m_bDidCreateBlockTable = true;
}


bool OciWriter::isGeographic(int32_t srid)
{
    std::unique_ptr<char> kind(new char[OWNAME]);
    std::string stmt("SELECT COORD_REF_SYS_KIND from "
        "MDSYS.SDO_COORD_REF_SYSTEM WHERE SRID = :1");

    Statement statement(m_connection->CreateStatement(stmt.c_str()));
    long l_srid = srid;
    statement->Bind(&l_srid);
    statement->Define(kind.get());

    try
    {
        statement->Execute();
    }
    catch (pdal_error const& e)
    {
        throwError("Failed to fetch geographicness of srid " +
            Utils::toString(srid) + ": " + e.what());
    }

    std::string k = Utils::toupper(kind.get());
    return ((k == "GEOGRAPHIC2D") || (k == "GEOGRAPHIC3D"));
}


std::string OciWriter::loadSQLData(std::string const& filename)
{
    if (!Utils::fileExists(filename))
        throwError("File '" + filename + "' does not exist");

    std::istream::pos_type size;
    std::istream* input = Utils::openFile(filename, true);
    if (!input->good())
    {
        Utils::closeFile(input);
        return std::string();
    }

    std::string output;
    std::string line;
    while (input->good())
    {
        getline(*input, line);
        if (output.size())
            output += "\n" + line;
        else
            output = line;
    }

    Utils::closeFile(input);

    return output;
}


void OciWriter::runFileSQL(std::string const& sql)
{
    if (sql.empty())
        return;

    std::ostringstream oss;
    if (!Utils::fileExists(sql))
        oss << sql;
    else
        // Our "sql" is really the filename in the ptree
        oss << loadSQLData(sql);
    runCommand(oss);
}


// Get the geometry type
long OciWriter::getGType()
{
    if (m_3d)
        return (m_solid ? 3008 : 3003);
    return (m_solid ? 2008 : 2003);
}


std::string OciWriter::createPCElemInfo()
{
    if (m_3d)
        return m_solid ? "(1,1007,3)" : "(1,1003,3)";
    return m_solid ? "(1,1007,3)" : "(1,1003,3)";
}


void OciWriter::createPCEntry()
{
    std::ostringstream oss;

    oss.setf(std::ios_base::fixed, std::ios_base::floatfield);
    oss.precision(m_precision);

    std::ostringstream columns;
    std::ostringstream values;

    columns << m_cloudColumnName;
    values << "pc";
    if (!m_baseTableAuxColumns.empty())
    {
        columns << "," << m_baseTableAuxColumns;
        values << "," << m_baseTableAuxValues;
    }

    int nPCPos = 1;
    int nSchemaPos = 1;
    nSchemaPos++;

    int nPos = nSchemaPos; // Bind column position

    if (!m_baseTableBoundaryColumn.empty())
    {
        columns << "," << m_baseTableBoundaryColumn;
        nPos++;
        values << ", SDO_GEOMETRY(:" << nPos;
        nPos++;
        values << ", :"<< nPos << ")";
    }

    std::ostringstream s_srid;
    std::ostringstream s_geom;
    std::ostringstream s_schema;

    if (m_srid == 0)
        s_srid << "NULL";
    else
        s_srid << m_srid;
    s_schema << "xmltype(:" << nSchemaPos << ")";

    std::string eleminfo = createPCElemInfo();
    BOX3D bounds = m_baseTableBounds;
    if (bounds.empty())
    {
        if (isGeographic(m_srid))
        {
            bounds.minx = -179.99;
            bounds.miny = -89.99;
            bounds.minz = 0.0;
            bounds.maxx = 179.99;
            bounds.maxy = 89.99;
            bounds.maxz = 20000.0;
        }
        else
        {
            bounds.minx = 0.0;
            bounds.miny = 0.0;
            bounds.minz = 0.0;
            bounds.maxx = 100.0;
            bounds.maxy = 100.0;
            bounds.maxz = 20000.0;
        }
    }
    s_geom << "           mdsys.sdo_geometry(" << m_gtype << ", " <<
        s_srid.str() << ", null,\n"
        "              mdsys.sdo_elem_info_array"<< eleminfo <<",\n"
        "              mdsys.sdo_ordinate_array(\n";
    s_geom << bounds.minx << "," << bounds.miny << ",";
    if (m_3d)
        s_geom << bounds.minz << ",";
    s_geom << bounds.maxy<< "," << bounds.maxy;
    if (m_3d)
        s_geom << "," << bounds.maxy;
    s_geom << "))";

    MetadataNode node;
    if (m_compression)
    {
        MetadataNode r = node.add("root");
        r.add("compression", "lazperf");
        r.add("version", "1.0");
    }
    XMLSchema schema(dbDimTypes(), node, m_orientation);
    std::string schemaData = schema.xml();

    oss << "declare\n"
        "  pc_id NUMBER := :" << nPCPos << ";\n"
        "  pc sdo_pc;\n"

        "begin\n"
        "  -- Initialize the Point Cloud object.\n"
        "  pc := sdo_pc_pkg.init( \n"
        "          '" << m_baseTableName <<
        "', -- Table that has the SDO_POINT_CLOUD column defined\n"
        "          '" << m_cloudColumnName <<
        "',   -- Column name of the SDO_POINT_CLOUD object\n"
        "          '" << m_blockTableName <<
        "', -- Table to store blocks of the point cloud\n"
        "           'blk_capacity="<< m_capacity <<"', -- max # of points "
                    "per block\n"
        << s_geom.str() <<
        ",  -- Extent\n"
        "     0.5, -- Tolerance for point cloud\n"
        "           " << dbDimTypes().size() <<
        ", -- Total number of dimensions\n"
        "           NULL,"
        "            NULL,"
        "            " << s_schema.str() << ");\n"
        "  :" << nPCPos << " := pc.pc_id;\n"

        "  -- Insert the Point Cloud object  into the \"base\" table.\n"
        "  insert into " << m_baseTableName << " ( ID, "<< columns.str() <<
        ") values ( pc.pc_id, " << values.str() << ");\n"

        "  "
        "end;\n";

    Statement statement(m_connection->CreateStatement(oss.str().c_str()));
    statement->Bind(&m_pc_id);

    OCILobLocator *schema_locator;
    OCILobLocator *boundary_locator;

    statement->WriteCLob(&schema_locator, const_cast<char *>(schemaData.c_str()));
    statement->BindClob(&schema_locator);

    std::ostringstream wkt_s;
    if (!m_baseTableBoundaryColumn.empty())
    {
        if (!Utils::fileExists(m_baseTableBoundaryWkt))
        {
            if (!isValidWKT(m_baseTableBoundaryWkt))
                throwError("WKT for base_table_boundary_wkt was not valid "
                    "and '" + m_baseTableBoundaryWkt + "' doesn't exist as "
                    "a file");
            wkt_s << m_baseTableBoundaryWkt;
        }
        else
        {
            std::string wkt = loadSQLData(m_baseTableBoundaryWkt);
            if (!isValidWKT(wkt))
                throwError("WKT for base_table_boundary_wkt was from file '" +
                    m_baseTableBoundaryWkt + "' is not valid");
            wkt_s << wkt;
        }
    }

    if (!m_baseTableBoundaryColumn.empty())
    {
        statement->WriteCLob(&boundary_locator,
            const_cast<char *>(wkt_s.str().c_str()));
        statement->BindClob(&boundary_locator);
        statement->Bind((int*)&m_srid);
    }

    try
    {
        statement->Execute();
    }
    catch (std::runtime_error const& e)
    {
        throwError("Failed at creating Point Cloud entry into " +
            m_baseTableName + " table: " + e.what() + ".");
    }
}


bool OciWriter::isValidWKT(std::string const& input)
{
    OGRGeometryH g;

    char* wkt = const_cast<char*>(input.c_str());
    OGRErr e = OGR_G_CreateFromWkt(&wkt, NULL, &g);
    OGR_G_DestroyGeometry(g);
    return (e == 0);
}


void OciWriter::addArgs(ProgramArgs& args)
{
    DbWriter::doAddArgs(args);
    args.add("base_table_name", "Base table name",
        m_baseTableName).setPositional();
    args.add("block_table_name", "Block table name",
        m_blockTableName).setPositional();
    args.add("cloud_column_name", "Cloud column name",
        m_cloudColumnName).setPositional();
    args.add("capacity", "Points per block", m_capacity).setPositional();

    args.add("is3d", "Should we use 3D objects (include the z dimension) "
        "for SDO_PC PC_EXTENT, BLK_EXTENT, and indexing", m_3d);
    args.add("stream_output_precision", "The number of digits past the "
        "decimal place for outputting floats/doubles to streams. This "
        "is used for creating the SDO_PC object and adding the index "
        "entry to the USER_SDO_GEOM_METADATA for the block table",
        m_precision, 8U);
    args.add("create_index", "Whether an index should be created",
        m_createIndex, true);
    args.add("overwrite", "Wipe the block table and recreate it before "
        "loading data", m_overwrite, false);
    args.add("reenable_cloud_trigger", "Reenable cloud trigger",
        m_reenableCloudTrigger);
    args.add("disable_cloud_trigger", "Disable cloud trigger",
        m_disableCloudTrigger);
    args.add("do_trace", "Trace processing", m_trace);
    args.add("solid", "Define the point cloud's PC_EXTENT geometry "
        "gtype as (1,1007,3) instead of the normal (1,1003,3), and use "
        "gtype 3008/2008 vs  3003/2003 for BLK_EXTENT geometry values.",
        m_solid);
    args.add("srid", "SRID", m_srid);
    args.add("base_table_bounds", "A bounding box, given in the Oracle "
        "SRID specified in 'srid' to set on the PC_EXTENT object of the "
        "SDO_PC. If none is specified, the cumulated bounds of all of "
        "the block data are used.", m_baseTableBounds);
    args.add("block_table_partition_column", "The column name for which "
        "'block_table_partition_value' will be placed in the "
        "'block_table_name'", m_blockTablePartitionColumn);
    args.add("block_table_partition_value", "Integer value to use to "
        "assign partition IDs in the block table. Used in conjunction "
        "with 'block_table_partition_column'", m_blockTablePartitionValue);
    args.add("base_table_aux_columns", "Quoted, comma-separated list of "
        "columns to add to the SQL that gets executed as part of the "
        "point cloud insertion into the 'base_table_name' table",
        m_baseTableAuxColumns);
    args.add("base_table_aux_values", "Quoted, comma-separated values that "
        "correspond to 'base_table_aux_columns', entries that will get "
        "inserted as part of the creation of the SDO_PC entry in the "
        "'base_table_name' table", m_baseTableAuxValues);
    args.add("base_table_boundary_column", "The SDO_GEOMETRY column in "
        "'base_table_name' in which to insert the WKT in "
        "'base_table_boundary_wkt' representing a boundary for the SDO_PC "
        "object. Note this is not the same as the 'base_table_bounds', "
        "which is just a bounding box that is placed on the SDO_PC object "
        "itself.", m_baseTableBoundaryColumn);
    args.add("base_table_boundary_wkt", "WKT, in the form of a string or "
        "a file location, to insert into the SDO_GEOMTRY column defined "
        "by 'base_table_boundary_column'", m_baseTableBoundaryWkt);
    args.add("blob_chunk_count", "Blob chunk count", m_chunkCount, 16U);
    args.add("stream_chunks", "Stream chunks", m_streamChunks);
    args.add("store_dimensional_orientation", "Point major(default) or "
        "dimension major storage", m_dimInterleaved);
    args.add("connection", "Database connection string", m_connSpec);
    args.add("compression", "Set to turn compression on", m_compression);
    args.add("pre_sql", "SQL to run before query", m_preSql);
    args.add("post_block_sql", "SQL to run when stage is done", m_postBlockSql);
    args.add("tolerance", "Oracle geometry tolerance ", m_tolerance, 0.05);
}


void OciWriter::initialize()
{
    m_baseTableName = Utils::toupper(m_baseTableName);
    m_blockTableName = Utils::toupper(m_blockTableName);
    m_cloudColumnName = Utils::toupper(m_cloudColumnName);
    m_orientation = m_dimInterleaved ? Orientation::DimensionMajor :
        Orientation::PointMajor;

    if (m_compression && (m_orientation == Orientation::DimensionMajor))
        throwError("LAZperf compression not supported for dimension-major "
            "point storage.");

    gdal::registerDrivers();
    try
    {
        m_connection = connect(m_connSpec);
    }
    catch (const connection_failed& err)
    {
        throwError(err.what());
    }

    m_gtype = getGType();
}


void OciWriter::write(const PointViewPtr view)
{
    // While we'd like a separate offset for each tile, the schema is stored
    // for the entire point cloud.
    if (m_lastBlockId == 0)
        setAutoXForm(view);
    writeInit();
    writeTile(view);
}


void OciWriter::writeInit()
{
    if (m_sdo_pc_is_initialized)
        return;

    if (m_trace)
    {
        log()->get(LogLevel::Debug) << "Setting database trace..." << std::endl;
        std::ostringstream oss;
        oss << "BEGIN " << std::endl;
        oss << "DBMS_SESSION.set_sql_trace(sql_trace => TRUE);" << std::endl;
        oss << "DBMS_SESSION.SESSION_TRACE_ENABLE(waits => TRUE, "
            "binds => TRUE);" << std::endl;
        oss << "END;" << std::endl;
        runCommand(oss);

        char selectCmd[] = "SELECT VALUE FROM V$DIAG_INFO WHERE NAME = "
            "'Default Trace File'";
        Statement statement(m_connection->CreateStatement(selectCmd));

        char traceTableName[1024] = {0};
        statement->Define(traceTableName, sizeof(traceTableName));
        statement->Execute();
        log()->get(LogLevel::Debug) << "Trace location name:  " <<
            traceTableName << std::endl;

    }
    createPCEntry();
    m_triggerName = shutOff_SDO_PC_Trigger();
    m_sdo_pc_is_initialized = true;
}


void OciWriter::ready(PointTableRef table)
{
    bool haveOutputTable = blockTableExists();
    if (m_overwrite && haveOutputTable)
        wipeBlockTable();
    runFileSQL(m_preSql);
    if (!haveOutputTable)
        createBlockTable();

    m_pcExtent.clear();
    m_lastBlockId = 0;
    DbWriter::doReady(table);
}


void OciWriter::done(PointTableRef table)
{
    if (!m_connection)
        return;

    m_connection->Commit();
    if (m_createIndex && m_bDidCreateBlockTable)
    {
        createSDOEntry();
        createBlockIndex();
    }

    // Update extent of SDO_PC entry
    updatePCExtent();

    if (m_reenableCloudTrigger)
        turnOn_SDO_PC_Trigger(m_triggerName);
    m_connection->Commit();
    runFileSQL(m_postBlockSql);
}


void OciWriter::setElements(Statement statement, OCIArray* elem_info)
{
    statement->AddElement(elem_info, 1);
    if (m_solid == true)
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


void OciWriter::setOrdinates(Statement statement, OCIArray* ordinates,
    const BOX3D& extent)
{

    statement->AddElement(ordinates, extent.minx);
    statement->AddElement(ordinates, extent.miny);
    if (m_3d)
        statement->AddElement(ordinates, extent.minz);

    statement->AddElement(ordinates, extent.maxx);
    statement->AddElement(ordinates, extent.maxy);
    if (m_3d)
        statement->AddElement(ordinates, extent.maxz);
}

void OciWriter::writePointMajor(PointViewPtr view, std::vector<char>& outbuf)
{
    if (m_compression)
    {
        outbuf.resize(0);
#ifdef PDAL_HAVE_LAZPERF
        XMLDimList xmlDims = dbDimTypes();
        DimTypeList dimTypes;
        for (XMLDim& xmlDim : xmlDims)
            dimTypes.push_back(xmlDim.m_dimType);

        auto cb = [&outbuf](char *buf, size_t bufsize)
        {
            outbuf.insert(outbuf.end(), buf, buf + bufsize);
        };

        LazPerfCompressor compressor(cb, dimTypes);

        try
        {
            std::vector<char> ptBuf(packedPointSize());
            for (PointId idx = 0; idx < view->size(); ++idx)
            {
                size_t size = readPoint(*view, idx, ptBuf.data());
                compressor.compress(ptBuf.data(), size);
            }
        }
        catch (const pdal_error& err)
        {
            compressor.done();
            throwError(err.what());
        }
        compressor.done();
#else
        throwError("Can't compress without LAZperf.");
#endif
    }
    else
    {
        size_t totalSize = 0;
        char *pos = outbuf.data();
        for (PointId idx = 0; idx < view->size(); ++idx)
        {
            size_t size = readPoint(*view.get(), idx, pos);
            totalSize += size;
            pos += size;
        }
        outbuf.resize(totalSize);
    }
}


void OciWriter::writeDimMajor(PointViewPtr view, std::vector<char>& outbuf)
{
    size_t interrupt = dbDimTypes().size() * 100;
    size_t totalSize = 0;
    char *pos = outbuf.data();

    XMLDimList xmlDims = dbDimTypes();
    for (auto& xmlDim : xmlDims)
    {
        for (PointId idx = 0; idx < view->size(); ++idx)
        {
            size_t size = readField(*view.get(), pos,
                xmlDim.m_dimType.m_id, idx);
            pos += size;
            totalSize += size;
        }
    }
    outbuf.resize(totalSize);
}


void OciWriter::writeTile(const PointViewPtr view)
{
    bool usePartition = (m_blockTablePartitionColumn.size() != 0);

    std::ostringstream oss;
    oss << "INSERT INTO "<< m_blockTableName <<
        "(OBJ_ID, BLK_ID, NUM_POINTS, POINTS, PCBLK_MIN_RES, BLK_EXTENT, "
        "PCBLK_MAX_RES, NUM_UNSORTED_POINTS, PT_SORT_DIM";
    if (usePartition)
        oss << "," << m_blockTablePartitionColumn;
    oss << ") "
        "VALUES ( :1, :2, :3, :4, 1, mdsys.sdo_geometry(:5, :6, null,:7, :8)"
        ", 1, 0, 1";
    if (usePartition)
        oss << ", :9";
    oss <<")";

    Statement statement(m_connection->CreateStatement(oss.str().c_str()));

    // :1
    statement->Bind(&m_pc_id);
    log()->get(LogLevel::Debug4) << "Block obj_id " << m_pc_id << std::endl;

    // :2
    statement->Bind(&m_lastBlockId);
    m_lastBlockId++;
    log()->get(LogLevel::Debug4) << "Last BlockId " <<
        m_lastBlockId << std::endl;

    // :3
    long long_num_points = static_cast<long>(view->size());
    statement->Bind(&long_num_points);
    log()->get(LogLevel::Debug4) << "Num points " <<
        long_num_points << std::endl;

    // :4

    //NOTE: packed point size is guaranteed to be of sufficient size to hold
    // a point's data, but it may be larger than the actual size of a point
    // if location scaling is being used.
    std::vector<char> outbuf(packedPointSize() * view->size());
    size_t totalSize = 0;
    if (m_orientation == Orientation::DimensionMajor)
        writeDimMajor(view, outbuf);
    else if (m_orientation == Orientation::PointMajor)
        writePointMajor(view, outbuf);


    log()->get(LogLevel::Debug4) << "Blob size " << outbuf.size() << std::endl;
    OCILobLocator* locator;
    if (m_streamChunks)
    {
        statement->WriteBlob(&locator, outbuf.data(), (long)outbuf.size(),
            m_chunkCount);
        statement->BindBlob(&locator);
    }
    else
        statement->Bind(outbuf.data(), (long)outbuf.size());

    // :5
    long long_gtype = static_cast<long>(m_gtype);
    statement->Bind(&long_gtype);
    log()->get(LogLevel::Debug4) << "OCI geometry type " <<
        m_gtype << std::endl;

    // :6
    long srid;

    if (m_srid != 0)
        srid = m_srid;
    statement->Bind(&srid);
    log()->get(LogLevel::Debug4) << "OCI SRID " << srid << std::endl;


    // :7
    OCIArray* sdo_elem_info = 0;
    m_connection->CreateType(&sdo_elem_info, m_connection->GetElemInfoType());
    setElements(statement, sdo_elem_info);
    statement->Bind(&sdo_elem_info, m_connection->GetElemInfoType());

    // :8
    OCIArray* sdo_ordinates = 0;
    m_connection->CreateType(&sdo_ordinates, m_connection->GetOrdinateType());

    BOX3D bounds;
    view->calculateBounds(bounds);
    // Cumulate a total bounds for the file.
    m_pcExtent.grow(bounds);

    setOrdinates(statement, sdo_ordinates, bounds);
    statement->Bind(&sdo_ordinates, m_connection->GetOrdinateType());
    log()->get(LogLevel::Debug4) << "Bounds " << bounds << std::endl;

    // :9
    if (usePartition)
    {
        long long_partition_id = (long)m_blockTablePartitionValue;
        statement->Bind(&long_partition_id);
        log()->get(LogLevel::Debug4) << "Partition ID " << long_partition_id <<
            std::endl;
    }

    try
    {
        statement->Execute();
    }
    catch (std::runtime_error const& e)
    {
        throwError("Failed to insert block # into '" + m_blockTableName +
            "' table: " + e.what());
    }

    if (m_streamChunks)
        // We don't use a locator unless we're stream-writing the data.
        OWStatement::Free(&locator, 1);

    m_connection->DestroyType(&sdo_elem_info);
    m_connection->DestroyType(&sdo_ordinates);
}


std::string OciWriter::shutOff_SDO_PC_Trigger()
{
    // Don't monkey with the trigger unless the user says to.
    if (!m_disableCloudTrigger)
        return std::string("");

    std::ostringstream oss;
    char szTrigger[OWNAME] = "";
    char szStatus[OWNAME] = "";

    oss << "select trigger_name, status from all_triggers where "
        "table_name = '" << m_baseTableName <<
        "' AND TRIGGER_NAME like upper('%%MDTNPC_%%') ";
    Statement statement(m_connection->CreateStatement(oss.str().c_str()));

    statement->Define(szTrigger);
    statement->Define(szStatus);
    statement->Execute();

    // Yes, we're assuming there's only one trigger that met these criteria.

    if (!strlen(szStatus))
    {
        // No rows returned, no trigger exists
        return std::string();
    }

    if (Utils::iequals(szStatus, "ENABLED"))
    {
        oss.str("");
        oss << "ALTER TRIGGER " << szTrigger << " DISABLE ";

        statement = Statement(m_connection->CreateStatement(oss.str().c_str()));
        statement->Execute();
        return std::string(szTrigger);
    }
    return std::string();
}


void OciWriter::turnOn_SDO_PC_Trigger(std::string trigger_name)
{
    if (!trigger_name.size())
        return;

    std::ostringstream oss;
    oss << "ALTER TRIGGER " << trigger_name << " ENABLE ";

    Statement statement(m_connection->CreateStatement(oss.str().c_str()));
    statement->Execute();
}


void OciWriter::updatePCExtent()
{
    BOX3D bounds = m_baseTableBounds;
    if (bounds.empty())
        bounds = m_pcExtent;

    std::ostringstream s_geom;
    s_geom.setf(std::ios_base::fixed, std::ios_base::floatfield);
    s_geom.precision(m_precision);

    s_geom << "           mdsys.sdo_geometry(" << m_gtype << ", " << m_srid <<
        ", null,\n"
        "              mdsys.sdo_elem_info_array" << createPCElemInfo() << ",\n"
        "              mdsys.sdo_ordinate_array(\n";

    s_geom << bounds.minx << "," << bounds.miny << ",";
    if (m_3d)
        s_geom << bounds.minz << ",";

    s_geom << bounds.maxx << "," << bounds.maxy;
    if (m_3d)
        s_geom << "," << bounds.maxz;
    s_geom << "))";

    std::ostringstream oss;

    oss << "UPDATE "<< m_baseTableName <<
        " A SET A." << m_cloudColumnName <<".PC_EXTENT = " << s_geom.str() <<
        " WHERE A.ID = " << m_pc_id;

    Statement statement(m_connection->CreateStatement(oss.str().c_str()));
    try
    {
        statement->Execute();
    }
    catch (std::runtime_error const& e)
    {
        throwError("Failed to update cloud extent in '" + m_baseTableName +
            "' table with id " + Utils::toString(m_pc_id) + ": " + e.what());
    }
    m_connection->Commit();
}


} // namespace pdal
