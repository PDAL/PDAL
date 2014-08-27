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

#include <boost/test/unit_test.hpp>
#include <boost/cstdint.hpp>
#include <boost/property_tree/ptree.hpp>

#include <pdal/drivers/oci/OciReader.hpp>
#include <pdal/drivers/oci/Writer.hpp>
#include <pdal/drivers/oci/common.hpp>

#include <pdal/drivers/las/Reader.hpp>
#include <pdal/drivers/las/Writer.hpp>

#include <pdal/filters/Cache.hpp>
#include <pdal/filters/Chipper.hpp>
#include <pdal/filters/InPlaceReprojection.hpp>

#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/filters/InPlaceReprojection.hpp>

#include <pdal/drivers/las/Writer.hpp>
#include <pdal/StageIterator.hpp>

#include "Support.hpp"
#include "TestConfig.hpp"

using namespace pdal;

// using namespace pdal::drivers::oci;

static unsigned chunk_size = 25;

bool ShouldRunTest()
{
    return TestConfig::g_oracle_connection.size() > 0;
}

Options getOCIOptions()
{
    Options options;


    Option capacity("capacity", chunk_size,"capacity");
    options.add(capacity);

    Option overwrite("overwrite", false,"overwrite");
    options.add(overwrite);

    Option connection("connection",std::string(TestConfig::g_oracle_connection), "connection");
    options.add(connection);

    Option debug("debug", "true", "debug");
    options.add(debug);

    Option verbose("verbose", 7, "verbose");
    options.add(verbose);

    Option block_table_name("block_table_name", "PDAL_TEST_BLOCKS", "block_table_name");
    options.add(block_table_name);

    Option base_table_name("base_table_name", "PDAL_TEST_BASE" , "");
    options.add(base_table_name);

    Option cloud_column_name("cloud_column_name", "CLOUD" , "");
    options.add(cloud_column_name);

    Option srid("srid",26910,"");
    options.add(srid);
    // 
    // Option out_srs("out_srs", "EPSG:4326","");
    // options.add(out_srs);
    // 
    // Option scale_x("scale_x", 0.0000001, "");
    // options.add(scale_x);
    


    // Option offset_x("offset_x", 493994.875f, "");
    // options.add(offset_x);
    // 
    // Option offset_y("offset_y", 4877429.62f, "");
    // options.add(offset_y);





    Option disable_cloud_trigger("disable_cloud_trigger", true, "");
    options.add(disable_cloud_trigger);

    Option max_cache_blocks("max_cache_blocks", 1, "");
    options.add(max_cache_blocks);

    // Option cache_block_size("cache_block_size", capacity.getValue<boost::uint32_t>(), "");
    // options.add(cache_block_size);

    Option filename("filename", Support::datapath("autzen-utm.las"), "");
    options.add(filename);

    Option query("query", "SELECT CLOUD FROM PDAL_TEST_BASE where ID=1", "");
    options.add(query);
    
    Option pack("pack_ignored_fields", true, "");
    options.add(pack);
    
    Option xml_schema_dump("xml_schema_dump", "pcs-oracle-xml-schema-dump.xml", "");
    options.add(xml_schema_dump);

    
    return options;
}

struct OracleTestFixture
{
    OracleTestFixture() :
        m_options(getOCIOptions())
        , m_connection(pdal::drivers::oci::Connection())
    {
        if (!ShouldRunTest()) return;
        pdal::drivers::oci::Connection connection = connect();

        std::string base_table_name = m_options.getValueOrThrow<std::string>("base_table_name");
        std::string create_pc_table("CREATE TABLE " + base_table_name +" (id number, CLOUD SDO_PC, DESCRIPTION VARCHAR2(20), HEADER BLOB, BOUNDARY SDO_GEOMETRY)");
        run(connection, create_pc_table);

        std::string block_table_name = m_options.getValueOrThrow<std::string>("block_table_name");
        std::string create_block_table = "CREATE TABLE " + block_table_name + " AS SELECT * FROM MDSYS.SDO_PC_BLK_TABLE";
        run(connection, create_block_table);

    }

    pdal::drivers::oci::Connection connect()
    {
        if (!m_connection.get())
        {
           std::string connSpec =
                m_options.getValueOrThrow<std::string>("connection");
            m_connection = pdal::drivers::oci::connect(connSpec);
        }
        return m_connection;

    }

    void run(pdal::drivers::oci::Connection connection, std::string sql)
    {
        pdal::drivers::oci::Statement statement = pdal::drivers::oci::Statement(connection->CreateStatement(sql.c_str()));
        statement->Execute();
    }

    pdal::Options m_options;
    pdal::drivers::oci::Connection m_connection;

    ~OracleTestFixture()
    {
        if (!ShouldRunTest())
            return;
        pdal::drivers::oci::Connection connection = connect();

        std::string base_table_name =
            m_options.getValueOrThrow<std::string>("base_table_name");
        std::string block_table_name =
            m_options.getValueOrThrow<std::string>("block_table_name");

        std::string drop_base_table = "DROP TABLE " + base_table_name;
        std::string drop_block_table = "DROP TABLE " + block_table_name;
        run(connection, drop_base_table);
        run(connection, drop_block_table);

        std::string cleanup_metadata = "DELETE FROM USER_SDO_GEOM_METADATA WHERE TABLE_NAME ='" + block_table_name + "'";
        run(connection, cleanup_metadata);
    }
};

BOOST_FIXTURE_TEST_SUITE(OCITest, OracleTestFixture)


bool WriteUnprojectedData()
{
    Options options;

    // Note that this corresponds to the autzen-utm-chipped-25.las 
    // file in test/data/
    Option capacity("capacity", 25,"capacity");
    options.add(capacity);

    Option connection("connection",std::string(TestConfig::g_oracle_connection), "connection");
    options.add(connection);

    Option debug("debug", "true", "debug");
    options.add(debug);

    Option verbose("verbose", 7, "verbose");
    options.add(verbose);

    Option block_table_name("block_table_name", "PDAL_TEST_BLOCKS", "block_table_name");
    options.add(block_table_name);

    Option base_table_name("base_table_name", "PDAL_TEST_BASE" , "");
    options.add(base_table_name);

    Option cloud_column_name("cloud_column_name", "CLOUD" , "");
    options.add(cloud_column_name);

    Option srid("srid", 26910,"");
    options.add(srid);

    Option disable_cloud_trigger("disable_cloud_trigger", true, "");
    options.add(disable_cloud_trigger);

    Option store_dimensional_orientation("store_dimensional_orientation", true, "");
    options.add(store_dimensional_orientation);
    
    Option filename("filename", Support::datapath("autzen-utm.las"), "");
    options.add(filename);
    
    PointContext ctx;
    
    pdal::drivers::las::Reader reader(options);
    // pdal::filters::Chipper chipper(options);
    // chipper.setInput(&reader);
    pdal::drivers::oci::Writer writer(options);
    writer.setInput(&reader);

    writer.prepare(ctx);
    writer.execute(ctx);

    //ABELL - This test doesn't test anything anymore.  Perhaps it should.
    
    return true;
}

void checkUnProjectedPoints(PointBuffer const& data)
{
    int X[] = { 49405730, 49413382, 49402110, 49419289, 49418622, 49403411 };
    int Y[] = { 487743335, 487743982, 487743983, 487744219, 487744254, 487745019 };
    int Z[] = { 13063, 13044, 13046, 13050, 13049, 13066 };
    int I[] = { 134, 75, 153, 93, 67, 167 };
    int R[] = { 142, 152, 146, 104, 113, 163 };
    int G[] = { 102, 108, 104, 93, 97, 118 };
    int B[] = { 137, 134, 140, 120, 123, 150 };
        
    for (unsigned i = 0; i < 6; ++i)
    {
        int32_t x = data.getFieldAs<int32_t>(Dimension::Id::X, i);
        int32_t y = data.getFieldAs<int32_t>(Dimension::Id::Y, i);
        int32_t z = data.getFieldAs<int32_t>(Dimension::Id::Z, i);
        uint16_t intensity =
            data.getFieldAs<uint16_t>(Dimension::Id::Intensity, i);
        uint16_t red = data.getFieldAs<uint16_t>(Dimension::Id::Red, i);
        uint16_t green = data.getFieldAs<uint16_t>(Dimension::Id::Green, i);
        uint16_t blue = data.getFieldAs<uint16_t>(Dimension::Id::Blue, i);

        BOOST_CHECK_EQUAL(x, X[i]);
        BOOST_CHECK_EQUAL(y, Y[i]);
        BOOST_CHECK_EQUAL(z, Z[i]);
        BOOST_CHECK_EQUAL(intensity, I[i]);
        BOOST_CHECK_EQUAL(red, R[i]);
        BOOST_CHECK_EQUAL(green, G[i]);
        BOOST_CHECK_EQUAL(blue, B[i]);
    }
}


void compareAgainstSourceBuffer(PointBuffer const& candidate,
    std::string filename)
{
    pdal::Options options;
    pdal::Option f("filename", filename);
    options.add(f);
    
    PointContext tc;
    pdal::drivers::las::Reader reader(options);

    reader.prepare(tc);
    
    BOOST_CHECK_EQUAL(candidate.size(), reader.getNumPoints());
    
    PointBufferSet pbSet = reader.execute(tc);
    BOOST_CHECK_EQUAL(pbSet.size(), 1);
    PointBufferPtr source = *pbSet.begin();

    BOOST_CHECK_EQUAL(source->size(), reader.getNumPoints());
    
    // int X[] = { 49405730, 49413382, 49402110, 494192890, 49418622, 49403411 };
    // int Y[] = { 487743335, 487743982, 487743983, 487744219, 487744254, 487745019 };
    // int Z[] = { 13063, 13044, 13046, 13050, 13049, 13066 };
    // int I[] = { 134, 75, 153, 93, 67, 167 };
    // int R[] = { 142, 152, 146, 104, 113, 163 };
    // int G[] = { 102, 108, 104, 96, 97, 118 };
    // int B[] = { 137, 134, 140, 120, 123, 150 };
    //     
    for (unsigned i = 0; i < 6; ++i)
    {
        int32_t sx = source->getFieldAs<int32_t>(Dimension::Id::X, i);
        int32_t sy = source->getFieldAs<int32_t>(Dimension::Id::Y, i);
        int32_t sz = source->getFieldAs<int32_t>(Dimension::Id::Z, i);
        uint16_t sintensity = source->getFieldAs<uint16_t>(
            Dimension::Id::Intensity, i);
        uint16_t sred = source->getFieldAs<uint16_t>(Dimension::Id::Red, i);
        uint16_t sgreen = source->getFieldAs<uint16_t>(Dimension::Id::Green, i);
        uint16_t sblue = source->getFieldAs<uint16_t>(Dimension::Id::Blue, i);

        int32_t cx = candidate.getFieldAs<int32_t>(Dimension::Id::X, i);
        int32_t cy = candidate.getFieldAs<int32_t>(Dimension::Id::Y, i);
        int32_t cz = candidate.getFieldAs<int32_t>(Dimension::Id::Z, i);
        uint16_t cintensity =
            candidate.getFieldAs<uint16_t>(Dimension::Id::Intensity, i);
        uint16_t cred = candidate.getFieldAs<uint16_t>(Dimension::Id::Red, i);
        uint16_t cgreen = candidate.getFieldAs<uint16_t>(Dimension::Id::Green, i);
        uint16_t cblue = candidate.getFieldAs<uint16_t>(Dimension::Id::Blue, i);

        BOOST_CHECK_EQUAL(sx, cx);
        BOOST_CHECK_EQUAL(sy, cy);
        BOOST_CHECK_EQUAL(sz, cz);
        BOOST_CHECK_EQUAL(sintensity, cintensity);
        BOOST_CHECK_EQUAL(sred, cred);
        BOOST_CHECK_EQUAL(sgreen, cgreen);
        BOOST_CHECK_EQUAL(sblue, cblue);
    }
}


BOOST_AUTO_TEST_CASE(read_unprojected_data)
{
    if (!ShouldRunTest()) return;

    WriteUnprojectedData();

    std::ostringstream oss;

    oss << "SELECT  l.\"OBJ_ID\", l.\"BLK_ID\", l.\"BLK_EXTENT\", l.\"BLK_DOMAIN\","
           "        l.\"PCBLK_MIN_RES\", l.\"PCBLK_MAX_RES\", l.\"NUM_POINTS\","
           "        l.\"NUM_UNSORTED_POINTS\", l.\"PT_SORT_DIM\", l.\"POINTS\", b.cloud "
           "FROM PDAL_TEST_BLOCKS l, PDAL_TEST_BASE b "
        "WHERE b.id=1 and l.obj_id = b.id and l.obj_id in (1) "
        "ORDER BY l.obj_id ";
    Options options = getOCIOptions();
    Option& query = options.getOptionByRef("query");
    query.setValue<std::string>(oss.str());

    Option& debug = options.getOptionByRef("debug");
    debug.setValue<std::string>( "true");

    Option& verbose = options.getOptionByRef("verbose");
    verbose.setValue<std::string>( "7");
    
    pdal::drivers::oci::OciReader reader(options);
    PointContext ctx;
    reader.prepare(ctx);
    
    pdal::PointBuffer data(ctx);
    pdal::StageSequentialIterator* iter = reader.createSequentialIterator();
    
    uint32_t numRead = iter->read(data);
    
    BOOST_CHECK_EQUAL(numRead, 1065u);
    BOOST_CHECK_EQUAL(data.size(), 1065u);
    
    // checkUnProjectedPoints(data);
    
    // compareAgainstSourceBuffer(data,  Support::datapath("autzen-utm-chipped-25.las"));
    compareAgainstSourceBuffer(data,  Support::datapath("autzen-utm.las"));
    
    delete iter;

}


// 
// 
// BOOST_AUTO_TEST_CASE(read_view_reproj)
// {
//     if (!ShouldRunTest()) return;
// 
//     WriteDefaultData();
// 
//     std::ostringstream oss;
// 
//     oss << "SELECT  l.\"OBJ_ID\", l.\"BLK_ID\", l.\"BLK_EXTENT\", l.\"BLK_DOMAIN\","
//            "        l.\"PCBLK_MIN_RES\", l.\"PCBLK_MAX_RES\", l.\"NUM_POINTS\","
//            "        l.\"NUM_UNSORTED_POINTS\", l.\"PT_SORT_DIM\", l.\"POINTS\", b.cloud "
//            "FROM PDAL_TEST_BLOCKS l, PDAL_TEST_BASE b "
//         "WHERE b.id=1 and l.obj_id = b.id "
//         "ORDER BY l.obj_id ";
//     Options options = getOCIOptions();
//     Option& query = options.getOptionByRef("query");
//     query.setValue<std::string>(oss.str());
// 
//     Option& out_srs = options.getOptionByRef("out_srs"); 
//     out_srs.setValue<std::string>("EPSG:26910");
// 
//     Option& x_scale = options.getOptionByRef("scale_x");
//     x_scale.setValue<float>(0.01);
// 
//     Option& y_scale = options.getOptionByRef("scale_y");
//     y_scale.setValue<float>(0.01);
// 
//     // Option& offset_x = options.getOptionByRef("offset_x");
//     // offset_x.setValue<float>( -123.0749695f);
//     // offset_x.setValue<float>( 0.0f);
// 
//     // Option& offset_y = options.getOptionByRef("offset_y");
//     // offset_y.setValue<float>( 44.0500086f);
//     // offset_y.setValue<float>( 0.0f);
// 
// 
//     pdal::Option x_dim("x_dim", std::string("drivers.oci.reader.X"), "Dimension name to use for 'X' data");
//     pdal::Option y_dim("y_dim", std::string("drivers.oci.reader.Y"), "Dimension name to use for 'Y' data");
//     pdal::Option z_dim("z_dim", std::string("drivers.oci.reader.Z"), "Dimension name to use for 'Z' data");    
// 
//     options.add(x_dim);
//     options.add(y_dim);
//     options.add(z_dim);
// 
//     Option& debug = options.getOptionByRef("debug");
//     debug.setValue<std::string>( "true");
// 
//     Option& verbose = options.getOptionByRef("verbose");
//     verbose.setValue<std::string>( "7");
//     
//     pdal::drivers::oci::Reader reader_reader(options);
//     // pdal::filters::InPlaceReprojection reproj(reader_reader, options);
//     reader_reader.prepare();
// 
//     pdal::PointBuffer data(reader_reader.getSchema(), chunk_size+30);
//     pdal::StageSequentialIterator* iter = reader_reader.createSequentialIterator(data);
// 
//     boost::uint32_t numRead(0);
// 
//     numRead = iter->read(data);
// 
//     BOOST_CHECK_EQUAL(numRead, chunk_size+30u);
//     BOOST_CHECK_EQUAL(data.getNumPoints(), chunk_size+30u);
//     
//     checkPoints(data);
// 
//     delete iter;
// 
// }

//
// BOOST_AUTO_TEST_CASE(read_all)
// {
//     if (!ShouldRunTest()) return;
// 
//     WriteDefaultData();
//     WriteDefaultData();
// 
//     std::ostringstream oss;
// 
//     oss << "SELECT CLOUD FROM PDAL_TEST_BASE";
//     Options options = getOCIOptions();
//     Option& query = options.getOptionByRef("query");
//     query.setValue<std::string>(oss.str());
//     pdal::drivers::oci::Reader reader_reader(options);
//     reader_reader.prepare();
// 
//     pdal::PointBuffer data(reader_reader.getSchema(), 2500);
//     boost::scoped_ptr<pdal::StageSequentialIterator> iter(reader_reader.createSequentialIterator(data));
// 
// 
//     boost::uint32_t numRead = iter->read(data);
// 
//     BOOST_CHECK_EQUAL(numRead, 2130u);
// 
//     pdal::Schema const& schema = data.getSchema();
//     pdal::Dimension const& dimX = schema.getDimension("X");
//     pdal::Dimension const& dimY = schema.getDimension("Y");
//     pdal::Dimension const& dimZ = schema.getDimension("Z");
//     pdal::Dimension const& dimIntensity = schema.getDimension("Intensity");
//     pdal::Dimension const& dimRed = schema.getDimension("Red");
// 
//     boost::int32_t x = data.getFieldAs<boost::int32_t>(dimX, 0);
//     boost::int32_t y = data.getFieldAs<boost::int32_t>(dimY, 0);
//     boost::int32_t z = data.getFieldAs<boost::int32_t>(dimZ, 0);
//     boost::uint16_t intensity = data.getFieldAs<boost::uint16_t>(dimIntensity, 6);
//     boost::uint16_t red = data.getFieldAs<boost::uint16_t>(dimRed, 6);
// 
//     BOOST_CHECK_EQUAL(x, -1250367506);
//     BOOST_CHECK_EQUAL(y, 492519663);
//     BOOST_CHECK_EQUAL(z, 12931);
//     BOOST_CHECK_EQUAL(intensity, 67);
//     BOOST_CHECK_EQUAL(red, 113);
// 
// }
// 

// BOOST_AUTO_TEST_CASE(read_view)
// {
//     if (!ShouldRunTest()) return;
// 
//     WriteDefaultData();
// 
//     std::ostringstream oss;
// 
//     oss << "SELECT  l.\"OBJ_ID\", l.\"BLK_ID\", l.\"BLK_EXTENT\", l.\"BLK_DOMAIN\","
//            "        l.\"PCBLK_MIN_RES\", l.\"PCBLK_MAX_RES\", l.\"NUM_POINTS\","
//            "        l.\"NUM_UNSORTED_POINTS\", l.\"PT_SORT_DIM\", l.\"POINTS\", b.cloud "
//            "FROM PDAL_TEST_BLOCKS l, PDAL_TEST_BASE b "
//         "WHERE b.id=1 and l.obj_id = b.id "
//         "ORDER BY l.obj_id ";
//     Options options = getOCIOptions();
//     Option& query = options.getOptionByRef("query");
//     query.setValue<std::string>(oss.str());
//     pdal::drivers::oci::Reader reader_reader(options);
//     reader_reader.prepare();
// 
//     pdal::PointBuffer data(reader_reader.getSchema(), chunk_size+30);
//     boost::scoped_ptr<pdal::StageSequentialIterator> iter(reader_reader.createSequentialIterator(data));
// 
// 
//     boost::uint32_t numTotal(0);
//     boost::uint32_t numRead(0);
// 
//     pdal::PointBuffer data3(reader_reader.getSchema(), 100);
//     numRead = iter->read(data3);
//     BOOST_CHECK_EQUAL(numRead, 100u);
//     numTotal = numRead + numTotal;
// 
//     while (numRead !=0)
//     {
//         numRead = iter->read(data3);
//         numTotal = numRead + numTotal;
// 
//     }
// 
//     BOOST_CHECK_EQUAL(numRead, 0u);
//     BOOST_CHECK_EQUAL(numTotal, 1065u);
// 
// }

BOOST_AUTO_TEST_SUITE_END()
