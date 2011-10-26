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

#include <pdal/drivers/oci/Reader.hpp>
#include <pdal/drivers/oci/Writer.hpp>
#include <pdal/drivers/oci/common.hpp>

#include <pdal/drivers/las/Reader.hpp>

#include <pdal/filters/CacheFilter.hpp>
#include <pdal/filters/Chipper.hpp>
#include <pdal/filters/InPlaceReprojectionFilter.hpp>

#include <pdal/drivers/faux/Reader.hpp>
#include <pdal/drivers/faux/Writer.hpp>

#include <pdal/drivers/las/Writer.hpp>
#include <pdal/StageIterator.hpp>

#include "Support.hpp"
#include "TestConfig.hpp"

using namespace pdal;

// using namespace pdal::drivers::oci;


bool ShouldRunTest()
{
    return TestConfig::g_oracle_connection.size() > 0;
}

Options getOptions()
{
    Options options;
    
    Option capacity("capacity", 333,"capacity");
    options.add(capacity);
    
    Option overwrite("overwrite", false,"overwrite");
    options.add(overwrite);
    
    Option connection("connection",TestConfig::g_oracle_connection, "connection");
    options.add(connection);
    
    Option debug("debug", true, "debug");
    options.add(debug);
    
    Option verbose("verbose", 1, "verbose");
    options.add(verbose);
    
    Option block_table_name("block_table_name", "PDAL_TEST_BLOCKS", "block_table_name");
    options.add(block_table_name);
    
    Option base_table_name("base_table_name", "PDAL_TEST_BASE" , "");
    options.add(base_table_name);
    
    Option cloud_column_name("cloud_column_name", "CLOUD" , "");
    options.add(cloud_column_name);
    
    Option is3d("is3d", false,"");
    options.add(is3d);
    
    Option solid("solid", false,"");
    options.add(solid);
    
    Option srid("srid",4269,"");
    options.add(srid);
    
    Option out_srs("out_srs", "EPSG:4269","");
    options.add(out_srs);

    Option scale_x("scale_x", 0.0000001, "");
    options.add(scale_x);
    
    Option scale_y("scale_y", 0.0000001, "");
    options.add(scale_y);
    
    Option max_cache_blocks("max_cache_blocks", 1, "");
    options.add(max_cache_blocks);
    
    Option cache_block_size("cache_block_size", capacity.getValue<boost::uint32_t>(), "");
    options.add(cache_block_size);
    
    Option filename("filename", Support::datapath("1.2-with-color.las"), "");
    options.add(filename);
    
    Option query("query", "SELECT CLOUD FROM PDAL_TEST_BASE where ID=1", "");
    options.add(query);

    Option a_srs("spatialreference", "EPSG:2926", "");
    options.add(a_srs);
    
    return options;
}

struct OracleTestFixture
{
    OracleTestFixture() :
    m_options(getOptions())
    , m_driver(getOptions())
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
        if (!m_connection.get() ) 
        {
            m_connection = m_driver.connect();
            
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
    pdal::drivers::oci::OracleDriver m_driver;
    
    ~OracleTestFixture() 
    {
        if (!ShouldRunTest()) return;    
        pdal::drivers::oci::Connection connection = connect();

        std::string base_table_name = m_options.getValueOrThrow<std::string>("base_table_name");
        std::string block_table_name = m_options.getValueOrThrow<std::string>("block_table_name");
    
        std::string drop_base_table = "DROP TABLE " + base_table_name;
        std::string drop_block_table = "DROP TABLE " + block_table_name;
        run(connection, drop_base_table);
        run(connection, drop_block_table);
        
        std::string cleanup_metadata = "DELETE FROM USER_SDO_GEOM_METADATA WHERE TABLE_NAME ='" + block_table_name + "'";
        run(connection, cleanup_metadata);
    }
};

BOOST_FIXTURE_TEST_SUITE(OCITest, OracleTestFixture)




BOOST_AUTO_TEST_CASE(initialize)
{
    if (!ShouldRunTest()) return;
    
    pdal::drivers::las::Reader writer_reader(getOptions());
    pdal::filters::CacheFilter writer_cache(writer_reader, getOptions());
    pdal::filters::Chipper writer_chipper(writer_cache, getOptions());
    pdal::filters::InPlaceReprojectionFilter writer_reproj(writer_chipper, getOptions());
    pdal::drivers::oci::Writer writer_writer(writer_reproj, getOptions());
    
    writer_writer.initialize();
    boost::uint64_t numPointsToRead = writer_reader.getNumPoints();
    
    BOOST_CHECK_EQUAL(numPointsToRead, 1065);
    
    writer_writer.write(0);
    
    pdal::drivers::oci::Reader reader_reader(getOptions());
    reader_reader.initialize();
    
    boost::scoped_ptr<pdal::StageSequentialIterator> iter(reader_reader.createSequentialIterator());

    pdal::PointBuffer data(reader_reader.getSchema(), 1000);
 
    boost::uint32_t numRead = iter->read(data);

    BOOST_CHECK_EQUAL(numRead, 799);

    pdal::Schema const& schema = reader_reader.getSchema();
    int offsetX = schema.getDimensionIndex(DimensionId::X_i32);
    BOOST_CHECK_EQUAL(offsetX, 0);
    int offsetY = schema.getDimensionIndex(DimensionId::Y_i32);
    BOOST_CHECK_EQUAL(offsetY, 1);
    int offsetZ = schema.getDimensionIndex(DimensionId::Z_i32);
    BOOST_CHECK_EQUAL(offsetZ, 2);
    
    // data.getSchema().dump();
    boost::int32_t x = data.getField<boost::int32_t>(0, offsetX);
    boost::int32_t y = data.getField<boost::int32_t>(0, offsetY);
    boost::int32_t z = data.getField<boost::int32_t>(0, offsetZ);

    BOOST_CHECK_EQUAL(x, -1250418763);
    BOOST_CHECK_EQUAL(y, 492548402); 
    BOOST_CHECK_EQUAL(z, 13625); // the test case goes m -> ft.
    
}
// 
// 
// BOOST_AUTO_TEST_CASE(test_writer)
// {
//     if (!ShouldRunTest()) return;
// 
//     Options options = GetOptions();
//     pdal::drivers::liblas::LiblasReader reader(Support::datapath("1.2-with-color.las"));
// 
//     boost::uint32_t capacity = GetOptions().GetPTree().get<boost::uint32_t>("capacity");
//     pdal::filters::CacheFilter cache(reader, 1, 1024);
//     pdal::filters::Chipper chipper(cache, capacity);
//     pdal::drivers::oci::Writer writer(chipper, options);
//     
//     BOOST_CHECK_MESSAGE(writer.getConnection().get(), "Unable to connect to Oracle" );
//     
//     writer.write(0);
//     
// }
// 
// BOOST_AUTO_TEST_CASE(test_reader)
// {
//     if (!ShouldRunTest()) return;
// 
//     Options options = GetOptions();
// 
//     pdal::drivers::oci::Reader reader(options);
//     const boost::uint64_t numPoints = reader.getNumPoints();
//     BOOST_CHECK_EQUAL(numPoints, 0);
//     
//     boost::scoped_ptr<SequentialIterator> iter(reader.createSequentialIterator());
//     
//     PointBuffer buffer(reader.getSchema(), 1065);
//     
//     const boost::uint32_t numPointsReadThisChunk = iter->read(buffer);
//     
//     BOOST_CHECK_EQUAL(numPointsReadThisChunk, 1065);
//     // std::ostream* ofs = Utils::createFile("temp.las");
//     // 
//     // pdal::drivers::liblas::LiblasWriter writer(reader, *ofs);
//     // writer.write(0);
//     // 
//     // Utils::closeFile(ofs);
// 
//     
// }




BOOST_AUTO_TEST_SUITE_END()
