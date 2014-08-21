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
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/concept_check.hpp>

#include <pdal/FileUtils.hpp>
#include <pdal/drivers/sqlite/SQLiteWriter.hpp>
#include <pdal/drivers/sqlite/SQLiteReader.hpp>
#include <pdal/drivers/las/Reader.hpp>
#include <pdal/filters/Cache.hpp>
#include <pdal/filters/Chipper.hpp>
#include <pdal/filters/InPlaceReprojection.hpp>
#include <pdal/filters/Selector.hpp>

#include <pdal/PointBuffer.hpp>

#include "Support.hpp"

using namespace pdal;

static unsigned chunk_size = 15;

Options getSQLITEOptions()
{
    Options options;


    Option capacity("capacity", chunk_size,"capacity");
    options.add(capacity);

    Option overwrite("overwrite", true,"overwrite");
    options.add(overwrite);
    
    std::string temp_filename(Support::temppath("temp-SqliteWriterTest_test_simple_las.sqlite"));
    Option connection("connection",temp_filename, "connection");
    options.add(connection);

    Option debug("debug", true, "debug");
    // options.add(debug);

    Option verbose("verbose", 7, "verbose");
    // options.add(verbose);

    Option block_table_name("block_table_name", "PDAL_TEST_BLOCKS", "block_table_name");
    options.add(block_table_name);

    Option base_table_name("cloud_table_name", "PDAL_TEST_BASE" , "");
    options.add(base_table_name);

    Option is3d("is3d", false,"");
    options.add(is3d);

    Option srid("srid",4326,"");
    options.add(srid);

    Option out_srs("out_srs", "EPSG:4269","");
    options.add(out_srs);

    Option scale_x("scale_x", 0.0000001f, "");
    options.add(scale_x);

    Option scale_y("scale_y", 0.0000001f, "");
    options.add(scale_y);

    Option filename("filename", Support::datapath("1.2-with-color.las"), "");
    options.add(filename);

    Option query("query", "                SELECT b.schema, l.cloud, l.block_id, l.num_points, l.bbox, l.extent, l.points, b.cloud FROM PDAL_TEST_BLOCKS l, PDAL_TEST_BASE b "
                 "WHERE l.cloud = b.cloud and l.cloud in (1) "
                "order by l.cloud", "");
    options.add(query);

    Option a_srs("spatialreference", "EPSG:2926", "");
    options.add(a_srs);
    
    Option pack("pack_ignored_fields", true, "");
    options.add(pack);


    Option cloud_column("cloud_column_name", "CLOUD", "");
    options.add(cloud_column);
        
    // Option xml_schema_dump("xml_schema_dump", "sqlite-xml-schema-dump.xml", "");
    // options.add(xml_schema_dump);

    Option con_type("type", "sqlite", "");
    options.add(con_type);
    
    return options;
}


struct SQLiteTestFixture
{
    SQLiteTestFixture() :
        m_options(getSQLITEOptions())
    {

    }

    ~SQLiteTestFixture()
    {
        std::string temp_filename = m_options.getValueOrThrow<std::string>("connection");
        FileUtils::deleteFile(temp_filename);
    }

    pdal::Options m_options;    
};



BOOST_FIXTURE_TEST_SUITE(SQLiteTest, SQLiteTestFixture)

BOOST_AUTO_TEST_CASE(SqliteTest_test_simple_las)
{
#ifdef PDAL_HAVE_SQLITE
    // remove file from earlier run, if needed
    std::string temp_filename = getSQLITEOptions().getValueOrThrow<std::string>("connection");

    pdal::drivers::las::Reader reader(Support::datapath("1.2-with-color.las"));

    {
        pdal::drivers::las::Reader writer_reader(getSQLITEOptions());
        pdal::drivers::sqlite::SQLiteWriter writer_writer(getSQLITEOptions());
        writer_writer.setInput(&writer_reader);
        
        PointContext ctx;
        writer_writer.prepare(ctx);
        boost::uint64_t numPointsToRead = writer_reader.getNumPoints();

        BOOST_CHECK_EQUAL(numPointsToRead, 1065u);

        writer_writer.execute(ctx);
    }
    
    {
        // Read the data

        pdal::drivers::sqlite::SQLiteReader reader(getSQLITEOptions());
        PointContext ctx;
        reader.prepare(ctx);
        PointBuffer buffer(ctx);

        pdal::StageSequentialIterator* iter =
            reader.createSequentialIterator();
        iter->read(buffer);
        
        boost::uint16_t r = buffer.getFieldAs<boost::uint16_t>(Dimension::Id::Red, 0);
        BOOST_CHECK_EQUAL(r, 68u);
        boost::int32_t x = buffer.getFieldAs<boost::int32_t>(Dimension::Id::X, 0);
        BOOST_CHECK_EQUAL(x, 637012);
        double xd = buffer.getFieldAs<double>(Dimension::Id::X, 0);        
        BOOST_CHECK_CLOSE(xd, 637012.240, 0.001);
    }
    // FileUtils::deleteFile(temp_filename);
#endif
}



BOOST_AUTO_TEST_SUITE_END()
