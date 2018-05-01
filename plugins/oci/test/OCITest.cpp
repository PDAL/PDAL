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

#include "gtest/gtest.h"

#include <pdal/Filter.hpp>
#include <pdal/StageFactory.hpp>
#include <io/LasReader.hpp>

#include "../io/OciCommon.hpp"
#include "Support.hpp"
#include "TestConfig.hpp"

using namespace pdal;

namespace
{

static std::string connectString(TestConfig::oracleConnection());
static const std::string baseTableName("PDAL_TEST_BASE");
static const std::string blockTableName("PDAL_TEST_BLOCKS");

}

GTEST_API_ int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    for (int i = 0; i < argc; ++i)
    {
        std::string s = argv[i];
        if (Utils::startsWith(s, "--connection"))
        {
            auto pos = s.find_first_of('=');
            if (pos == std::string::npos)
                throw pdal_error("Invalid command line connection string.");
            connectString = s.substr(pos + 1);
            break;
        }
    }
    return RUN_ALL_TESTS();
}


bool ShouldRunTest()
{
    return connectString.size() > 0;
}


Options readerOptions()
{
    Options options;

    options.add("connection", connectString);
    options.add("xml_schema_dump", "pcs-oracle-xml-schema-dump.xml");
    return options;
}

Options writerOptions()
{
    Options options;

    options.add("overwrite", false);
    options.add("connection", connectString);
    options.add("block_table_name", blockTableName);
    options.add("base_table_name", baseTableName);
    options.add("cloud_column_name", "CLOUD");
    options.add("srid", 26910);
    options.add("disable_cloud_trigger", true);
    return options;
}

class SplitFilter : public Filter
{
public:
   SplitFilter(point_count_t viewSize = 100) : m_viewSize(viewSize)
       {}

   virtual std::string getName() const
       { return "split_filter"; }

private:
    point_count_t m_viewSize;

    virtual PointViewSet run(PointViewPtr view)
    {
        PointViewSet out;
        PointViewPtr v = view->makeNew();
        for (PointId i = 0; i < view->size(); ++i)
        {
            if (i && (i % m_viewSize == 0))
            {
                out.insert(v);
                v = v->makeNew();
            }
            v->appendPoint(*view, i);
        }
        out.insert(v);
        return out;
    }
};

class OCITest : public testing::Test
{
protected:
    virtual void SetUp()
    {
        connect();
        cleanup();

        std::string create_pc_table("CREATE TABLE " + baseTableName +
            " (id number, CLOUD SDO_PC, DESCRIPTION VARCHAR2(20), HEADER "
            "BLOB, BOUNDARY SDO_GEOMETRY)");
        run(create_pc_table);

        std::string create_block_table = "CREATE TABLE " + blockTableName +
            " AS SELECT * FROM MDSYS.SDO_PC_BLK_TABLE";
        run(create_block_table);
    }

    void connect()
    {
        if (!m_connection.get())
            m_connection = pdal::connect(connectString);
        if (!m_connection.get() || !m_connection->Succeeded())
        {
            std::ostringstream oss;

            oss << "Couldn't connect via OCI using spec '" <<
                connectString << "'";
            throw pdal_error(oss.str());
        }
    }

    void run(std::string sql)
    {
        Statement statement(m_connection->CreateStatement(sql.c_str()));
        statement->Execute();
    }

    Connection m_connection;

    virtual void TearDown()
    {
    }

    void cleanup()
    {
        std::string drop_base_table = "DROP TABLE " + baseTableName;
        std::string drop_block_table = "DROP TABLE " + blockTableName;
        run(drop_base_table);
        run(drop_block_table);

        std::string cleanup_metadata = "DELETE FROM USER_SDO_GEOM_METADATA "
            "WHERE TABLE_NAME ='" + blockTableName + "'";
        run(cleanup_metadata);
    }

};


void writeData(Orientation orient, bool scaling, bool compression = false)
{
    Options options;

    options.add("capacity", 10000);
    options.add("connection", std::string(connectString));
    options.add("block_table_name", blockTableName);
    options.add("base_table_name", baseTableName);
    options.add("cloud_column_name", "CLOUD");
    options.add("srid", 26910);
    options.add("disable_cloud_trigger", true);
    options.add("store_dimensional_orientation",
        orient == Orientation::DimensionMajor);
    if (scaling)
    {
        options.add("offset_x", "auto");
        options.add("offset_y", "auto");
        options.add("offset_z", "auto");
        options.add("scale_x", 1e-6);
        options.add("scale_y", 1e-6);
        options.add("scale_z", 1e-6);
    }
    if (compression)
        options.add("compression", true);

    PointTable table;

    Options readerOps;
    readerOps.add("filename", Support::datapath("autzen/autzen-utm.las"));

    StageFactory f;
    LasReader reader;
    reader.setOptions(readerOps);

    SplitFilter split;
    split.setInput(reader);

    Stage* writer(f.createStage("writers.oci"));
    EXPECT_TRUE(writer);
    writer->setOptions(options);
    writer->setInput(split);

    writer->prepare(table);
    writer->execute(table);
}


// Compare the source LAS file with the extracted OCI data.
// Candidate is the OCI reader's view.
void compare(const PointViewPtr candidate, std::string filename)
{
    Options options;
    Option fn("filename", filename);
    options.add(fn);

    PointTable table;

    LasReader reader;
    reader.setOptions(options);

    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);

    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr source = *viewSet.begin();

    EXPECT_EQ(source->size(), candidate->size());
    PointId limit = (std::min)(source->size(), candidate->size());

    for (PointId i = 0; i < limit; ++i)
    {
        using namespace Dimension;

        int32_t sx = source->getFieldAs<int32_t>(Id::X, i);
        int32_t sy = source->getFieldAs<int32_t>(Id::Y, i);
        int32_t sz = source->getFieldAs<int32_t>(Id::Z, i);
        uint16_t sintensity = source->getFieldAs<uint16_t>(Id::Intensity, i);
        uint16_t sred = source->getFieldAs<uint16_t>(Id::Red, i);
        uint16_t sgreen = source->getFieldAs<uint16_t>(Id::Green, i);
        uint16_t sblue = source->getFieldAs<uint16_t>(Id::Blue, i);

        int32_t cx = candidate->getFieldAs<int32_t>(Id::X, i);
        int32_t cy = candidate->getFieldAs<int32_t>(Id::Y, i);
        int32_t cz = candidate->getFieldAs<int32_t>(Id::Z, i);
        uint16_t cintensity = candidate->getFieldAs<uint16_t>(Id::Intensity, i);
        uint16_t cred = candidate->getFieldAs<uint16_t>(Id::Red, i);
        uint16_t cgreen = candidate->getFieldAs<uint16_t>(Id::Green, i);
        uint16_t cblue = candidate->getFieldAs<uint16_t>(Id::Blue, i);

        EXPECT_EQ(sx, cx);
        EXPECT_EQ(sy, cy);
        EXPECT_EQ(sz, cz);
        EXPECT_EQ(sintensity, cintensity);
        EXPECT_EQ(sred, cred);
        EXPECT_EQ(sgreen, cgreen);
        EXPECT_EQ(sblue, cblue);
    }
}


void readData()
{
    std::ostringstream oss;

    oss << "SELECT  l.\"OBJ_ID\", l.\"BLK_ID\", l.\"BLK_EXTENT\", " <<
        "l.\"BLK_DOMAIN\", l.\"PCBLK_MIN_RES\", l.\"PCBLK_MAX_RES\", " <<
        "l.\"NUM_POINTS\", l.\"NUM_UNSORTED_POINTS\", l.\"PT_SORT_DIM\", " <<
        "l.\"POINTS\", b.cloud "
           "FROM PDAL_TEST_BLOCKS l, PDAL_TEST_BASE b "
        "WHERE b.id = l.obj_id ORDER BY l.blk_id ";

    Options options = readerOptions();
    options.add("query", oss.str());

    StageFactory f;
    Stage* reader(f.createStage("readers.oci"));
    EXPECT_TRUE(reader);

    reader->setOptions(options);

    PointTable table;
    reader->prepare(table);
    PointViewSet viewSet = reader->execute(table);

    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 1065u);

    compare(view, Support::datapath("autzen/autzen-utm.las"));
}


TEST_F(OCITest, dim_major_unscaled)
{
    writeData(Orientation::DimensionMajor, false);
    readData();
}


TEST_F(OCITest, dim_major_scaled)
{
    writeData(Orientation::DimensionMajor, true);
    readData();
}


TEST_F(OCITest, point_major_unscaled)
{
    writeData(Orientation::PointMajor, false);
    readData();
}


TEST_F(OCITest, point_major_unscaled_compressed)
{
    writeData(Orientation::PointMajor, false, true);
    readData();
}


TEST_F(OCITest, point_major_scaled)
{
    writeData(Orientation::PointMajor, true);
    readData();
}
