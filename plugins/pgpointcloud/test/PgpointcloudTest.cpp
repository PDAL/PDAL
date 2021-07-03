/******************************************************************************
* Copyright (c) 2014, Pete Gadomski (pete.gadomski@gmail.com)
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

#include <pdal/pdal_test_main.hpp>

#include <pdal/Writer.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/util/Algorithm.hpp>

#include "Support.hpp"
#include "Pgtest-Support.hpp"
#include "../io/PgCommon.hpp"

using namespace pdal;

namespace { // anonymous

std::string getTestConnBase()
{
    std::string s;
    if ( ! testDbPort.empty() )
        s += " port='" + testDbPort + "'";
    if ( ! testDbHost.empty() )
        s += " host='" + testDbHost + "'";
    if ( ! testDbUser.empty() )
        s += " user='" + testDbUser + "'";
    return s;
}


std::string getConnectionString(const std::string& dbname)
{
    return getTestConnBase() + " dbname='" + dbname + "'";
}


std::string getTestDBTempConn()
{
    return getConnectionString(testDbTempname);
}

std::string getMasterDBConn()
{
    return getConnectionString(testDbName);
}

} // anonymous namespace

Options getDbOptions()
{
    Options options;

    options.add(Option("connection", getTestDBTempConn()));
    options.add(Option("table", "4dal-\"test\"-table")); // intentional quotes
    options.add(Option("column", "p\"a")); // intentional quotes

    return options;
}

class PgpointcloudTest : public testing::Test
{
public:
    PgpointcloudTest() : m_masterConnection(0), m_testConnection(0),
        m_bSkipTests(false) {};
protected:
    virtual void SetUp()
    {
        std::string connstr = getMasterDBConn();
        m_masterConnection = pg_connect( connstr );
        m_testConnection = NULL;

        // Silence those pesky notices
        executeOnMasterDb("SET client_min_messages TO WARNING");

        dropTestDb();

        std::stringstream createDbSql;
        createDbSql << "CREATE DATABASE " <<
            testDbTempname << " TEMPLATE template0";
        try
        {
            executeOnMasterDb(createDbSql.str());
        }
        catch( const pdal_error& )
        {
            m_bSkipTests = true;
            return;
        }

        m_testConnection = pg_connect( getTestDBTempConn() );

        try
        {
            executeOnTestDb("CREATE EXTENSION pointcloud");
        }
        catch( const pdal_error& )
        {
            m_bSkipTests = true;
            return;
        }
    }

    void executeOnTestDb(const std::string& sql)
    {
        pg_execute(m_testConnection, sql);
    }

    virtual void TearDown()
    {
        if (!m_testConnection || !m_masterConnection) return;
        if (m_testConnection)
        {
            PQfinish(m_testConnection);
        }
        dropTestDb();
        if (m_masterConnection)
        {
            PQfinish(m_masterConnection);
        }
    }

    bool shouldSkipTests() const { return m_bSkipTests; }

private:

    void executeOnMasterDb(const std::string& sql)
    {
        pg_execute(m_masterConnection, sql);
    }

    void execute(PGconn* connection, const std::string& sql)
    {
        pg_execute(connection, sql);
    }

    void dropTestDb()
    {
        std::stringstream dropDbSql;
        dropDbSql << "DROP DATABASE IF EXISTS " << testDbTempname;
        executeOnMasterDb(dropDbSql.str());
    }

    PGconn* m_masterConnection;
    PGconn* m_testConnection;
    bool m_bSkipTests;
};

class PgpointcloudWriterTest : public PgpointcloudTest
{
};

namespace
{

void optionsWrite(const Options& writerOps)
{
    StageFactory f;
    Stage* reader(f.createStage("readers.las"));

    const std::string file(Support::datapath("las/1.2-with-color.las"));
    Options options;
    options.add("filename", file);
    reader->setOptions(options);

    Stage* writer(f.createStage("writers.pgpointcloud"));
    writer->setOptions(writerOps);
    writer->setInput(*reader);

    PointTable table;
    writer->prepare(table);

    PointViewSet written = writer->execute(table);

    point_count_t count(0);
    for(auto i = written.begin(); i != written.end(); ++i)
	    count += (*i)->size();
    EXPECT_EQ(written.size(), 1U);
    EXPECT_EQ(count, 1065U);
}

} // unnamed namespace

TEST_F(PgpointcloudWriterTest, write)
{
    if (shouldSkipTests())
    {
        return;
    }

    optionsWrite(getDbOptions());
}

TEST_F(PgpointcloudWriterTest, writeScaled)
{
    if (shouldSkipTests())
    {
        return;
    }

    Options ops = getDbOptions();
    ops.add("scale_x", .01);
    ops.add("scale_y", .01);
    ops.add("scale_z", .01);

    optionsWrite(ops);
}

TEST_F(PgpointcloudWriterTest, writeXYZ)
{
    if (shouldSkipTests())
    {
        return;
    }

    Options ops = getDbOptions();
    ops.add("output_dims", "X,Y,Z");

    optionsWrite(ops);

    PointTable table;
    StageFactory factory;
    Stage* reader(factory.createStage("readers.pgpointcloud"));
    reader->setOptions(getDbOptions());

    reader->prepare(table);
    Dimension::IdList dims = table.layout()->dims();
    EXPECT_EQ(dims.size(), (size_t)3);
    EXPECT_TRUE(Utils::contains(dims, Dimension::Id::X));
    EXPECT_TRUE(Utils::contains(dims, Dimension::Id::Y));
    EXPECT_TRUE(Utils::contains(dims, Dimension::Id::Z));
}

TEST_F(PgpointcloudWriterTest, writetNoPointcloudExtension)
{
    if (shouldSkipTests())
    {
        return;
    }

    StageFactory f;
    Stage* writer(f.createStage("writers.pgpointcloud"));
    EXPECT_TRUE(writer);

    executeOnTestDb("DROP EXTENSION pointcloud");

    const std::string file(Support::datapath("las/1.2-with-color.las"));

    const Option opt_filename("filename", file);

    Stage* reader(f.createStage("readers.las"));
    EXPECT_TRUE(reader);
    Options options;
    options.add(opt_filename);
    reader->setOptions(options);
    writer->setOptions(getDbOptions());
    writer->setInput(*reader);

    PointTable table;
    writer->prepare(table);

    EXPECT_THROW(writer->execute(table), pdal_error);
}

TEST_F(PgpointcloudWriterTest, writeDeleteTable)
{
    if (shouldSkipTests())
    {
        return;
    }

    executeOnTestDb("CREATE SCHEMA \"4dal-\"\"test\"\"-schema\"");
    executeOnTestDb("CREATE TABLE \"4dal-\"\"test\"\"-schema\"."
                    "\"4dal-\"\"test\"\"-table\" (p PCPATCH)");
    Options ops = getDbOptions();
    ops.add("overwrite", true);
    ops.add("schema", "4dal-\"test\"-schema");

    optionsWrite(ops);
}

TEST_F(PgpointcloudWriterTest, selectExistingSchema)
{
    if (shouldSkipTests())
    {
	return;
    }

    executeOnTestDb("CREATE SCHEMA \"4dal-\"\"test\"\"-schema\"");

    // Create a data table for one SRS
    Options ops = getDbOptions();
    ops.add("srid", 31467);
    ops.replace("table", "data_srs_1");
    optionsWrite(ops);

    // Create a data table for another SRS
    ops.replace("srid", 25832);
    ops.replace("table", "data_srs_2");
    optionsWrite(ops);


    // Read the data from the first data table
    PointTable table1;
    StageFactory factory;
    Stage* reader1(factory.createStage("readers.pgpointcloud"));
    Options rops = getDbOptions();
    rops.replace("table", "data_srs_1");
    reader1->setOptions(rops);

    // ... verify srs settings used by the reader 
    reader1->prepare(table1);
    PointViewSet pv1 = reader1->execute(table1);
    EXPECT_TRUE(reader1->getSpatialReference().valid());
    EXPECT_EQ("31467", reader1->getSpatialReference().identifyHorizontalEPSG());

    // ... and verify the resulting SRS in the data table
    EXPECT_NE( pv1.begin(), pv1.end());
    SpatialReference srs = (*pv1.begin())->spatialReference();
    EXPECT_TRUE(srs.valid());
    EXPECT_EQ(std::string("31467"), srs.identifyHorizontalEPSG());


    // Read the data from the second data table
    PointTable table2;
    Stage* reader2(factory.createStage("readers.pgpointcloud"));
    rops.replace("table", "data_srs_2");

    // ... verify srs settings used by the reader
    reader2->setOptions(rops);
    reader2->prepare(table2);
    PointViewSet pv2 = reader2->execute(table2);
    EXPECT_TRUE(reader2->getSpatialReference().valid());
    EXPECT_EQ("25832", reader2->getSpatialReference().identifyHorizontalEPSG());

    // ... and verify the resulting SRS in the data table
    EXPECT_NE( pv2.begin(), pv2.end());
    srs = (*pv2.begin())->spatialReference();
    EXPECT_TRUE(srs.valid());
    EXPECT_EQ(std::string("25832"), srs.identifyHorizontalEPSG());
}

class PgpointcloudReaderTest : public PgpointcloudTest
{
};

// make sure Pgpointcloud readers supports streaming
TEST_F(PgpointcloudReaderTest, streaming )
{
    if (shouldSkipTests())
    {
        return;
    }
    executeOnTestDb("CREATE SCHEMA \"4dal-\"\"test\"\"-schema\"");
    
    // write sample data (las/1.2-with.color.las)
    // to newly created schema
    Options ops;
    ops = getDbOptions();
    optionsWrite(ops);


    StageFactory factory;
    Stage* reader(factory.createStage("readers.pgpointcloud"));
    reader->setOptions(ops);

    // make sure pipeline is streamble
    EXPECT_TRUE(reader->pipelineStreamable());

    // execute pipeline
    PointTable table;

    reader->prepare(table);
    PointViewSet pws = reader->execute(table);
    PointViewPtr pwsPtr = *pws.begin();


    // Returned dataset should have the same bounds
    // and point count as the LAS-file we inserted
    EXPECT_EQ(pwsPtr->size(), 1065U);

    BOX3D bbox_las = BOX3D(635619.85, 848899.7, 406.59, 638982.55, 853535.43, 586.38);
    BOX3D bbox_pg;
    pwsPtr->calculateBounds(bbox_pg);
    EXPECT_FLOAT_EQ((float)bbox_pg.maxx, (float)bbox_las.maxx);
    EXPECT_FLOAT_EQ((float)bbox_pg.maxy, (float)bbox_las.maxy);
    EXPECT_FLOAT_EQ((float)bbox_pg.maxz, (float)bbox_las.maxz);
    EXPECT_FLOAT_EQ((float)bbox_pg.minx, (float)bbox_las.minx);
    EXPECT_FLOAT_EQ((float)bbox_pg.miny, (float)bbox_las.miny);
    EXPECT_FLOAT_EQ((float)bbox_pg.minz, (float)bbox_las.minz);
}


