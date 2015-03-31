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
    options.add(Option("table", "pdal-\"test\"-table")); // intentional quotes
    options.add(Option("column", "p\"a")); // intentional quotes
    options.add(Option("srid", "4326"));
    options.add(Option("capacity", "10000"));

    return options;
}

class PgpointcloudWriterTest : public testing::Test
{
public:
    PgpointcloudWriterTest() : m_masterConnection(0), m_testConnection(0) {};
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
        executeOnMasterDb(createDbSql.str());

        m_testConnection = pg_connect( getTestDBTempConn() );

        executeOnTestDb("CREATE EXTENSION pointcloud");
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
};

namespace
{

void optionsWrite(const Options& writerOps)
{
    StageFactory f;
    std::unique_ptr<Stage> writer(f.createStage("writers.pgpointcloud"));
    std::unique_ptr<Stage> reader(f.createStage("readers.las"));

    EXPECT_TRUE(writer.get());
    EXPECT_TRUE(reader.get());
    if (!writer.get() || !reader.get())
        return;

    const std::string file(Support::datapath("las/1.2-with-color.las"));
    Options options;
    options.add("filename", file);
    reader->setOptions(options);
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
    optionsWrite(getDbOptions());
}

TEST_F(PgpointcloudWriterTest, writeScaled)
{
    Options ops = getDbOptions();
    ops.add("scale_x", .01);
    ops.add("scale_y", .01);
    ops.add("scale_z", .01);

    optionsWrite(ops);
}

TEST_F(PgpointcloudWriterTest, writeXYZ)
{
    Options ops = getDbOptions();
    ops.add("output_dims", "X,Y,Z");

    optionsWrite(ops);

    PointTable table;
    std::unique_ptr<Stage> reader(
        StageFactory().createStage("readers.pgpointcloud"));
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
    StageFactory f;
    std::unique_ptr<Stage> writer(f.createStage("writers.pgpointcloud"));
    EXPECT_TRUE(writer.get());

    executeOnTestDb("DROP EXTENSION pointcloud");

    const std::string file(Support::datapath("las/1.2-with-color.las"));

    const Option opt_filename("filename", file);

    std::unique_ptr<Stage> reader(f.createStage("readers.las"));
    EXPECT_TRUE(reader.get());
    Options options;
    options.add(opt_filename);
    reader->setOptions(options);
    writer->setOptions(getDbOptions());
    writer->setInput(*reader);

    PointTable table;
    writer->prepare(table);

    EXPECT_THROW(writer->execute(table), pdal_error);
}
