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

#include "gtest/gtest.h"

#include <pdal/Writer.hpp>
#include <pdal/StageFactory.hpp>

#include "Support.hpp"
#include "Pgtest-Support.hpp"
#include "../io/PgCommon.hpp"

using namespace pdal;

Options getWriterOptions()
{
    Options options;

    options.add(Option("connection", testDbTempConn));
    options.add(Option("table", "pdal_test_table"));
    options.add(Option("srid", "4326"));
    options.add(Option("capacity", "10000"));
    // options.add(Option("debug", true));
    // options.add(Option("verbose", 7));

    return options;
}

class PgpointcloudWriterTest : public testing::Test
{
  protected:
    virtual void SetUp()
    {
        m_masterConnection = pg_connect(testDbConn);
        m_testConnection = NULL;

        // Silence those pesky notices
        executeOnMasterDb("SET client_min_messages TO WARNING");

        dropTestDb();

        std::stringstream createDbSql;
        createDbSql << "CREATE DATABASE " <<
            testDbTempname << " TEMPLATE template0";
        executeOnMasterDb(createDbSql.str());
        m_testConnection = pg_connect(
                testDbTempConn);

        executeOnTestDb("CREATE EXTENSION pointcloud");
    }

    void executeOnTestDb(const std::string& sql)
    {
        pg_execute(m_testConnection, sql);
    }

    virtual void TearDown()
    {
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
        if (connection)
        {
            pg_execute(connection, sql);
        }
        else
        {
            throw std::runtime_error("Not connected to database for testing");
        }
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

TEST_F(PgpointcloudWriterTest, testWrite)
{
    StageFactory f;
    StageFactory::WriterCreator* wc = f.getWriterCreator("writers.pgpointcloud");
    if (wc)
    {
        EXPECT_TRUE(wc);
        const std::string file(Support::datapath("las/1.2-with-color.las"));

        const Option opt_filename("filename", file);

        StageFactory::ReaderCreator* rc = f.getReaderCreator("readers.las");
        if (rc)
        {
            EXPECT_TRUE(rc);

            Stage* reader = rc();
            Options options;
            options.add(opt_filename);
            reader->setOptions(options);
            std::unique_ptr<Writer> writer(wc());
            writer->setOptions(getWriterOptions());
            writer->setInput(reader);

            PointContext ctx;
            writer->prepare(ctx);

            PointBufferSet written = writer->execute(ctx);

            point_count_t count(0);
            for(auto i = written.begin(); i != written.end(); ++i)
            {
                count += (*i)->size();
            }
            EXPECT_EQ(written.size(), 1);
            // EXPECT_EQ(count, 0);
            EXPECT_EQ(count, 1065);
        }
    }
}


TEST_F(PgpointcloudWriterTest, testNoPointcloudExtension)
{
    StageFactory f;
    StageFactory::WriterCreator* wc = f.getWriterCreator("writers.pgpointcloud");
    if (wc)
    {
        EXPECT_TRUE(wc);

        executeOnTestDb("DROP EXTENSION pointcloud");

        const std::string file(Support::datapath("las/1.2-with-color.las"));

        const Option opt_filename("filename", file);

        StageFactory::ReaderCreator* rc = f.getReaderCreator("readers.las");
        if (rc)
        {
            EXPECT_TRUE(rc);

            Stage* reader = rc();
            Options options;
            options.add(opt_filename);
            reader->setOptions(options);
            std::unique_ptr<Writer> writer(wc());
            writer->setOptions(getWriterOptions());
            writer->setInput(reader);

            PointContext ctx;
            writer->prepare(ctx);

            EXPECT_THROW(writer->execute(ctx), pdal_error);
        }
    }
}
