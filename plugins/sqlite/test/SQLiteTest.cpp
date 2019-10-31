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

#include <pdal/pdal_test_main.hpp>

#include <pdal/util/FileUtils.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/PointView.hpp>
#include <pdal/pdal_config.hpp>
#include <io/LasReader.hpp>

#include "../io/SQLiteCommon.hpp"

#include "Support.hpp"

using namespace pdal;

std::string tempFilename(Support::temppath(
    "temp-SqliteWriterTest_test_simple_las.sqlite"));

Options getWriterOptions()
{
    Options options;

    options.add("overwrite", true);
    options.add("connection", tempFilename);
    options.add("block_table_name", "PDAL_TEST_BLOCKS");
    options.add("cloud_table_name", "PDAL_TEST_BASE");
    options.add("is3d", false);
    options.add("srid", 4326);
    options.add("cloud_column_name", "CLOUD");

    return options;
}

Options getReaderOptions()
{
    Options options;

    options.add("spatialreference", "EPSG:2926");
    options.add("connection", tempFilename);
    options.add("query", "SELECT b.schema, l.cloud, l.block_id, "
        "l.num_points, l.bbox, l.extent, l.points, b.cloud "
        "FROM PDAL_TEST_BLOCKS l, PDAL_TEST_BASE b "
        "WHERE l.cloud = b.cloud and l.cloud in (1) "
        "order by l.cloud");

    return options;
}

void testReadWrite(bool compression, bool scaling)
{
    FileUtils::deleteFile(tempFilename);

    Options writerOptions = getWriterOptions();
    if (scaling)
    {
        writerOptions.add("scale_x", 0.01);
        writerOptions.add("scale_y", 0.01);
    }
    writerOptions.add("compression", compression);

    StageFactory f;
    {
    Options lasReadOpts;
    lasReadOpts.add("filename", Support::datapath("las/1.2-with-color.las"));
    lasReadOpts.add("count", 11);

    LasReader reader;
    reader.setOptions(lasReadOpts);

    Stage* sqliteWriter(f.createStage("writers.sqlite"));
    sqliteWriter->setOptions(writerOptions);
    sqliteWriter->setInput(reader);

    PointTable table;
    sqliteWriter->prepare(table);
    sqliteWriter->execute(table);
    }

    {
    Stage* sqliteReader(f.createStage("readers.sqlite"));
    sqliteReader->setOptions(getReaderOptions());

    PointTable table2;
    sqliteReader->prepare(table2);
    PointViewSet viewSet = sqliteReader->execute(table2);
    EXPECT_EQ(viewSet.size(), 1U);
    PointViewPtr view = *viewSet.begin();

    using namespace Dimension;

    uint16_t reds[] = {68, 54, 112, 178, 134, 99, 90, 106, 106, 100, 64};
    for (PointId idx = 0; idx < 11; idx++)
    {
        uint16_t r = view->getFieldAs<uint16_t>(Id::Red, idx);
        EXPECT_EQ(r, reds[idx]);
    }
    int32_t x = view->getFieldAs<int32_t>(Id::X, 10);
    EXPECT_EQ(x, 636038);
    double xd = view->getFieldAs<double>(Id::X, 10);
	EXPECT_DOUBLE_EQ(xd, 636037.53);
    }

//    FileUtils::deleteFile(tempFilename);
}


TEST(SQLiteTest, readWrite)
{
    testReadWrite(false, false);
}

TEST(SQLiteTest, readWriteCompress)
{
    if (Config::hasFeature(Config::Feature::LAZPERF))
        testReadWrite(true, false);
}

TEST(SQLiteTest, readWriteScale)
{
    testReadWrite(false, true);
}

TEST(SQLiteTest, readWriteCompressScale)
{
    if (Config::hasFeature(Config::Feature::LAZPERF))
        testReadWrite(true, true);
}

TEST(SQLiteTest, Issue895)
{
    LogPtr log(Log::makeLog("Issue895", "stdout"));
    log->setLevel(LogLevel::Debug);

    const std::string filename(Support::temppath("issue895.sqlite"));

    FileUtils::deleteFile(filename);

    const char* sql;

    // make a DB, put a table in it
    {
        SQLite db(filename, log);
        db.connect(true);
        sql = "CREATE TABLE MyTable (id INTEGER PRIMARY KEY AUTOINCREMENT, "
            "data)";
        db.execute(sql);
    }

    // open the DB, manually check the tables
    {
        SQLite db(filename, log);
        db.connect(false);
        sql = "SELECT name FROM sqlite_master WHERE type = \"table\"";
        db.query(sql);

        // because order of the returned rows is undefined
        bool foundMine = false;
        bool foundTheirs = false;

        {
            const row* r = db.get();
            column const& c = r->at(0);

            foundMine = (strcmp(c.data.c_str(), "MyTable") == 0);
            foundTheirs = (strcmp(c.data.c_str(), "sqlite_sequence") == 0);
            //printf("%s %d %d\n", c.data.c_str(), (int)foundMine, (int)foundTheirs);
            EXPECT_TRUE(foundMine || foundTheirs);
        }

        EXPECT_TRUE(db.next());

        {
            const row* r = db.get();
            column const& c = r->at(0);
            foundMine |= (strcmp(c.data.c_str(), "MyTable") == 0);
            foundTheirs |= (strcmp(c.data.c_str(), "sqlite_sequence") == 0);
            //printf("%s %d %d\n", c.data.c_str(), (int)foundMine, (int)foundTheirs);
            EXPECT_TRUE(foundMine && foundTheirs);
        }

        EXPECT_FALSE(db.next());
    }

    // open the DB, ask if the tables exist
    {
        SQLite db(filename, LogPtr(log));
        db.connect(false);

        EXPECT_TRUE(db.doesTableExist("MyTable"));

        EXPECT_TRUE(db.doesTableExist("sqlite_sequence"));
    }
}


TEST(SQLiteTest, testSpatialite)
{
    LogPtr log(Log::makeLog("spat", "stdout"));
    log->setLevel(LogLevel::Debug);

    const std::string filename(Support::temppath("spat.sqlite"));

    FileUtils::deleteFile(filename);

    SQLite db(filename, LogPtr(log));
    db.connect(true);

    EXPECT_FALSE(db.haveSpatialite());

    db.loadSpatialite();
    db.initSpatialiteMetadata();

    EXPECT_TRUE(db.haveSpatialite());

//    FileUtils::deleteFile(filename);
}


TEST(SQLiteTest, testVersionInfo)
{
    LogPtr log = Log::makeLog("spver", "stdout");
    log->setLevel(LogLevel::Debug);

    const std::string filename(Support::temppath("spver.sqlite"));

    FileUtils::deleteFile(filename);

    SQLite db(filename, LogPtr(log));
    db.connect(true);
    db.loadSpatialite();

    const std::string p = db.getSQLiteVersion();
    EXPECT_EQ(p[0], '3'); // 3.8.9 as of this commit

    FileUtils::deleteFile(filename);
}
