/******************************************************************************
* Copyright (c) 2019 TileDB, Inc
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
*     * Neither the name of Hobu, Inc. or Flaxen Consulting LLC nor the
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

#include <pdal/PointView.hpp>
#include <pdal/PipelineManager.hpp>
#include <pdal/StageFactory.hpp>

#include "Support.hpp"

#include "../io/TileDBReader.hpp"


namespace pdal
{

class TileDBReaderTest : public ::testing::Test
{
    protected:
        virtual void SetUp()
        {
        }
    };

    TEST_F(TileDBReaderTest, constructor)
    {
        TileDBReader reader;
    }

    TEST_F(TileDBReaderTest, findStage)
    {
        StageFactory factory;
        Stage* stage(factory.createStage("readers.tiledb"));
        EXPECT_TRUE(stage);
    }

    TEST_F(TileDBReaderTest, read)
    {
        tiledb::Context ctx;
        tiledb::VFS vfs(ctx);
        std::string pth(Support::datapath("tiledb/array"));

        Options opts;
        opts.add("array_name", pth);

        tiledb::Array array(ctx, pth, TILEDB_READ);
        auto domain = array.non_empty_domain<double>();
        std::vector<double> subarray;

        for (const auto& kv: domain)
        {
            subarray.push_back(kv.second.first);
            subarray.push_back(kv.second.second);
        }

        tiledb::Query q(ctx, array, TILEDB_READ);
        q.set_subarray(subarray);

        auto max_el = array.max_buffer_elements(subarray);
        std::vector<double> coords(max_el[TILEDB_COORDS].second);
        q.set_coordinates(coords);
        q.submit();
        array.close();

        TileDBReader reader;
        reader.setOptions(opts);

        PointTable table;
        reader.prepare(table);
        PointViewSet ts = reader.execute(table);
        EXPECT_EQ(ts.size(), 1U);
        PointViewPtr tv = *ts.begin();
        EXPECT_EQ(tv->size(), coords.size() / 3);

        BOX3D bbox;
        tv->calculateBounds(bbox);
        ASSERT_DOUBLE_EQ(subarray[0], bbox.minx);
        ASSERT_DOUBLE_EQ(subarray[2], bbox.miny);
        ASSERT_DOUBLE_EQ(subarray[4], bbox.minz);
        ASSERT_DOUBLE_EQ(subarray[1], bbox.maxx);
        ASSERT_DOUBLE_EQ(subarray[3], bbox.maxy);
        ASSERT_DOUBLE_EQ(subarray[5], bbox.maxz);
    }
}