/******************************************************************************
* Copyright (c) 2018, Howard Butler (howard@hobu.co)
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

#include <pdal/PipelineManager.hpp>
#include <pdal/StageFactory.hpp>
#include <filters/StatsFilter.hpp>

#include "../io/NumpyReader.hpp"

#include <pdal/StageWrapper.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(NumpyReaderTest, NumpyReaderTest_read_fields)
{
    StageFactory f;

    Options ops;
    ops.add("filename", Support::datapath("plang/1.2-with-color.npy"));

    NumpyReader reader;
    reader.setOptions(ops);

    PointTable table;

    reader.prepare(table);
    PointViewSet viewSet = reader.execute(table);
    EXPECT_EQ(viewSet.size(), 1u);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 1065u);
    EXPECT_EQ(view->layout()->pointSize(), 34u);

    EXPECT_EQ(view->getFieldAs<int16_t>(pdal::Dimension::Id::Intensity,800),
        49);
    EXPECT_EQ(view->getFieldAs<int32_t>(pdal::Dimension::Id::X,400), 63679039);

}


TEST(NumpyReaderTest, NumpyReaderTest_read_array)
{
    StageFactory f;

    Options ops;
    ops.add("filename", Support::datapath("plang/perlin.npy"));

    NumpyReader reader;
    reader.setOptions(ops);

    PointTable table;

    reader.prepare(table);

    PointViewSet viewSet = reader.execute(table);
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 10000u);
    EXPECT_EQ(view->layout()->pointSize(), 16u);

    EXPECT_EQ(view->getFieldAs<double>(pdal::Dimension::Id::Intensity,5000),
        0.5);
    EXPECT_EQ(view->getFieldAs<uint32_t>(pdal::Dimension::Id::X,5000), 51u);
}

