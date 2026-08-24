/******************************************************************************
* Copyright (c) 2023, Howard Butler (howard@hobu.co)*
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
*
****************************************************************************/

#include <pdal/pdal_test_main.hpp>

#include <pdal/StageFactory.hpp>
#include <io/LasReader.hpp>
#include <io/BufferReader.hpp>
#include <io/LasWriter.hpp>
#include "../io/ArrowReader.hpp"
#include "Support.hpp"
#include <pdal/util/FileUtils.hpp>

namespace pdal
{

namespace
{

void compareArrowLasStreaming(const std::string& pcdFilename,
                            const std::string& lasFilename, bool compareSRS=false)
{
    std::string tempname(Support::temppath("testlas.las"));

    FileUtils::deleteFile(tempname);

    ArrowReader t;
    Options to;
    to.add("filename", pcdFilename);
    t.setOptions(to);

    LasWriter w;
    Options wo;
    wo.add("filename", tempname);
    w.setInput(t);
    w.setOptions(wo);

    FixedPointTable in(1000);
    w.prepare(in);
    w.execute(in);

    LasReader l1;
    Options l1o;
    l1o.add("filename", lasFilename);
    l1.setOptions(l1o);

    LasReader l2;
    Options l2o;
    l2o.add("filename", tempname);
    l2.setOptions(l2o);

    PointTable t1;
    l1.prepare(t1);
    PointViewSet s1 = l1.execute(t1);
    EXPECT_EQ(s1.size(), 1U);
    PointViewPtr v1 = *s1.begin();

    PointTable t2;
    l2.prepare(t2);
    PointViewSet s2 = l2.execute(t2);
    EXPECT_EQ(s2.size(), 1U);
    PointViewPtr v2 = *s2.begin();

    EXPECT_EQ(v1->size(), v2->size());
    if (compareSRS)
        EXPECT_EQ(v1->spatialReference(), v2->spatialReference());

    // Validate some point data.
    for (PointId i = 0; i < v1->size(); ++i)
    {
        EXPECT_DOUBLE_EQ(v1->getFieldAs<float>(Dimension::Id::X, i),
                         v2->getFieldAs<float>(Dimension::Id::X, i));
        EXPECT_DOUBLE_EQ(v1->getFieldAs<float>(Dimension::Id::Y, i),
                         v2->getFieldAs<float>(Dimension::Id::Y, i));
        EXPECT_DOUBLE_EQ(v1->getFieldAs<float>(Dimension::Id::Z, i),
                         v2->getFieldAs<float>(Dimension::Id::Z, i));
    }
}

}  // unnamed namespace


TEST(ArrowParquetReaderTest, ReadingPoints)
{
    compareArrowLasStreaming(Support::datapath("arrow/1.2-with-color.parquet"),
                             Support::datapath("las/1.2-with-color.las"));
}

TEST(ArrowParquetReaderTest, ReadingPoints_RowGroups)
{
    compareArrowLasStreaming(Support::datapath("arrow/1.2-with-color_rowgroups.parquet"),
                             Support::datapath("las/1.2-with-color.las"));
}

TEST(ArrowParquetReaderTest, ReadingPoints_GeoParquetPrimaryColumn)
{
    // input file that only has a 'wkb' geometry column (no 'xyz' struct)
    compareArrowLasStreaming(Support::datapath("arrow/1.2-with-color_wkb.parquet"),
                             Support::datapath("las/1.2-with-color.las"));
}

TEST(ArrowParquetReaderTest, ReadingPoints_GeoParquetV1x)
{
    compareArrowLasStreaming(Support::datapath("arrow/1.2-with-color_v1.parquet"),
                             Support::datapath("las/1.2-with-color.las"));
 
}

TEST(ArrowParquetReaderTest, ReadingPoints_GeoParquetV2)
{
    compareArrowLasStreaming(Support::datapath("arrow/1.2-with-color_v2.parquet"),
                             Support::datapath("las/1.2-with-color.las"));
}

TEST(ArrowParquetReaderTest, ReadingPoints_SRS)
{
    compareArrowLasStreaming(Support::datapath("arrow/autzen-utm.parquet"),
                             Support::datapath("autzen/autzen-utm.las"), true);
}

TEST(ArrowParquetReaderTest, ReadingPoints_SRS_GeoParquetV2)
{
    compareArrowLasStreaming(Support::datapath("arrow/autzen-utm_v2.parquet"),
                             Support::datapath("autzen/autzen-utm.las"), true);
}

// File with no geoparquet "geo" metadata, just native parquet geometry type
TEST(ArrowParquetReaderTest, ReadingPoints_noMD)
{
    ArrowReader r;
    Options opts;
    opts.add("filename", Support::datapath("arrow/no-metadata.parquet"));
    opts.add("geoarrow_dimension_name", "foo");
    r.addOptions(opts);

    PointTable t;
    t.layout()->registerDim(Dimension::Id::X);
    t.layout()->registerDim(Dimension::Id::Y);
    t.layout()->registerDim(Dimension::Id::Z);

    PointViewPtr v1(new PointView(t));
    v1->setField(Dimension::Id::X, 0, 1);
    v1->setField(Dimension::Id::Y, 0, 1);
    v1->setField(Dimension::Id::Z, 0, 1);

    v1->setField(Dimension::Id::X, 1, 1);
    v1->setField(Dimension::Id::Y, 1, 2);
    v1->setField(Dimension::Id::Z, 1, 1);

    v1->setField(Dimension::Id::X, 2, 2);
    v1->setField(Dimension::Id::Y, 2, 3);
    v1->setField(Dimension::Id::Z, 2, 4);

    PointTable t2;
    r.prepare(t2);
    PointViewSet viewSet = r.execute(t2);
    PointViewPtr v2 = *viewSet.begin();
    EXPECT_EQ(v1->size(), v2->size());

    for (PointId i = 0; i < v1->size(); ++i)
    {
        EXPECT_DOUBLE_EQ(v1->getFieldAs<float>(Dimension::Id::X, i),
                         v2->getFieldAs<float>(Dimension::Id::X, i));
        EXPECT_DOUBLE_EQ(v1->getFieldAs<float>(Dimension::Id::Y, i),
                         v2->getFieldAs<float>(Dimension::Id::Y, i));
        EXPECT_DOUBLE_EQ(v1->getFieldAs<float>(Dimension::Id::Z, i),
                         v2->getFieldAs<float>(Dimension::Id::Z, i));
    }
}

TEST(ArrowParquetReaderTest, SRS_noMD)
{
    ArrowReader r;
    Options opts;
    opts.add("filename", Support::datapath("arrow/no-metadata_utm.parquet"));
    opts.add("geoarrow_dimension_name", "foo");
    r.addOptions(opts);

    PointTable table;
    r.prepare(table);
    PointViewSet viewSet = r.execute(table);
    PointViewPtr v = *viewSet.begin();
    EXPECT_EQ(v->spatialReference(), SpatialReference("EPSG:32610"));
}

} // namespace pdal

