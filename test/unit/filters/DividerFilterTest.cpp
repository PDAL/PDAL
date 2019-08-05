/******************************************************************************
* Copyright (c) 2015, Hobu Inc. (hobu@hobu.co)
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

#include <io/FauxReader.hpp>
#include <filters/DividerFilter.hpp>

using namespace pdal;

TEST(DividerFilterTest, partition_count)
{
    point_count_t count = 1000;

    Options readerOps;
    readerOps.add("bounds", BOX3D(1, 1, 1,
        (double)count, (double)count, (double)count));
    readerOps.add("mode", "ramp");
    readerOps.add("count", count);

    FauxReader r;
    r.setOptions(readerOps);

    Options filterOps;
    filterOps.add("count", 10);
    DividerFilter f;
    f.setInput(r);
    f.setOptions(filterOps);

    PointTable t;
    f.prepare(t);
    PointViewSet s = f.execute(t);

    EXPECT_EQ(s.size(), 10u);

    PointId i = 0;
    for (PointViewPtr v : s)
    {
        EXPECT_EQ(v->size(), 100u);
        for (PointId p = 0; p < v->size(); ++p)
        {
            EXPECT_DOUBLE_EQ((double)(i + 1),
                v->getFieldAs<double>(Dimension::Id::X, p));
            i++;
        }
    }
}

TEST(DividerFilterTest, partition_capacity)
{
    point_count_t count = 1000;

    Options readerOps;
    readerOps.add("bounds", BOX3D(1, 1, 1,
        (double)count, (double)count, (double)count));
    readerOps.add("mode", "ramp");
    readerOps.add("count", count);

    FauxReader r;
    r.setOptions(readerOps);

    Options filterOps;
    filterOps.add("capacity", 25);
    DividerFilter f;
    f.setInput(r);
    f.setOptions(filterOps);

    PointTable t;
    f.prepare(t);
    PointViewSet s = f.execute(t);

    EXPECT_EQ(s.size(), 40u);

    PointId i = 0;
    for (PointViewPtr v : s)
    {
        EXPECT_EQ(v->size(), 25u);
        for (PointId p = 0; p < v->size(); ++p)
        {
            EXPECT_DOUBLE_EQ((double)(i + 1),
                v->getFieldAs<double>(Dimension::Id::X, p));
            i++;
        }
    }
}

TEST(DividerFilterTest, round_robin_count)
{
    point_count_t count = 1000;

    Options readerOps;
    readerOps.add("bounds", BOX3D(1, 1, 1,
        (double)count, (double)count, (double)count));
    readerOps.add("mode", "ramp");
    readerOps.add("count", count);

    FauxReader r;
    r.setOptions(readerOps);

    Options filterOps;
    filterOps.add("count", 10);
    filterOps.add("mode", "round_robin");
    DividerFilter f;
    f.setInput(r);
    f.setOptions(filterOps);

    PointTable t;
    f.prepare(t);
    PointViewSet s = f.execute(t);

    EXPECT_EQ(s.size(), 10u);

    PointId i = 0;
    for (PointViewPtr v : s)
        EXPECT_EQ(v->size(), 100u);

    unsigned viewNum = 0;
    PointId start = 1;
    for (PointViewPtr v : s)
    {
        double value = static_cast<double>(start);
        for (PointId i = 0 ; i < v->size(); i++)
        {
            EXPECT_DOUBLE_EQ(value,
                v->getFieldAs<double>(Dimension::Id::X, i));
            value += 10;
        }
        start++;
    }
}


TEST(DividerFilterTest, round_robin_capacity)
{
    point_count_t count = 1000;

    Options readerOps;
    readerOps.add("bounds", BOX3D(1, 1, 1,
        (double)count, (double)count, (double)count));
    readerOps.add("mode", "ramp");
    readerOps.add("count", count);

    FauxReader r;
    r.setOptions(readerOps);

    Options filterOps;
    filterOps.add("capacity", 25);
    filterOps.add("mode", "round_robin");
    DividerFilter f;
    f.setInput(r);
    f.setOptions(filterOps);

    PointTable t;
    f.prepare(t);
    PointViewSet s = f.execute(t);

    EXPECT_EQ(s.size(), 40u);

    PointId i = 0;
    for (PointViewPtr v : s)
        EXPECT_EQ(v->size(), 25u);

    unsigned viewNum = 0;
    PointId start = 1;
    for (PointViewPtr v : s)
    {
        double value((double)start);
        for (PointId i = 0 ; i < v->size(); i++)
        {
            EXPECT_DOUBLE_EQ((double)value,
                v->getFieldAs<double>(Dimension::Id::X, i));
            value += 40;
        }
        start++;
    }
}

