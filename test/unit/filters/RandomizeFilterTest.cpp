/******************************************************************************
 * Copyright (c) 2015, Hobu Inc. (hobu@hobu.co)
 * Copyright (c) 2019, Bradley J Chambers (brad.chambers@gmail.com)
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

#include <filters/RandomizeFilter.hpp>
#include <io/FauxReader.hpp>

using namespace pdal;

TEST(RandomizeFilterTest, simple)
{
    // This isn't a real test.  It's just here to allow easy debugging.

    point_count_t count = 1000;

    Options readerOps;
    readerOps.add("bounds",
                  BOX3D(1, 1, 1, (double)count, (double)count, (double)count));
    readerOps.add("mode", "ramp");
    readerOps.add("count", count);

    FauxReader r;
    r.setOptions(readerOps);

    RandomizeFilter f;
    f.setInput(r);

    PointTable t;
    f.prepare(t);
    PointViewSet s = f.execute(t);

    EXPECT_EQ(s.size(), 1u);
    PointViewPtr v = *s.begin();
    EXPECT_EQ(v->size(), (size_t)count);

    /**
        for (PointId i = 0; i < count; i++)
            std::cerr << "X[" << i << "] = " <<
                v->getFieldAs<double>(Dimension::Id::X, i) << "!\n";
    **/
}

TEST(RandomizeFilterTest, seedZero)
{
    point_count_t count = 10;

    Options readerOps;
    readerOps.add("bounds",
                  BOX3D(1, 1, 1, (double)count, (double)count, double(count)));
    readerOps.add("mode", "ramp");
    readerOps.add("count", count);

    FauxReader r;
    r.setOptions(readerOps);

    Options filterOps;
    filterOps.add("seed", 0);

    RandomizeFilter f;
    f.setInput(r);
    f.setOptions(filterOps);

    PointTable t;
    f.prepare(t);
    PointViewSet s = f.execute(t);

    EXPECT_EQ(1u, s.size());
    PointViewPtr v = *s.begin();
    EXPECT_EQ((size_t)count, v->size());

    std::vector<double> vals{6, 2, 1, 7, 8, 4, 10, 9, 5, 3};

    for (PointId i = 0; i < count; ++i)
    {
        double x = v->getFieldAs<double>(Dimension::Id::X, i);
        EXPECT_EQ(vals[i], x);
    }
}

TEST(RandomizeFilterTest, seedOne)
{
    point_count_t count = 10;

    Options readerOps;
    readerOps.add("bounds",
                  BOX3D(1, 1, 1, (double)count, (double)count, double(count)));
    readerOps.add("mode", "ramp");
    readerOps.add("count", count);

    FauxReader r;
    r.setOptions(readerOps);

    Options filterOps;
    filterOps.add("seed", 1);

    RandomizeFilter f;
    f.setInput(r);
    f.setOptions(filterOps);

    PointTable t;
    f.prepare(t);
    PointViewSet s = f.execute(t);

    EXPECT_EQ(1u, s.size());
    PointViewPtr v = *s.begin();
    EXPECT_EQ((size_t)count, v->size());

    std::vector<double> vals{6, 10, 2, 5, 8, 1, 7, 9, 4, 3};

    for (PointId i = 0; i < count; ++i)
    {
        double x = v->getFieldAs<double>(Dimension::Id::X, i);
        EXPECT_EQ(vals[i], x);
    }
}

TEST(RandomizeFilterTest, unseeded)
{
    point_count_t count = 10;

    Options readerOps;
    readerOps.add("bounds",
                  BOX3D(1, 1, 1, (double)count, (double)count, double(count)));
    readerOps.add("mode", "ramp");
    readerOps.add("count", count);

    FauxReader r;
    r.setOptions(readerOps);

    RandomizeFilter f;
    f.setInput(r);

    PointTable t;
    f.prepare(t);
    PointViewSet s = f.execute(t);

    EXPECT_EQ(1u, s.size());
    PointViewPtr v = *s.begin();
    EXPECT_EQ((size_t)count, v->size());

    std::vector<double> vals{6, 2, 1, 7, 8, 4, 10, 9, 5, 3};

    // I don't love this test. With such a small number of samples (even with a
    // large number), there is some probability that at least one will match
    // what we expect from seed=0. It was the best I could think of for now
    // though. In short, some can match, but certainly not all of them should.
    size_t numMatches(0);
    for (PointId i = 0; i < count; ++i)
    {
        double x = v->getFieldAs<double>(Dimension::Id::X, i);
        if (vals[i] == x)
            numMatches++;
    }
    EXPECT_LT(numMatches, count);
}
