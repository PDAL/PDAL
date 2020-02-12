/******************************************************************************
* Copyright (c) 2020, Hobu Inc. (info@hobu.co)
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
#include <pdal/StageFactory.hpp>
#include <io/BufferReader.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(SkewnessTest, t1)
{
    StageFactory f;

    Stage *reader(f.createStage("readers.las"));
    Options rOpts;
    rOpts.add("filename", Support::datapath("las/autzen_trim.las"));
    reader->setOptions(rOpts);

    Stage* filter(f.createStage("filters.skewnessbalancing"));
    filter->setInput(*reader);

    PointTable t;
    filter->prepare(t);
    PointViewSet s = filter->execute(t);

    EXPECT_EQ(s.size(), 1U);
    PointViewPtr v = *s.begin();

    size_t ground {0};
    for (PointId id = 0; id < v->size(); ++id)
    {
        uint8_t cl = v->getFieldAs<uint8_t>(Dimension::Id::Classification, id);
        if (cl == ClassLabel::Ground)
            ground++;
    }
    EXPECT_EQ(ground, 102234u);
}

TEST(SkewnessTest, t2)
{
    StageFactory f;

    Stage *reader(f.createStage("readers.faux"));
    Options rOpts;
    rOpts.add("mode", "constant");
    rOpts.add("count", 100);
    rOpts.add("bounds", "([1, 2],[2, 3],[3, 4])");
    reader->setOptions(rOpts);

    Stage* filter(f.createStage("filters.skewnessbalancing"));
    filter->setInput(*reader);

    PointTable t;
    filter->prepare(t);
    PointViewSet s = filter->execute(t);

    EXPECT_EQ(s.size(), 1U);
    PointViewPtr v = *s.begin();

    size_t ground {0};
    for (PointId id = 0; id < v->size(); ++id)
    {
        uint8_t cl = v->getFieldAs<uint8_t>(Dimension::Id::Classification, id);
        if (cl == ClassLabel::Ground)
            ground++;
    }
    EXPECT_EQ(ground, 0U);
}


TEST(SkewnessTest, t3)
{
    StageFactory f;
    PointTable t;
    t.layout()->registerDims(
        {Dimension::Id::Z, Dimension::Id::Classification} );

    PointViewPtr bv(new PointView(t));
    for (int i = 0; i < 10; ++i)
        bv->setField(Dimension::Id::Z, i, i - 5);

    BufferReader reader;
    reader.addView(bv);

    Stage* filter(f.createStage("filters.skewnessbalancing"));
    filter->setInput(reader);

    filter->prepare(t);

    PointViewSet s = filter->execute(t);

    EXPECT_EQ(s.size(), 1U);
    PointViewPtr v = *s.begin();

    size_t ground {0};
    for (PointId id = 0; id < v->size(); ++id)
    {
        uint8_t cl = v->getFieldAs<uint8_t>(Dimension::Id::Classification, id);
        if (cl == ClassLabel::Ground)
            ground++;
    }
    EXPECT_EQ(ground, 10U);
}


