/******************************************************************************
* Copyright (c) 2018, Bradley J Chambers (brad.chambers@gmail.com)
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
#include <filters/SMRFilter.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(SMRFilterTest, invalidReturns)
{
    Options opts;
    opts.add("returns", "foo");

    SMRFilter filter;
    filter.setOptions(opts);

    PointTable table;
    EXPECT_THROW(filter.prepare(table), pdal_error);
}

TEST(SMRFilterTest, validReturns)
{
    Options opts;
    opts.add("returns", "last,first,intermediate,only");

    SMRFilter filter;
    filter.setOptions(opts);

    PointTable table;
    EXPECT_NO_THROW(filter.prepare(table));
}

// Issue 2775.  Test that files without return counts are processed correctly.
TEST(SMRFFilterTest, noreturns)
{
    StageFactory factory;

    Stage *r = factory.createStage("readers.text");
    Stage *f = factory.createStage("filters.smrf");

    Options rOptions;
    rOptions.add("filename", Support::datapath("text/utm17_1.txt"));
    r->setOptions(rOptions);

    f->setInput(*r);

    PointTable t;
    f->prepare(t);
    PointViewSet s = f->execute(t);
    EXPECT_EQ(s.size(), 1U);
    PointViewPtr v = *s.begin();
    EXPECT_EQ(v->size(), 10U);
    std::map<uint8_t, int> classCount;
    for (PointId idx = 0; idx < v->size(); ++idx)
    {
        uint8_t classification = v->getFieldAs<uint8_t>(Dimension::Id::Classification,
            idx);
        classCount[classification]++;
    }
    EXPECT_EQ(classCount.size(), 1U);
    EXPECT_EQ(classCount[ClassLabel::Ground], 10);
}
