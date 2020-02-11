/******************************************************************************
 * Copyright (c) 2018, 2020 Bradley J Chambers (brad.chambers@gmail.com)
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

#include <filters/PMFFilter.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/pdal_test_main.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(PMFFilterTest, invalidReturns)
{
    Options opts;
    opts.add("returns", "foo");

    PMFFilter filter;
    filter.setOptions(opts);

    PointTable table;
    EXPECT_THROW(filter.prepare(table), pdal_error);
}

TEST(PMFFilterTest, validReturns)
{
    Options opts;
    opts.add("returns", "last,first,intermediate,only");

    PMFFilter filter;
    filter.setOptions(opts);

    PointTable table;
    EXPECT_NO_THROW(filter.prepare(table));
}

TEST(PMFFilterTest, classOneAndTwoOnly)
{
    StageFactory f;

    Stage* reader(f.createStage("readers.las"));
    Options rOpts;
    rOpts.add("filename", Support::datapath("las/autzen_trim.las"));
    reader->setOptions(rOpts);

    Stage* assign(f.createStage("filters.assign"));
    Options aOpts;
    aOpts.add("assignment", "Classification[:]=0");
    assign->setInput(*reader);
    assign->setOptions(aOpts);

    Stage* filter(f.createStage("filters.pmf"));
    Options fOpts;
    fOpts.add("returns", "first,last,intermediate,only");
    filter->setInput(*assign);
    filter->setOptions(fOpts);

    PointTable t;
    filter->prepare(t);
    PointViewSet s = filter->execute(t);

    EXPECT_EQ(s.size(), 1U);
    PointViewPtr v = *s.begin();

    size_t classZero{0};
    for (PointId id = 0; id < v->size(); ++id)
    {
        uint8_t cl = v->getFieldAs<uint8_t>(Dimension::Id::Classification, id);
        if (cl == ClassLabel::CreatedNeverClassified)
            classZero++;
    }
    EXPECT_EQ(v->size(), 110000u);
    EXPECT_EQ(classZero, 0u);
}
