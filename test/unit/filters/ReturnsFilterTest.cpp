/******************************************************************************
* Copyright (c) 2018, Hobu Inc. (info@hobu.co)
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

#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/PipelineManager.hpp>
#include "Support.hpp"

using namespace pdal;

TEST(ReturnsFilterTest, t1)
{
    StageFactory f;

    Stage *reader(f.createStage("readers.las"));
    Options rOpts;
    rOpts.add("filename", Support::datapath("las/autzen_trim.las"));
    reader->setOptions(rOpts);

    Stage* filter(f.createStage("filters.returns"));
    Options fOpts;
    fOpts.add("groups", "last, first");
    filter->setOptions(fOpts);
    filter->setInput(*reader);

    PointTable t;
    filter->prepare(t);
    PointViewSet s = filter->execute(t);

    EXPECT_EQ(s.size(), 2U);
    auto si = s.begin();

    // First view is first returns.
    PointViewPtr v = *si;
    EXPECT_EQ(v->size(), 9036U);
    for (PointId id = 0; id < v->size(); ++id)
    {
        uint8_t rn = v->getFieldAs<uint8_t>(Dimension::Id::ReturnNumber, id);
        uint8_t nr = v->getFieldAs<uint8_t>(Dimension::Id::NumberOfReturns, id);
        EXPECT_EQ(rn, 1);
        EXPECT_NE(nr, 1);
    }

    // Second view is last returns.
    si++;
    v = *si;
    EXPECT_EQ(v->size(), 9015U);
    for (PointId id = 0; id < v->size(); ++id)
    {
        uint8_t rn = v->getFieldAs<uint8_t>(Dimension::Id::ReturnNumber, id);
        uint8_t nr = v->getFieldAs<uint8_t>(Dimension::Id::NumberOfReturns, id);
        EXPECT_EQ(rn, nr);
        EXPECT_NE(rn, 1);
    }
}

