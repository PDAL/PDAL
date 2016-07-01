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

#include <FauxReader.hpp>
#include <RandomizeFilter.hpp>

using namespace pdal;

TEST(RandomizeFilterTest, simple)
{
    // This isn't a real test.  It's just here to allow easy debugging.

    point_count_t count = 1000;

    Options readerOps;
    readerOps.add("bounds", BOX3D(1, 1, 1, count, count, count));
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

