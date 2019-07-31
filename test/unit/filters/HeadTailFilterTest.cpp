/******************************************************************************
* Copyright (c) 2019, Hobu Inc. (info@hobu.co)
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
*     * Neither the name of Hobu, Inc. nor the
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
#include <io/FauxReader.hpp>
#include <filters/HeadFilter.hpp>
#include <filters/TailFilter.hpp>

#include "Support.hpp"

using namespace pdal;


void testFilter(bool head, bool invert)
{
    BOX3D srcBounds(0.0, 0.0, 1.0, 0.0, 0.0, 10.0);

    Options ops;
    ops.add("bounds", srcBounds);
    ops.add("mode", "ramp");
    ops.add("count", 10);

    FauxReader reader;
    reader.setOptions(ops);

    Options ops2;
    ops2.add("count", 4);
    ops2.add("invert", invert);

    StageFactory fac;
    Stage& f = *(fac.createStage(head ? "filters.head" : "filters.tail"));
    f.setOptions(ops2);
    f.setInput(reader);

    PointTable t;
    f.prepare(t);
    PointViewSet s = f.execute(t);
    PointViewPtr v = *s.begin();
    EXPECT_EQ(v->size(), invert ? 6u : 4u);

    int min { 0 };
    if (head)
        min = invert ? 5 : 1;
    else
        min = invert ? 1 : 7;

    for (PointId i = 0; i < v->size(); ++i)
    {
        int z = v->getFieldAs<int>(Dimension::Id::Z, i);
        EXPECT_EQ(z, min);
        min++;
    }

}

TEST(HeadTailFilterTest, t1)
{
    testFilter(true, true);
    testFilter(true, false);
    testFilter(false, true);
    testFilter(false, false);
}

