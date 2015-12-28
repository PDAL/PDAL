/******************************************************************************
* Copyright (c) 2015, Hobu Inc. (info@hobu.co)
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

#include "GDALReader.hpp"
#include "Support.hpp"

#include <iostream>

using namespace pdal;

TEST(GDALReaderTest, simple)
{
    Options ro;
    ro.add("filename", Support::datapath("png/autzen-height.png"));

    GDALReader gr;
    gr.setOptions(ro);

    PointTable t;
    gr.prepare(t);
    PointViewSet s = gr.execute(t);
    PointViewPtr v = *s.begin();
    PointLayoutPtr l = t.layout();
    Dimension::Id::Enum id1 = l->findDim("band-1");
    Dimension::Id::Enum id2 = l->findDim("band-2");
    Dimension::Id::Enum id3 = l->findDim("band-3");
    EXPECT_EQ(v->size(), (size_t)(735 * 973));

    auto verify = [v, id1, id2, id3]
        (PointId idx, double xx, double xy, double xr, double xg, double xb)
    {
        double r, g, b, x, y;
        x = v->getFieldAs<double>(Dimension::Id::X, idx);
        y = v->getFieldAs<double>(Dimension::Id::Y, idx);
        r = v->getFieldAs<double>(id1, idx);
        g = v->getFieldAs<double>(id2, idx);
        b = v->getFieldAs<double>(id3, idx);
        EXPECT_DOUBLE_EQ(x, xx);
        EXPECT_DOUBLE_EQ(y, xy);
        EXPECT_DOUBLE_EQ(r, xr);
        EXPECT_DOUBLE_EQ(g, xg);
        EXPECT_DOUBLE_EQ(b, xb);
    };

    verify(0, .5, .5, 0, 0, 0);
    verify(120000, 163.5, 195.5, 255, 213, 0);
    verify(290000, 394.5, 410.5, 0, 255, 206);
    verify(715154, 972.5, 734.5, 0, 0, 0);
}
