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

#include <pdal/StageFactory.hpp>
#include <pdal/util/FileUtils.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(AssignFilterTest, value)
{
    Options ro;
    ro.add("filename", Support::datapath("autzen/autzen-dd.las"));

    StageFactory factory;
    Stage& r = *(factory.createStage("readers.las"));
    r.setOptions(ro);

    Options fo;
    /**
    fo.add("assignment", "X[:]=27.5");
    fo.add("assignment", "Classification[:]=0");
    fo.add("assignment", "GpsTime[:]=3000");
    fo.add("assignment", "Red[:]=255");
    **/
    fo.add("value", "X = 27.5");
    fo.add("value", "Classification = 0");
    fo.add("value", "GpsTime = 3000");
    fo.add("value", "Red = 255");

    Stage& f = *(factory.createStage("filters.assign"));
    f.setInput(r);
    f.setOptions(fo);

    std::string tempfile(Support::temppath("out.las"));

    Options wo;
    wo.add("filename", tempfile);
    Stage& w = *(factory.createStage("writers.las"));
    w.setInput(f);
    w.setOptions(wo);

    FileUtils::deleteFile(tempfile);
    PointTable t1;
    w.prepare(t1);
    w.execute(t1);

    Options testOptions;
    testOptions.add("filename", tempfile);

    Stage& test = *(factory.createStage("readers.las"));
    test.setOptions(testOptions);

    PointTable t2;
    test.prepare(t2);
    PointViewSet s = test.execute(t2);
    PointViewPtr v = *s.begin();
    for (PointId i = 0; i < v->size(); ++i)
    {
        EXPECT_DOUBLE_EQ(v->getFieldAs<double>(Dimension::Id::X, i), 27.5);
        EXPECT_EQ(v->getFieldAs<uint8_t>(
            Dimension::Id::Classification, i), ClassLabel::CreatedNeverClassified);
        EXPECT_DOUBLE_EQ(v->getFieldAs<double>(
            Dimension::Id::GpsTime, i), 3000.0);
        EXPECT_EQ(v->getFieldAs<int>(
            Dimension::Id::Red, i), 255);
    }
}


TEST(AssignFilterTest, t2)
{
    StageFactory factory;

    Stage& r = *factory.createStage("readers.las");
    Stage& f = *factory.createStage("filters.assign");

    // utm17.las contains 5 points with intensity of 280, 3 of 260 and 2 of 240
    Options ro;
    ro.add("filename", Support::datapath("las/utm17.las"));
    r.setOptions(ro);

    Options fo;
    /**
    fo.add("assignment", "Intensity[:250]=4");
    fo.add("assignment", "Intensity[245:270 ]=6");
    fo.add("assignment", "Intensity[272:] = 8");
    **/
    fo.add("value", "Intensity = 4 where Intensity <= 250");
    fo.add("value", "Intensity = 6 where Intensity >= 245 && intensity <= 270");
    fo.add("value", "Intensity = 8 where Intensity >= 272");

    f.setInput(r);
    f.setOptions(fo);

    PointTable t;
    f.prepare(t);
    PointViewSet s = f.execute(t);
    PointViewPtr v = *s.begin();

    int i4 = 0;
    int i6 = 0;
    int i8 = 0;
    for (PointId i = 0; i < v->size(); ++i)
    {
        int ii = v->getFieldAs<int>(Dimension::Id::Intensity, i);
        if (ii == 4)
            i4++;
        else if (ii == 6)
            i6++;
        else if (ii == 8)
            i8++;
    }
    EXPECT_EQ(i4, 2);
    EXPECT_EQ(i6, 3);
    EXPECT_EQ(i8, 5);
}

TEST(AssignFilterTest, test_condition)
{
    StageFactory factory;

    Stage& r = *factory.createStage("readers.las");
    Stage& f = *factory.createStage("filters.assign");

    // utm17.las contains 5 points with intensity of 280, 3 of 260 and 2 of 240
    Options ro;
    ro.add("filename", Support::datapath("las/utm17.las"));
    r.setOptions(ro);

    Options fo;
    /**
    fo.add("condition", "Intensity[260:260]");
    fo.add("assignment", "PointSourceId[:]=6");
    **/
    fo.add("value", "PointSourceId = 6 where intensity == 260");

    f.setInput(r);
    f.setOptions(fo);

    PointTable t;
    f.prepare(t);
    PointViewSet s = f.execute(t);
    PointViewPtr v = *s.begin();

    int ielse = 0;
    int i6 = 0;
    for (PointId i = 0; i < v->size(); ++i)
    {
        int ii = v->getFieldAs<int>(Dimension::Id::PointSourceId, i);
        if (ii == 6)
            i6++;
        else
            ielse++;
    }
    EXPECT_EQ(i6, 3);
    EXPECT_EQ(v->size(), 10u);
    EXPECT_EQ(ielse, 7);
}
