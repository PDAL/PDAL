/******************************************************************************
* Copyright (c) 2017, Hobu Inc. (info@hobu.co)
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

void testOverlay(int numReaders, bool stream)
{
    Options ro;
    ro.add("filename", Support::datapath("autzen/autzen-dd.las"));

    StageFactory factory;

    Options fo;
    fo.add("dimension", "Classification");
    fo.add("column", "cls");
    fo.add("datasource", Support::datapath("autzen/attributes.shp"));

    LogPtr l(Log::makeLog("readers.las", "stderr"));
    Stage& f = *(factory.createStage("filters.overlay"));
    for (int i = 0; i < numReaders; ++i)
    {
        Stage& r = *(factory.createStage("readers.las"));
        r.setLog(l);
        r.setOptions(ro);
        f.setInput(r);
    }
    f.setOptions(fo);

    std::string tempfile(Support::temppath("out.las"));

    Options wo;
    wo.add("filename", tempfile);
    wo.add("forward", "all");
    Stage& w = *(factory.createStage("writers.las"));
    w.setInput(f);
    w.setOptions(wo);

    FileUtils::deleteFile(tempfile);
    if (stream)
    {
        FixedPointTable t(100);
        w.prepare(t);
        w.execute(t);
    }
    else
    {
        PointTable t;
        w.prepare(t);
        w.execute(t);
    }
//
//
    Options testOptions;
    testOptions.add("filename", tempfile);

    Stage& test = *(factory.createStage("readers.las"));
    test.setOptions(testOptions);

    Stage& c = *(factory.createStage("filters.crop"));
    c.setInput(test);

    Options o1;
    o1.add("polygon", "POLYGON ((-123.067019000727967 44.059524946819884,-123.066697831944637 44.059771500882199,-123.065494970755537 44.059838504937517,-123.064074882074451 44.059742872480356,-123.063707784110264 44.059184772926969,-123.06401206144227 44.05752667418929,-123.065925935478475 44.057786669839672,-123.065745673821624 44.058221493390228,-123.06633379431868 44.058264073197797,-123.066237450902648 44.058593303895073,-123.06633104847343 44.058966114097124,-123.06667192031054 44.059429072738524,-123.067019000727967 44.059524946819884))");

    c.setOptions(o1);

    PointTable t1;
    c.prepare(t1);
    PointViewSet s = c.execute(t1);
    PointViewPtr v = *s.begin();
    for (PointId i = 0; i < v->size(); ++i)
        EXPECT_EQ(v->getFieldAs<uint8_t>(Dimension::Id::Classification, i), ClassLabel::Ground);
    Options o2;
    o2.add("polygon", "POLYGON ((-123.064404672110015 44.062248205780641,-123.063001791092177 44.062295757390288,-123.062938948566199 44.061866413306625,-123.063711529699802 44.061825083575727,-123.062903757506561 44.060798284876931,-123.0634590922878 44.06084959530147,-123.063939134831102 44.061205726019097,-123.064707560079256 44.061834553141757,-123.064404672110015 44.062248205780641))");
    c.setOptions(o2);

    PointTable t2;
    c.prepare(t2);
    s = c.execute(t2);
    v = *s.begin();
    for (PointId i = 0; i < v->size(); ++i)
        EXPECT_EQ(v->getFieldAs<uint8_t>(Dimension::Id::Classification, i), ClassLabel::HighVegetation);

    Options o3;
    o3.add("polygon", "POLYGON ((-123.071105494548775 44.059121603563895,-123.070160265697027 44.059007342229378,-123.070431650409773 44.058379004056327,-123.071225617617372 44.058576450660659,-123.071105494548775 44.059121603563895))");
    c.setOptions(o3);

    PointTable t3;
    c.prepare(t3);
    s = c.execute(t3);
    v = *s.begin();
    for (PointId i = 0; i < v->size(); ++i)
        EXPECT_EQ(v->getFieldAs<uint8_t>(Dimension::Id::Classification, i), ClassLabel::Building);

    Options o4;
    o4.add("polygon", "POLYGON ((-123.06887558672102 44.059227793751305,-123.068425255931515 44.059301773132752,-123.067936238335534 44.059248990941661,-123.06749590912527 44.059043544671049,-123.06710185170931 44.058629650191868,-123.067099413306224 44.058063335596437,-123.067333190514816 44.057587417330502,-123.067852567837207 44.057320435256116,-123.068290450314009 44.057214830538491,-123.068899280712444 44.057225021409202,-123.069338909397359 44.057414523826182,-123.069655618140771 44.057830157147713,-123.069799646794209 44.058090147755628,-123.069828689995248 44.058504298331584,-123.069657775977149 44.058899032362596,-123.06887558672102 44.059227793751305))");
    c.setOptions(o4);

    PointTable t4;
    c.prepare(t4);
    s = c.execute(t4);
    v = *s.begin();
    for (PointId i = 0; i < v->size(); ++i)
        EXPECT_EQ(v->getFieldAs<uint8_t>(Dimension::Id::Classification, i), ClassLabel::Building);

    Options o5;
    o5.add("polygon", "POLYGON ((-123.071871740947586 44.058426242457685,-123.070376025800414 44.058117017731242,-123.07060216253906 44.057465769662898,-123.072144836409578 44.057837746292243,-123.071871740947586 44.058426242457685))");
    c.setOptions(o5);

    PointTable t5;
    c.prepare(t5);
    s = c.execute(t5);
    v = *s.begin();
    for (PointId i = 0; i < v->size(); ++i)
        EXPECT_EQ(v->getFieldAs<uint8_t>(Dimension::Id::Classification, i), ClassLabel::Building);
}

TEST(OverlayFilterTest, nostream)
{
    testOverlay(10, false);
}

TEST(OverlayFilterTest, stream)
{
    testOverlay(10, true);
}
