/******************************************************************************
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

#include "Support.hpp"

#include <io/BufferReader.hpp>
#include <io/LasReader.hpp>
#include <io/PcdReader.hpp>
#include <io/PcdWriter.hpp>
#include <pdal/util/FileUtils.hpp>

using namespace pdal;

void comparePcdPcd(const std::string& srcFilename, Options& pcdOptions,
                   const std::string& dstFilename)
{
    PcdReader t;
    pcdOptions.add("filename", srcFilename);
    t.setOptions(pcdOptions);

    PcdReader l;
    Options lo;
    lo.add("filename", dstFilename);
    l.setOptions(lo);

    PointTable tt;
    t.prepare(tt);
    PointViewSet ts = t.execute(tt);
    EXPECT_EQ(ts.size(), 1U);
    PointViewPtr tv = *ts.begin();

    PointTable lt;
    l.prepare(lt);
    PointViewSet ls = l.execute(lt);
    EXPECT_EQ(ls.size(), 1U);
    PointViewPtr lv = *ls.begin();

    EXPECT_EQ(tv->size(), lv->size());

    // Validate some point data.
    for (PointId i = 0; i < lv->size(); ++i)
    {
        EXPECT_DOUBLE_EQ(tv->getFieldAs<float>(Dimension::Id::X, i),
                         lv->getFieldAs<float>(Dimension::Id::X, i));
        EXPECT_DOUBLE_EQ(tv->getFieldAs<float>(Dimension::Id::Y, i),
                         lv->getFieldAs<float>(Dimension::Id::Y, i));
        EXPECT_DOUBLE_EQ(tv->getFieldAs<float>(Dimension::Id::Z, i),
                         lv->getFieldAs<float>(Dimension::Id::Z, i));
    }
}

void comparePcdPcd(const std::string& srcFilename,
                   const std::string& dstFilename)
{
    Options pcdOptions;

    comparePcdPcd(srcFilename, pcdOptions, dstFilename);
}

void compareLasPcd(const std::string& lasFilename, Options& pcdOptions,
                   const std::string& pcdFilename)
{
    PcdReader t;
    pcdOptions.add("filename", pcdFilename);
    t.setOptions(pcdOptions);

    LasReader l;
    Options lo;
    lo.add("filename", lasFilename);
    l.setOptions(lo);

    PointTable tt;
    t.prepare(tt);
    PointViewSet ts = t.execute(tt);
    EXPECT_EQ(ts.size(), 1U);
    PointViewPtr tv = *ts.begin();

    PointTable lt;
    l.prepare(lt);
    PointViewSet ls = l.execute(lt);
    EXPECT_EQ(ls.size(), 1U);
    PointViewPtr lv = *ls.begin();

    EXPECT_EQ(tv->size(), lv->size());

    // Validate some point data.
    for (PointId i = 0; i < lv->size(); ++i)
    {
        EXPECT_DOUBLE_EQ(tv->getFieldAs<float>(Dimension::Id::X, i),
                         lv->getFieldAs<float>(Dimension::Id::X, i));
        EXPECT_DOUBLE_EQ(tv->getFieldAs<float>(Dimension::Id::Y, i),
                         lv->getFieldAs<float>(Dimension::Id::Y, i));
        EXPECT_DOUBLE_EQ(tv->getFieldAs<float>(Dimension::Id::Z, i),
                         lv->getFieldAs<float>(Dimension::Id::Z, i));
    }
}

void compareLasPcd(const std::string& lasFilename,
                   const std::string& pcdFilename)
{
    Options pcdOptions;

    compareLasPcd(lasFilename, pcdOptions, pcdFilename);
}

TEST(PcdWriterTest, pcd2pcd)
{
    std::string outfile(Support::temppath("utm17.pcd"));
    std::string infile(Support::datapath("pcd/utm17_space.pcd"));

    FileUtils::deleteFile(outfile);

    PcdReader r;
    Options ro;

    ro.add("filename", infile);
    r.setOptions(ro);

    PcdWriter w;
    Options wo;

    wo.add("filename", outfile);
    wo.add("order", "X,Y,Z");
    wo.add("precision", 2);
    w.setOptions(wo);
    w.setInput(r);

    PointTable t;

    w.prepare(t);
    w.execute(t);

    comparePcdPcd(infile, outfile);
}

TEST(PcdWriterTest, las2pcd)
{
    std::string outfile(Support::temppath("utm17.pcd"));
    std::string infile(Support::datapath("las/utm17.las"));

    FileUtils::deleteFile(outfile);

    LasReader r;
    Options ro;

    ro.add("filename", infile);
    r.setOptions(ro);

    PcdWriter w;
    Options wo;

    wo.add("filename", outfile);
    wo.add("order", "X,Y,Z");
    wo.add("precision", 2);
    w.setOptions(wo);
    w.setInput(r);

    PointTable t;

    w.prepare(t);
    w.execute(t);

    compareLasPcd(infile, outfile);
}

TEST(PcdWriterTest, precision)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::X, Id::Y, Id::Z, Id::Intensity});

    PointViewPtr view(new PointView(table));
    view->setField(Id::X, 0, 1);
    view->setField(Id::Y, 0, 1);
    view->setField(Id::Z, 0, 1);
    view->setField(Id::Intensity, 0, 1);

    view->setField(Id::X, 1, 2.2222222222);
    view->setField(Id::Y, 1, 2.2222222222);
    view->setField(Id::Z, 1, 2.2222222222);
    view->setField(Id::Intensity, 1, 2.22222222);

    view->setField(Id::X, 2, 3.33);
    view->setField(Id::Y, 2, 3.33);
    view->setField(Id::Z, 2, 3.33);
    view->setField(Id::Intensity, 2, 3.33);

    BufferReader r;
    r.addView(view);

    std::string outfile(Support::temppath("precision.pcd"));

    PcdWriter w;

    Options o;
    o.add("precision", 5);
    o.add("order", "X=Float:0,Y=Float:0,Z=Float:0,Intensity=Float:0");
    o.add("filename", outfile);

    w.setInput(r);
    w.setOptions(o);

    w.prepare(table);
    w.execute(table);

    PcdReader pr;
    Options po;
    po.add("filename", outfile);
    pr.setOptions(po);
    PointTable t;
    pr.prepare(t);
    PointViewSet pvs = pr.execute(t);

    PointViewPtr v = *pvs.begin();
    EXPECT_EQ(3U, v->size());

    // Validate some point data.
    EXPECT_DOUBLE_EQ(1, v->getFieldAs<float>(Dimension::Id::X, 0));
    EXPECT_DOUBLE_EQ(1, v->getFieldAs<float>(Dimension::Id::Y, 0));
    EXPECT_DOUBLE_EQ(1, v->getFieldAs<float>(Dimension::Id::Z, 0));

    EXPECT_DOUBLE_EQ(2, v->getFieldAs<float>(Dimension::Id::X, 1));
    EXPECT_DOUBLE_EQ(2, v->getFieldAs<float>(Dimension::Id::Y, 1));
    EXPECT_DOUBLE_EQ(2, v->getFieldAs<float>(Dimension::Id::Z, 1));

    EXPECT_DOUBLE_EQ(3, v->getFieldAs<float>(Dimension::Id::X, 2));
    EXPECT_DOUBLE_EQ(3, v->getFieldAs<float>(Dimension::Id::Y, 2));
    EXPECT_DOUBLE_EQ(3, v->getFieldAs<float>(Dimension::Id::Z, 2));
}

TEST(PcdWriterTest, binaryPdalTypes)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims({Id::X, Id::Y, Id::Z, Id::Intensity});

    PointViewPtr view(new PointView(table));
    view->setField(Id::X, 0, 1);
    view->setField(Id::Y, 0, 1);
    view->setField(Id::Z, 0, 1);
    view->setField(Id::Intensity, 0, 1);

    view->setField(Id::X, 1, 2.2222222222);
    view->setField(Id::Y, 1, 2.2222222222);
    view->setField(Id::Z, 1, 2.2222222222);
    view->setField(Id::Intensity, 1, 2.22222222);

    view->setField(Id::X, 2, 3.33);
    view->setField(Id::Y, 2, 3.33);
    view->setField(Id::Z, 2, 3.33);
    view->setField(Id::Intensity, 2, 3.33);

    BufferReader r;
    r.addView(view);

    std::string outfile(Support::temppath("binary-test.pcd"));

    PcdWriter w;

    Options o;
    o.add("order", "X=Float,Y=Float,Z=Float,Intensity=Unsigned32");
    o.add("compression", "binary");
    o.add("filename", outfile);

    w.setInput(r);
    w.setOptions(o);

    w.prepare(table);
    w.execute(table);

    PcdReader pr;
    Options po;
    po.add("filename", outfile);
    pr.setOptions(po);
    PointTable t;
    pr.prepare(t);
    PointViewSet pvs = pr.execute(t);

    PointViewPtr v = *pvs.begin();
    EXPECT_EQ(3U, v->size());

    // Validate some point data.
    EXPECT_DOUBLE_EQ(1, v->getFieldAs<float>(Dimension::Id::X, 0));
    EXPECT_DOUBLE_EQ(1, v->getFieldAs<float>(Dimension::Id::Y, 0));
    EXPECT_DOUBLE_EQ(1, v->getFieldAs<float>(Dimension::Id::Z, 0));
    EXPECT_EQ(1, v->getFieldAs<int>(Dimension::Id::Intensity, 0));

    EXPECT_NEAR(2.2222222222, v->getFieldAs<float>(Dimension::Id::X, 1),
        0.0001);
    EXPECT_NEAR(2.2222222222, v->getFieldAs<float>(Dimension::Id::Y, 1),
        0.0001);
    EXPECT_NEAR(2.2222222222, v->getFieldAs<float>(Dimension::Id::Z, 1),
        0.0001);
    EXPECT_EQ(2, v->getFieldAs<int>(Dimension::Id::Intensity, 1));

    EXPECT_NEAR(3.33, v->getFieldAs<float>(Dimension::Id::X, 2), 0.0001);
    EXPECT_NEAR(3.33, v->getFieldAs<float>(Dimension::Id::Y, 2), 0.0001);
    EXPECT_NEAR(3.33, v->getFieldAs<float>(Dimension::Id::Z, 2), 0.0001);
    EXPECT_EQ(3, v->getFieldAs<int>(Dimension::Id::Intensity, 2));
}
