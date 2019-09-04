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

#include <io/LasReader.hpp>
#include <io/LasWriter.hpp>
#include <io/PcdReader.hpp>
#include <pdal/util/FileUtils.hpp>

using namespace pdal;

void comparePcdLas(const std::string& pcdFilename, Options& pcdOptions,
                   const std::string& lasFilename)
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

void comparePcdLas(const std::string& pcdFilename,
                   const std::string& lasFilename)
{
    Options pcdOptions;

    comparePcdLas(pcdFilename, pcdOptions, lasFilename);
}

void comparePcdLasStreaming(const std::string& pcdFilename,
                            const std::string& lasFilename)
{
    std::string tempname(Support::temppath("testlas.las"));

    FileUtils::deleteFile(tempname);

    PcdReader t;
    Options to;
    to.add("filename", pcdFilename);
    t.setOptions(to);

    LasWriter w;
    Options wo;
    wo.add("filename", tempname);
    w.setInput(t);
    w.setOptions(wo);

    FixedPointTable in(1000);
    w.prepare(in);
    w.execute(in);

    LasReader l1;
    Options l1o;
    l1o.add("filename", lasFilename);
    l1.setOptions(l1o);

    LasReader l2;
    Options l2o;
    l2o.add("filename", tempname);
    l2.setOptions(l2o);

    PointTable t1;
    l1.prepare(t1);
    PointViewSet s1 = l1.execute(t1);
    EXPECT_EQ(s1.size(), 1U);
    PointViewPtr v1 = *s1.begin();

    PointTable t2;
    l2.prepare(t2);
    PointViewSet s2 = l2.execute(t2);
    EXPECT_EQ(s2.size(), 1U);
    PointViewPtr v2 = *s2.begin();

    EXPECT_EQ(v1->size(), v2->size());

    // Validate some point data.
    for (PointId i = 0; i < v1->size(); ++i)
    {
        EXPECT_DOUBLE_EQ(v1->getFieldAs<float>(Dimension::Id::X, i),
                         v2->getFieldAs<float>(Dimension::Id::X, i));
        EXPECT_DOUBLE_EQ(v1->getFieldAs<float>(Dimension::Id::Y, i),
                         v2->getFieldAs<float>(Dimension::Id::Y, i));
        EXPECT_DOUBLE_EQ(v1->getFieldAs<float>(Dimension::Id::Z, i),
                         v2->getFieldAs<float>(Dimension::Id::Z, i));
    }
}

TEST(PcdReaderTest, compareSpaceSeparated)
{
    comparePcdLas(Support::datapath("pcd/utm17_space.pcd"),
                  Support::datapath("las/utm17.las"));
}

TEST(PcdReaderTest, compateTabSeparated)
{
    comparePcdLas(Support::datapath("pcd/utm17_tab.pcd"),
                  Support::datapath("las/utm17.las"));
}

TEST(PcdReaderTest, throwCommaSeparated)
{
    PcdReader t;
    Options to;
    to.add("filename", Support::datapath("pcd/utm17_comma.pcd"));
    t.setOptions(to);

    PointTable tt;
    t.prepare(tt);

    PointViewSet s = t.execute(tt);
    EXPECT_EQ(s.size(), 1U);
    PointViewPtr v = *s.begin();

    EXPECT_EQ((size_t)0, v->size());
}

TEST(PcdReaderTest, compareSpaceSeparatedStreaming)
{
    comparePcdLasStreaming(Support::datapath("pcd/utm17_space.pcd"),
                           Support::datapath("las/utm17.las"));
}

TEST(PcdReaderTest, compareTabSeparatedStreaming)
{
    comparePcdLasStreaming(Support::datapath("pcd/utm17_tab.pcd"),
                           Support::datapath("las/utm17.las"));
}

TEST(PcdReaderTest, throwMissingHeader)
{
    PcdReader t;
    Options to;
    to.add("filename", Support::datapath("pcd/missingheader.pcd"));
    t.setOptions(to);

    PointTable tt;
    EXPECT_THROW(t.prepare(tt), pdal_error);
}
