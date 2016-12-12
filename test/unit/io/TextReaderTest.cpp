/******************************************************************************
 * Copyright (c) 2016, Hobu Inc. (info@hobu.co)
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
#include <io/TextReader.hpp>

using namespace pdal;

void compareTextLas(const std::string& textFilename,
    const std::string& lasFilename)
{
    TextReader t;
    Options to;
    to.add("filename", textFilename);
    t.setOptions(to);

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
       EXPECT_DOUBLE_EQ(tv->getFieldAs<double>(Dimension::Id::X, i),
           lv->getFieldAs<double>(Dimension::Id::X, i));
       EXPECT_DOUBLE_EQ(tv->getFieldAs<double>(Dimension::Id::Y, i),
           lv->getFieldAs<double>(Dimension::Id::Y, i));
       EXPECT_DOUBLE_EQ(tv->getFieldAs<double>(Dimension::Id::Z, i),
           lv->getFieldAs<double>(Dimension::Id::Z, i));
    }
}

TEST(TextReaderTest, t1)
{
    compareTextLas(Support::datapath("text/utm17_1.txt"),
        Support::datapath("las/utm17.las"));
}

TEST(TextReaderTest, t2)
{
    compareTextLas(Support::datapath("text/utm17_2.txt"),
        Support::datapath("las/utm17.las"));
}

TEST(TextReaderTest, t3)
{
    compareTextLas(Support::datapath("text/utm17_3.txt"),
        Support::datapath("las/utm17.las"));
}

TEST(TextReaderTest, badheader)
{
    TextReader t;
    Options to;
    to.add("filename", Support::datapath("text/badheader.txt"));
    t.setOptions(to);

    PointTable tt;
    EXPECT_THROW(t.prepare(tt), pdal_error);
}
