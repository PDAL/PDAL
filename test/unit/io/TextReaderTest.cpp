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
#include <io/LasWriter.hpp>
#include <io/TextReader.hpp>
#include <pdal/util/FileUtils.hpp>

using namespace pdal;

void compareTextLas(const std::string& textFilename,
    Options& textOptions, const std::string& lasFilename)
{
    TextReader t;
    textOptions.add("filename", textFilename);
    t.setOptions(textOptions);

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


void compareTextLas(const std::string& textFilename,
    const std::string& lasFilename)
{
    Options textOptions;

    compareTextLas(textFilename, textOptions, lasFilename);
}


void compareTextLasStreaming(const std::string& textFilename,
    const std::string& lasFilename)
{
    std::string tempname(Support::temppath("testlas.las"));

    FileUtils::deleteFile(tempname);

    TextReader t;
    Options to;
    to.add("filename", textFilename);
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
       EXPECT_DOUBLE_EQ(v1->getFieldAs<double>(Dimension::Id::X, i),
           v2->getFieldAs<double>(Dimension::Id::X, i));
       EXPECT_DOUBLE_EQ(v1->getFieldAs<double>(Dimension::Id::Y, i),
           v2->getFieldAs<double>(Dimension::Id::Y, i));
       EXPECT_DOUBLE_EQ(v1->getFieldAs<double>(Dimension::Id::Z, i),
           v2->getFieldAs<double>(Dimension::Id::Z, i));
    }
}

TEST(TextReaderTest, t1)
{
    compareTextLas(Support::datapath("text/utm17_1.txt"),
        Support::datapath("las/utm17.las"));
}

TEST(TextReaderTest, t1a)
{
    Options textOptions;

    textOptions.add("separator", ',');

    compareTextLas(Support::datapath("text/utm17_1.txt"),
        textOptions, Support::datapath("las/utm17.las"));
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

TEST(TextReaderTest, s1)
{
    compareTextLasStreaming(Support::datapath("text/utm17_1.txt"),
                            Support::datapath("las/utm17.las"));
}

TEST(TextReaderTest, strip_whitespace_from_dimension_names)
{
    TextReader reader;
    Options options;
    options.add("filename", Support::datapath("text/crlf_test.txt"));
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);
    PointViewSet pointViewSet = reader.execute(table);
    PointViewPtr pointViewPtr = *pointViewSet.begin();

    for (PointId i = 0; i < pointViewPtr->size(); ++i) {
        EXPECT_EQ(
            i, pointViewPtr->getFieldAs<uint16_t>(Dimension::Id::Intensity, i));
    }
}

// Check that space-delimited files work with a CR/LF terminated file.
TEST(TextReaderTest, issue1939)
{
    TextReader reader;
    Options options;
    options.add("filename", Support::datapath("text/crlf_test2.txt"));
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);
    PointViewSet pointViewSet = reader.execute(table);
    PointViewPtr pointViewPtr = *pointViewSet.begin();

    EXPECT_EQ(pointViewPtr->size(), 10U);
    for (PointId i = 0; i < pointViewPtr->size(); ++i) {
        EXPECT_EQ(
            i, pointViewPtr->getFieldAs<uint16_t>(Dimension::Id::Intensity, i));
    }
}

TEST(TextReaderTest, warnMissingHeader)
{
    std::string infile = Support::datapath("text/missingheader.txt");
    std::string outfile = Support::temppath("out.txt");

    std::string cmd = "pdal translate " + infile + " " + outfile + " 2>&1";
    std::string output;

    Utils::run_shell_command(Support::binpath(cmd), output);
    EXPECT_NE(output.find("doesn't appear to contain a header"),
        std::string::npos);
}

// Skip the file's header and use another in it's place.
TEST(TextReaderTest, overrideHeader)
{
    TextReader reader;
    Options options;
    options.add("skip", 1);
    options.add("header", "A,B,C,G");
    options.add("filename", Support::datapath("text/crlf_test.txt"));
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);
    PointViewSet pointViewSet = reader.execute(table);
    PointViewPtr pointViewPtr = *pointViewSet.begin();

    EXPECT_EQ(pointViewPtr->size(), 10U);
    PointLayoutPtr layout = table.layout();
    EXPECT_TRUE(layout->findDim("A") != Dimension::Id::Unknown);
    EXPECT_TRUE(layout->findDim("B") != Dimension::Id::Unknown);
    EXPECT_TRUE(layout->findDim("C") != Dimension::Id::Unknown);
    EXPECT_TRUE(layout->findDim("G") != Dimension::Id::Unknown);
}

TEST(TextReaderTest, insertHeader)
{
    TextReader reader;
    Options options;
    options.add("header", "A,B,C,G");
    options.add("filename", Support::datapath("text/crlf_test.txt"));
    reader.setOptions(options);

    PointTable table;
    reader.prepare(table);
    PointViewSet s = reader.execute(table);
    PointViewPtr v = *s.begin();

    EXPECT_EQ(v->size(), 11U);
    PointLayoutPtr layout = table.layout();
    EXPECT_TRUE(layout->findDim("A") != Dimension::Id::Unknown);
    EXPECT_TRUE(layout->findDim("B") != Dimension::Id::Unknown);
    EXPECT_TRUE(layout->findDim("C") != Dimension::Id::Unknown);
    EXPECT_TRUE(layout->findDim("G") != Dimension::Id::Unknown);
}

TEST(TextReaderTest, quotedHeader)
{
    auto testme = [](Options& options,
        const std::string& filename = "text/quoted.txt")
    {
        TextReader reader;
        options.add("filename", Support::datapath(filename));
        reader.setOptions(options);

        PointTable table;
        reader.prepare(table);
        PointViewSet s = reader.execute(table);
        PointViewPtr v = *s.begin();
        EXPECT_EQ(v->size(), 9U);
        PointLayoutPtr layout = table.layout();
        EXPECT_TRUE(layout->findDim("X") != Dimension::Id::Unknown);
        EXPECT_TRUE(layout->findDim("Y") != Dimension::Id::Unknown);
        EXPECT_TRUE(layout->findDim("Z") != Dimension::Id::Unknown);
    };

    {
        Options opts;
        testme(opts);
    }

    {
        Options opts;
        opts.add("header", "\"X\",\"Y\",\"Z\"");
        opts.add("skip", 1);
        testme(opts);
    }

    {
        Options opts;
        opts.add("header", "\"X\",  \"Y\"  , \"Z\"  ");
        opts.add("skip", 1);
        testme(opts);
    }

    {
        Options opts;
        opts.add("header", "\"X\",\"Y\"   \"  ");
        opts.add("skip", 1);
        EXPECT_THROW(testme(opts), pdal_error);
    }

    {
        Options opts;
        opts.add("header", "  \"X\",,\"Y\",\"Z\"");
        opts.add("skip", 1);
        EXPECT_THROW(testme(opts), pdal_error);
    }

    {
        Options opts;
        opts.add("separator", ' ');
        testme(opts, "text/quoted2.txt");
    }
}
