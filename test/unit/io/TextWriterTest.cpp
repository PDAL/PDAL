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

#include <pdal/util/FileUtils.hpp>
#include <io/BufferReader.hpp>
#include <io/TextReader.hpp>
#include <io/TextWriter.hpp>

using namespace pdal;

TEST(TextWriterTest, t1)
{
    std::string outfile(Support::temppath("utm17.txt"));
    std::string infile(Support::datapath("text/utm17_1.txt"));

    FileUtils::deleteFile(outfile);

    TextReader r;
    Options ro;

    ro.add("filename", infile);
    r.setOptions(ro);

    TextWriter w;
    Options wo;

    wo.add("filename", outfile);
    wo.add("order", "X,Y,Z");
    wo.add("quote_header", false);
    wo.add("precision", 2);
    w.setOptions(wo);
    w.setInput(r);

    PointTable t;

    w.prepare(t);
    w.execute(t);

    EXPECT_EQ(Support::compare_text_files(infile, outfile), true);
}

TEST(TextWriterTest, t2)
{
    std::string outfile(Support::temppath("utm17.txt"));
    std::string infile(Support::datapath("text/utm17_2.txt"));

    FileUtils::deleteFile(outfile);

    TextReader r;
    Options ro;

    ro.add("filename", infile);
    r.setOptions(ro);

    TextWriter w;
    Options wo;

    wo.add("filename", outfile);
    wo.add("order", "X,Y,Z");
    wo.add("quote_header", false);
    wo.add("precision", 2);
    wo.add("delimiter", "  ");
    w.setOptions(wo);
    w.setInput(r);

    PointTable t;

    w.prepare(t);
    w.execute(t);

    EXPECT_EQ(Support::compare_text_files(infile, outfile), true);
}

TEST(TextWriterTest, t2stream)
{
    std::string outfile(Support::temppath("utm17.txt"));
    std::string infile(Support::datapath("text/utm17_2.txt"));

    FileUtils::deleteFile(outfile);

    TextReader r;
    Options ro;

    ro.add("filename", infile);
    r.setOptions(ro);

    TextWriter w;
    Options wo;

    wo.add("filename", outfile);
    wo.add("order", "X,Y,Z");
    wo.add("quote_header", false);
    wo.add("precision", 2);
    wo.add("delimiter", "  ");
    w.setOptions(wo);
    w.setInput(r);

    FixedPointTable t(1000);

    w.prepare(t);
    w.execute(t);

    EXPECT_EQ(Support::compare_text_files(infile, outfile), true);
}

TEST(TextWriterTest, precision)
{
    using namespace Dimension;

    PointTable table;
    table.layout()->registerDims( { Id::X, Id::Y, Id::Z, Id::Intensity } );

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

    std::string outfile(Support::temppath("precision.txt"));

    TextWriter w;

    Options o;
    o.add("precision", 5);
    o.add("order", "X:0,Y:0,Z:0,Intensity:0");
    o.add("filename", outfile);

    w.setInput(r);
    w.setOptions(o);

    w.prepare(table);
    w.execute(table);

    std::string out = FileUtils::readFileIntoString(outfile);
    EXPECT_NE(out.find("1,1,1,1"), std::string::npos);
    EXPECT_NE(out.find("2,2,2,2"), std::string::npos);
    EXPECT_NE(out.find("3,3,3,3"), std::string::npos);
}

TEST(TextWriterTest, geojson)
{
    std::string outfile(Support::temppath("utm17.geojson"));
    std::string comparefile(Support::datapath("text/utm17_1.geojson"));
    std::string infile(Support::datapath("text/utm17_1.txt"));

    FileUtils::deleteFile(outfile);

    TextReader r;
    Options ro;

    ro.add("filename", infile);
    r.setOptions(ro);

    TextWriter w;
    Options wo;

    wo.add("filename", outfile);
    wo.add("format", "geojson");
    wo.add("order", "X,Y,Z");
    wo.add("write_header", true);
    wo.add("precision", 2);
    w.setOptions(wo);
    w.setInput(r);

    PointTable t;

    w.prepare(t);
    w.execute(t);

    EXPECT_EQ(Support::compare_text_files(comparefile, outfile), true);
}
