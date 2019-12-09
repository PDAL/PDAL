/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <stdlib.h>

#include <pdal/pdal_features.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/StageWrapper.hpp>
#include <pdal/util/FileUtils.hpp>
#include <io/BufferReader.hpp>
#include <io/LasHeader.hpp>
#include <io/LasReader.hpp>
#include <io/LasWriter.hpp>
#include <io/BpfReader.hpp>

#include "Support.hpp"

namespace pdal
{

//ABELL - Should probably be moved to its own file.
class LasTester
{
public:
    LasHeader *header(LasWriter& w)
        { return &w.m_lasHeader; }
    SpatialReference srs(LasWriter& w)
        { return w.m_srs; }
    void addVlr(LasWriter& w, const std::string& userId, uint16_t recordId,
        std::string description, std::vector<uint8_t>& data)
        { w.addVlr(userId, recordId, description, data); }
};

} // namespace pdal

using namespace pdal;

TEST(LasWriterTest, srs)
{
    Options readerOps;
    readerOps.add("filename", Support::datapath("las/utm15.las"));

    LasReader reader;
    reader.setOptions(readerOps);

    Options writerOps;
    writerOps.add("filename", Support::temppath("out.las"));
    LasWriter writer;
    writer.setInput(reader);
    writer.setOptions(writerOps);

    PointTable table;
    writer.prepare(table);
    writer.execute(table);

    LasTester tester;
    SpatialReference srs = tester.srs(writer);
    EXPECT_EQ(srs, SpatialReference("EPSG:26915"));
}


TEST(LasWriterTest, srs2)
{
    Options readerOps;
    readerOps.add("filename", Support::datapath("las/utm15.las"));

    LasReader reader;
    reader.setOptions(readerOps);

    Options writerOps;
    writerOps.add("filename", Support::temppath("out.las"));
    writerOps.add("a_srs", "EPSG:32615");
    LasWriter writer;
    writer.setInput(reader);
    writer.setOptions(writerOps);

    PointTable table;
    writer.prepare(table);
    writer.execute(table);

    LasTester tester;
    SpatialReference srs = tester.srs(writer);
    EXPECT_EQ(srs, SpatialReference("EPSG:32615"));
}


TEST(LasWriterTest, auto_offset)
{
    using namespace Dimension;

    const std::string FILENAME(Support::temppath("offset_test.las"));
    PointTable table;

    table.layout()->registerDims({Id::X, Id::Y, Id::Z});

    BufferReader bufferReader;

    PointViewPtr view(new PointView(table));
    view->setField(Id::X, 0, 125000.00);
    view->setField(Id::X, 1, 74529.00);
    view->setField(Id::X, 2, 1000000.02);
    view->setField(Id::Y, 0, 0);
    view->setField(Id::Y, 1, 1);
    view->setField(Id::Y, 2, 2);
    view->setField(Id::Z, 0, -123);
    view->setField(Id::Z, 1, 456.78);
    view->setField(Id::Z, 2, 945.23);
    bufferReader.addView(view);

    view.reset(new PointView(table));
    view->setField(Id::X, 0, 25.00);
    view->setField(Id::X, 1, 74529.00);
    view->setField(Id::X, 2, 534252.35);
    view->setField(Id::Y, 0, 3);
    view->setField(Id::Y, 1, 4);
    view->setField(Id::Y, 2, 5);
    view->setField(Id::Z, 0, 1.5);
    view->setField(Id::Z, 1, 2147483524);
    view->setField(Id::Z, 2, 745.23);
    bufferReader.addView(view);

    Options writerOps;
    writerOps.add("filename", FILENAME);
    writerOps.add("offset_x", "auto");
    writerOps.add("scale_x", "auto");
    writerOps.add("offset_z", "auto");
    writerOps.add("scale_z", "auto");

    LasWriter writer;
    writer.setOptions(writerOps);
    writer.setInput(bufferReader);

    writer.prepare(table);
    writer.execute(table);

    Options readerOps;
    readerOps.add("filename", FILENAME);

    PointTable readTable;

    LasReader reader;
    reader.setOptions(readerOps);

    reader.prepare(readTable);
    EXPECT_DOUBLE_EQ(25.00, reader.header().offsetX());
    EXPECT_DOUBLE_EQ(0, reader.header().offsetY());
    EXPECT_DOUBLE_EQ(-123, reader.header().offsetZ());

    EXPECT_DOUBLE_EQ(0.00046564965530561733, reader.header().scaleX());
    EXPECT_DOUBLE_EQ(.01, reader.header().scaleY());
    // (max - min) are chosen to yield std::numeric_limits<int>::max();
    EXPECT_DOUBLE_EQ(1.0, reader.header().scaleZ());

    PointViewSet viewSet = reader.execute(readTable);
    EXPECT_EQ(viewSet.size(), 1u);
    view = *viewSet.begin();
    EXPECT_EQ(view->size(), 6u);
    EXPECT_NEAR(125000.00, view->getFieldAs<double>(Id::X, 0), .0001);
    EXPECT_NEAR(74529.00, view->getFieldAs<double>(Id::X, 1), .001);
    EXPECT_NEAR(1000000.02, view->getFieldAs<double>(Id::X, 2), .0001);
    EXPECT_NEAR(25.00, view->getFieldAs<double>(Id::X, 3), .0001);
    EXPECT_NEAR(74529.00, view->getFieldAs<double>(Id::X, 4), .001);
    EXPECT_NEAR(534252.35, view->getFieldAs<double>(Id::X, 5), .0001);

    FileUtils::deleteFile(FILENAME);
}

// Identical to above, but writes each input view to a separate output file.
TEST(LasWriterTest, auto_offset2)
{
    using namespace Dimension;

    const std::string outname(Support::temppath("offset_test#.las"));
    const std::string inname1(Support::temppath("offset_test1.las"));
    const std::string inname2(Support::temppath("offset_test2.las"));
    PointTable table;

    table.layout()->registerDims({Id::X, Id::Y, Id::Z});

    BufferReader bufferReader;

    PointViewPtr view(new PointView(table));
    view->setField(Id::X, 0, 125000.00);
    view->setField(Id::X, 1, 74529.00);
    view->setField(Id::X, 2, 1000000.02);
    view->setField(Id::Y, 0, 0);
    view->setField(Id::Y, 1, 1);
    view->setField(Id::Y, 2, 2);
    view->setField(Id::Z, 0, -123);
    view->setField(Id::Z, 1, 456.78);
    view->setField(Id::Z, 2, 945.23);
    bufferReader.addView(view);

    view.reset(new PointView(table));
    view->setField(Id::X, 0, 25.00);
    view->setField(Id::X, 1, 74529.00);
    view->setField(Id::X, 2, 534252.35);
    view->setField(Id::Y, 0, 3);
    view->setField(Id::Y, 1, 4);
    view->setField(Id::Y, 2, 5);
    view->setField(Id::Z, 0, 1.5);
    view->setField(Id::Z, 1, 2147483524);
    view->setField(Id::Z, 2, 745.23);
    bufferReader.addView(view);

    Options writerOps;
    writerOps.add("filename", outname);
    writerOps.add("offset_x", "auto");
    writerOps.add("scale_x", "auto");
    writerOps.add("offset_z", "auto");
    writerOps.add("scale_z", "auto");

    LasWriter writer;
    writer.setOptions(writerOps);
    writer.setInput(bufferReader);

    writer.prepare(table);
    writer.execute(table);

    {
        Options readerOps;
        readerOps.add("filename", inname1);
        PointTable readTable;

        LasReader reader;
        reader.setOptions(readerOps);

        reader.prepare(readTable);
        EXPECT_DOUBLE_EQ(74529.00, reader.header().offsetX());
        EXPECT_DOUBLE_EQ(0, reader.header().offsetY());
        EXPECT_DOUBLE_EQ(-123, reader.header().offsetZ());

        EXPECT_NEAR(4.30956e-4, reader.header().scaleX(), 1e-4);
        EXPECT_DOUBLE_EQ(.01, reader.header().scaleY());
        // (max - min) are chosen to yield std::numeric_limits<int>::max();
        EXPECT_NEAR(4.9743e-7, reader.header().scaleZ(), 1e-7);

        PointViewSet viewSet = reader.execute(readTable);
        EXPECT_EQ(viewSet.size(), 1u);
        view = *viewSet.begin();
        EXPECT_EQ(view->size(), 3u);
        EXPECT_NEAR(125000.00, view->getFieldAs<double>(Id::X, 0), .001);
        EXPECT_NEAR(74529.00, view->getFieldAs<double>(Id::X, 1), .001);
        EXPECT_NEAR(1000000.02, view->getFieldAs<double>(Id::X, 2), .0001);
    }

    {
        Options readerOps;
        readerOps.add("filename", inname2);
        PointTable readTable;

        LasReader reader;
        reader.setOptions(readerOps);

        reader.prepare(readTable);
        EXPECT_DOUBLE_EQ(25.0, reader.header().offsetX());
        EXPECT_DOUBLE_EQ(0, reader.header().offsetY());
        EXPECT_DOUBLE_EQ(1.5, reader.header().offsetZ());

        EXPECT_NEAR(2.4876e-4, reader.header().scaleX(), 1e-7);
        EXPECT_DOUBLE_EQ(.01, reader.header().scaleY());
        EXPECT_NEAR(.99999, reader.header().scaleZ(), 1e-5);

        PointViewSet viewSet = reader.execute(readTable);
        EXPECT_EQ(viewSet.size(), 1u);
        view = *viewSet.begin();
        EXPECT_EQ(view->size(), 3u);
        EXPECT_NEAR(25.00, view->getFieldAs<double>(Id::X, 0), .0001);
        EXPECT_NEAR(74529.00, view->getFieldAs<double>(Id::X, 1), .001);
        EXPECT_NEAR(534252.35, view->getFieldAs<double>(Id::X, 2), .0001);
    }

    FileUtils::deleteFile(inname1);
    FileUtils::deleteFile(inname2);
}


TEST(LasWriterTest, extra_dims)
{
    Options readerOps;

    readerOps.add("filename", Support::datapath("las/1.2-with-color.las"));
    LasReader reader;
    reader.setOptions(readerOps);

    Options writerOps;
    writerOps.add("extra_dims", "Red=int32, Blue = int16, Green = int32_t");
    writerOps.add("filename", Support::temppath("simple.las"));
    LasWriter writer;
    writer.setInput(reader);
    writer.setOptions(writerOps);

    PointTable table;
    writer.prepare(table);
    PointViewSet viewSet = writer.execute(table);

    LasTester tester;
    LasHeader *header = tester.header(writer);
    EXPECT_EQ(header->pointLen(), header->basePointLen() + 10);
    PointViewPtr pb = *viewSet.begin();

    uint16_t colors[][3] = {
        { 68, 77, 88 },
        { 92, 100, 110 },
        { 79, 87, 87 },
        { 100, 102, 116 },
        { 162, 114, 145 },
        { 163, 137, 155 },
        { 154, 131, 144 },
        { 104, 111, 126 },
        { 164, 136, 156 },
        { 72, 87, 82 },
        { 117, 117, 136 }
    };

    Options reader2Ops;
    reader2Ops.add("filename", Support::temppath("simple.las"));
    reader2Ops.add("extra_dims", "R1 =int32, B1= int16 ,G1=int32_t");

    LasReader reader2;
    reader2.setOptions(reader2Ops);

    PointTable readTable;
    reader2.prepare(readTable);
    viewSet = reader2.execute(readTable);
    pb = *viewSet.begin();
    Dimension::Id r1 = readTable.layout()->findDim("R1");
    EXPECT_TRUE(r1 != Dimension::Id::Unknown);
    Dimension::Id b1 = readTable.layout()->findDim("B1");
    EXPECT_TRUE(b1 != Dimension::Id::Unknown);
    Dimension::Id g1 = readTable.layout()->findDim("G1");
    EXPECT_TRUE(g1 != Dimension::Id::Unknown);
    EXPECT_EQ(pb->size(), (size_t)1065);
    size_t j = 0;
    for (PointId i = 0; i < pb->size(); i += 100)
    {
        EXPECT_EQ(pb->getFieldAs<int16_t>(r1, i), colors[j][0]);
        EXPECT_EQ(pb->getFieldAs<int16_t>(g1, i), colors[j][1]);
        EXPECT_EQ(pb->getFieldAs<int16_t>(b1, i), colors[j][2]);
        j++;
    }
}

TEST(LasWriterTest, all_extra_dims)
{
    Options readerOps;

    readerOps.add("filename", Support::datapath("bpf/simple-extra.bpf"));
    BpfReader reader;
    reader.setOptions(readerOps);

    FileUtils::deleteFile(Support::temppath("simple.las"));

    Options writerOps;
    writerOps.add("extra_dims", "all");
    writerOps.add("filename", Support::temppath("simple.las"));
    writerOps.add("minor_version", 4);
    LasWriter writer;
    writer.setInput(reader);
    writer.setOptions(writerOps);

    PointTable table;
    writer.prepare(table);
    writer.execute(table);

    Options ops;
    ops.add("filename", Support::temppath("simple.las"));

    LasReader r;
    r.setOptions(ops);

    PointTable t2;
    r.prepare(t2);
    Dimension::Id foo = t2.layout()->findDim("Foo");
    Dimension::Id bar = t2.layout()->findDim("Bar");
    Dimension::Id baz = t2.layout()->findDim("Baz");

    PointViewSet s = r.execute(t2);
    EXPECT_EQ(s.size(), 1u);
    PointViewPtr v = *s.begin();

    // We test for floats instead of doubles because when X, Y and Z
    // get written, they are written scaled, which loses precision.  The
    // foo, bar and baz values are written as full-precision doubles.
    for (PointId i = 0; i < v->size(); ++i)
    {
        using namespace Dimension;

        ASSERT_FLOAT_EQ(v->getFieldAs<float>(Id::X, i),
            v->getFieldAs<float>(foo, i));
        ASSERT_FLOAT_EQ(v->getFieldAs<float>(Id::Y, i),
            v->getFieldAs<float>(bar, i));
        ASSERT_FLOAT_EQ(v->getFieldAs<float>(Id::Z, i),
            v->getFieldAs<float>(baz, i));
    }
}

// Merge a couple of 1.4 LAS files with point formats 1 and 6.  Write the
// output with version 3.  Verify that you get point format 3 (default),
// LAS 1.3 output and other header data forwarded.
TEST(LasWriterTest, forward)
{
    Options readerOps1;

    readerOps1.add("filename", Support::datapath("las/4_1.las"));

    Options readerOps2;

    readerOps2.add("filename", Support::datapath("las/4_6.las"));

    LasReader r1;
    r1.addOptions(readerOps1);

    LasReader r2;
    r2.addOptions(readerOps2);

    StageFactory sf;
    Stage *m = sf.createStage("filters.merge");
    m->setInput(r1);
    m->setInput(r2);

    std::string testfile = Support::temppath("tmp.las");
    FileUtils::deleteFile(testfile);

    Options writerOps;
    writerOps.add("forward", "header");
    writerOps.add("minor_version", 3);
    writerOps.add("filename", testfile);

    LasWriter w;
    w.setInput(*m);
    w.addOptions(writerOps);

    PointTable t;

    w.prepare(t);
    w.execute(t);

    Options readerOps;
    readerOps.add("filename", testfile);

    LasReader r;

    r.setOptions(readerOps);

    PointTable t2;

    r.prepare(t2);
    r.execute(t2);

    MetadataNode n1 = r.getMetadata();
    EXPECT_EQ(n1.findChild("major_version").value<uint8_t>(), 1);
    EXPECT_EQ(n1.findChild("minor_version").value<uint8_t>(), 3);
    EXPECT_EQ(n1.findChild("dataformat_id").value<uint8_t>(), 3);
    EXPECT_EQ(n1.findChild("filesource_id").value<uint8_t>(), 0);
    // Global encoding doesn't match because 4_1.las has a bad value, so we
    // get the default.
    EXPECT_EQ(n1.findChild("global_encoding").value<uint16_t>(), 0);
    EXPECT_EQ(n1.findChild("project_id").value<Uuid>(), Uuid());
    EXPECT_EQ(n1.findChild("system_id").value(), "");
    EXPECT_EQ(n1.findChild("software_id").value(), "TerraScan");
    EXPECT_EQ(n1.findChild("creation_doy").value<uint16_t>(), 142);
    EXPECT_EQ(n1.findChild("creation_year").value<uint16_t>(), 2014);
}

TEST(LasWriterTest, forwardvlr)
{
    Options readerOps1;

    readerOps1.add("filename", Support::datapath("las/lots_of_vlr.las"));
    LasReader r1;
    r1.addOptions(readerOps1);

    std::string testfile = Support::temppath("tmp.las");
    FileUtils::deleteFile(testfile);

    Options writerOps;
    writerOps.add("forward", "vlr");
    writerOps.add("filename", testfile);

    LasWriter w;
    w.setInput(r1);
    w.addOptions(writerOps);

    PointTable t;

    w.prepare(t);
    w.execute(t);

    Options readerOps;
    readerOps.add("filename", testfile);

    LasReader r;

    r.setOptions(readerOps);

    PointTable t2;

    r.prepare(t2);
    r.execute(t2);

    MetadataNode forward = t2.privateMetadata("lasforward");

    auto pred = [](MetadataNode temp)
        { return Utils::startsWith(temp.name(), "vlr_"); };
    MetadataNodeList nodes = forward.findChildren(pred);
    EXPECT_EQ(nodes.size(), 388UL);
}

// Test that data from three input views gets written to separate output files.
TEST(LasWriterTest, flex)
{
    std::array<std::string, 3> outname =
        {{ "test_1.las", "test_2.las", "test_3.las" }};

    Options readerOps;
    readerOps.add("filename", Support::datapath("las/simple.las"));

    PointTable table;

    LasReader reader;
    reader.setOptions(readerOps);

    reader.prepare(table);
    PointViewSet views = reader.execute(table);
    PointViewPtr v = *(views.begin());

    PointViewPtr v1(new PointView(table));
    PointViewPtr v2(new PointView(table));
    PointViewPtr v3(new PointView(table));

    std::vector<PointViewPtr> vs;
    vs.push_back(v1);
    vs.push_back(v2);
    vs.push_back(v3);

    for (PointId i = 0; i < v->size(); ++i)
        vs[i % 3]->appendPoint(*v, i);

    for (size_t i = 0; i < outname.size(); ++i)
        FileUtils::deleteFile(Support::temppath(outname[i]));

    BufferReader reader2;
    reader2.addView(v1);
    reader2.addView(v2);
    reader2.addView(v3);

    Options writerOps;
    writerOps.add("filename", Support::temppath("test_#.las"));

    LasWriter writer;
    writer.setOptions(writerOps);
    writer.setInput(reader2);

    writer.prepare(table);
    writer.execute(table);

    for (size_t i = 0; i < outname.size(); ++i)
    {
        std::string filename = Support::temppath(outname[i]);
        EXPECT_TRUE(FileUtils::fileExists(filename));

        Options ops;
        ops.add("filename", filename);

        LasReader r;
        r.setOptions(ops);
        EXPECT_EQ(r.preview().m_pointCount, 355u);
    }
}

// Test that data from three input views gets written to a single output file.
TEST(LasWriterTest, flex2)
{
    Options readerOps;
    readerOps.add("filename", Support::datapath("las/simple.las"));

    PointTable table;

    LasReader reader;
    reader.setOptions(readerOps);

    reader.prepare(table);
    PointViewSet views = reader.execute(table);
    PointViewPtr v = *(views.begin());

    PointViewPtr v1(new PointView(table));
    PointViewPtr v2(new PointView(table));
    PointViewPtr v3(new PointView(table));

    std::vector<PointViewPtr> vs;
    vs.push_back(v1);
    vs.push_back(v2);
    vs.push_back(v3);

    for (PointId i = 0; i < v->size(); ++i)
        vs[i % 3]->appendPoint(*v, i);

    std::string outfile(Support::temppath("test_flex.las"));
    FileUtils::deleteFile(outfile);

    BufferReader reader2;
    reader2.addView(v1);
    reader2.addView(v2);
    reader2.addView(v3);

    Options writerOps;
    writerOps.add("filename", outfile);

    LasWriter writer;
    writer.setOptions(writerOps);
    writer.setInput(reader2);

    writer.prepare(table);
    writer.execute(table);

    EXPECT_TRUE(FileUtils::fileExists(outfile));

    Options ops;
    ops.add("filename", outfile);

    LasReader r;
    r.setOptions(ops);
    EXPECT_EQ(r.preview().m_pointCount, 1065u);
}

#if defined(PDAL_HAVE_LAZPERF) && defined(PDAL_HAVE_LASZIP)
// LAZ files are normally written in chunks of 50,000, so a file of size
// 110,000 ensures we read some whole chunks and a partial.
TEST(LasWriterTest, lazperf)
{
    Options readerOps;
    readerOps.add("filename", Support::datapath("las/autzen_trim.las"));

    LasReader lazReader;
    lazReader.setOptions(readerOps);

    std::string testfile(Support::temppath("temp.laz"));

    FileUtils::deleteFile(testfile);

    Options writerOps;
    writerOps.add("filename", testfile);
    writerOps.add("compression", "lazperf");

    LasWriter lazWriter;
    lazWriter.setOptions(writerOps);
    lazWriter.setInput(lazReader);

    PointTable t;
    lazWriter.prepare(t);
    lazWriter.execute(t);

    // Now test the points were properly written.  Use laszip.
    Options ops1;
    ops1.add("filename", testfile);

    LasReader r1;
    r1.setOptions(ops1);

    PointTable t1;
    r1.prepare(t1);
    PointViewSet set1 = r1.execute(t1);
    PointViewPtr view1 = *set1.begin();

    Options ops2;
    ops2.add("filename", Support::datapath("las/autzen_trim.las"));

    LasReader r2;
    r2.setOptions(ops2);

    PointTable t2;
    r2.prepare(t2);
    PointViewSet set2 = r2.execute(t2);
    PointViewPtr view2 = *set2.begin();

    EXPECT_EQ(view1->size(), view2->size());
    EXPECT_EQ(view1->size(), (point_count_t)110000);

    DimTypeList dims = view1->dimTypes();
    size_t pointSize = view1->pointSize();
    EXPECT_EQ(view1->pointSize(), view2->pointSize());

   // Validate some point data.
    std::unique_ptr<char> buf1(new char[pointSize]);
    std::unique_ptr<char> buf2(new char[pointSize]);
    for (PointId i = 0; i < view1->pointSize(); i += 100)
    {
       view1->getPackedPoint(dims, i, buf1.get());
       view2->getPackedPoint(dims, i, buf2.get());
       EXPECT_EQ(memcmp(buf1.get(), buf2.get(), pointSize), 0);
    }
}
#endif

#if defined(PDAL_HAVE_LASZIP)
// LAZ files are normally written in chunks of 50,000, so a file of size
// 110,000 ensures we read some whole chunks and a partial.
TEST(LasWriterTest, laszip)
{
    Options readerOps;
    readerOps.add("filename", Support::datapath("las/autzen_trim.las"));

    LasReader lazReader;
    lazReader.setOptions(readerOps);

    std::string testfile(Support::temppath("temp.laz"));

    FileUtils::deleteFile(testfile);

    Options writerOps;
    writerOps.add("filename", testfile);

    LasWriter lazWriter;
    lazWriter.setOptions(writerOps);
    lazWriter.setInput(lazReader);

    PointTable t;
    lazWriter.prepare(t);
    lazWriter.execute(t);

    // Now test the points were properly written.  Use laszip.
    Options ops1;
    ops1.add("filename", testfile);

    LasReader r1;
    r1.setOptions(ops1);

    PointTable t1;
    r1.prepare(t1);
    PointViewSet set1 = r1.execute(t1);
    PointViewPtr view1 = *set1.begin();

    Options ops2;
    ops2.add("filename", Support::datapath("las/autzen_trim.las"));

    LasReader r2;
    r2.setOptions(ops2);

    PointTable t2;
    r2.prepare(t2);
    PointViewSet set2 = r2.execute(t2);
    PointViewPtr view2 = *set2.begin();

    EXPECT_EQ(view1->size(), view2->size());
    EXPECT_EQ(view1->size(), (point_count_t)110000);

    DimTypeList dims = view1->dimTypes();
    size_t pointSize = view1->pointSize();
    EXPECT_EQ(view1->pointSize(), view2->pointSize());

   // Validate some point data.
    std::unique_ptr<char> buf1(new char[pointSize]);
    std::unique_ptr<char> buf2(new char[pointSize]);
    for (PointId i = 0; i < view1->size(); i += 100)
    {
       view1->getPackedPoint(dims, i, buf1.get());
       view2->getPackedPoint(dims, i, buf2.get());
       EXPECT_EQ(memcmp(buf1.get(), buf2.get(), pointSize), 0);
    }
}

// This is the same test as the above, but for a 1.4-specific point format.
// LAZ files are normally written in chunks of 50,000, so a file of size
// 110,000 ensures we read some whole chunks and a partial.
TEST(LasWriterTest, laszip1_4)
{
    Options readerOps;
    std::string baseFilename = Support::datapath("las/autzen_trim_7.las");
    readerOps.add("filename", baseFilename);

    LasReader lazReader;
    lazReader.setOptions(readerOps);

    std::string testfile(Support::temppath("temp.laz"));

    FileUtils::deleteFile(testfile);

    Options writerOps;
    writerOps.add("filename", testfile);
    writerOps.add("dataformat_id", 7);
    writerOps.add("minor_version", 4);

    LasWriter lazWriter;
    lazWriter.setOptions(writerOps);
    lazWriter.setInput(lazReader);

    PointTable t;
    lazWriter.prepare(t);
    lazWriter.execute(t);

    // Now test the points were properly written.  Use laszip.
    Options ops1;
    ops1.add("filename", testfile);

    LasReader r1;
    r1.setOptions(ops1);

    PointTable t1;
    r1.prepare(t1);
    PointViewSet set1 = r1.execute(t1);
    PointViewPtr view1 = *set1.begin();

    Options ops2;
    ops2.add("filename", baseFilename);

    LasReader r2;
    r2.setOptions(ops2);

    PointTable t2;
    r2.prepare(t2);
    PointViewSet set2 = r2.execute(t2);
    PointViewPtr view2 = *set2.begin();

    EXPECT_EQ(view1->size(), view2->size());
    EXPECT_EQ(view1->size(), (point_count_t)110000);

    DimTypeList dims = view1->dimTypes();
    size_t pointSize = view1->pointSize();
    EXPECT_EQ(view1->pointSize(), view2->pointSize());

   // Validate some point data.
    std::unique_ptr<char> buf1(new char[pointSize]);
    std::unique_ptr<char> buf2(new char[pointSize]);
    for (PointId idx = 0; idx < view1->size(); idx++)
    {
       view1->getPackedPoint(dims, idx, buf1.get());
       view2->getPackedPoint(dims, idx, buf2.get());
       char *b1 = buf1.get();
       char *b2 = buf2.get();
       // Uncomment this to figure out the exact byte at which things are
       // broken.
       /**
       for (size_t i = 0; i < pointSize; ++i)
       {
           if (*b1++ != *b2++)
           {
               {
                   size_t s = 0;
                   for (auto di = dims.begin(); di != dims.end(); ++di)
                   {
                       std::cerr << "Dim " << view1->dimName(di->m_id) <<
                           " at " << s << "!\n";
                       s += Dimension::size(di->m_type);
                   }
               }
               throw pdal_error("Mismatch at byte = " + to_string(i) + "!");
           }
       }
       **/
       EXPECT_EQ(memcmp(buf1.get(), buf2.get(), pointSize), 0);
    }
}

TEST(LasWriterTest, flex_vlr)
{
    std::array<std::string, 3> outname =
        {{ "test_1.laz", "test_2.laz", "test_3.laz" }};

    Options readerOps;
    readerOps.add("filename", Support::datapath("las/simple.las"));

    PointTable table;

    LasReader reader;
    reader.setOptions(readerOps);

    reader.prepare(table);
    PointViewSet views = reader.execute(table);
    PointViewPtr v = *(views.begin());

    PointViewPtr v1(new PointView(table));
    PointViewPtr v2(new PointView(table));
    PointViewPtr v3(new PointView(table));

    std::vector<PointViewPtr> vs;
    vs.push_back(v1);
    vs.push_back(v2);
    vs.push_back(v3);

    for (PointId i = 0; i < v->size(); ++i)
        vs[i % 3]->appendPoint(*v, i);

    for (size_t i = 0; i < outname.size(); ++i)
        FileUtils::deleteFile(Support::temppath(outname[i]));

    BufferReader reader2;
    reader2.addView(v1);
    reader2.addView(v2);
    reader2.addView(v3);

    Options writerOps;
    writerOps.add("filename", Support::temppath("test_#.laz"));
    writerOps.add("pdal_metadata", true);

    LasWriter writer;
    writer.setOptions(writerOps);
    writer.setInput(reader2);

    writer.prepare(table);
    writer.execute(table);

    // Make sure that the files have the same three VLRs.
    for (size_t i = 0; i < outname.size(); ++i)
    {
        std::string filename = Support::temppath(outname[i]);
        EXPECT_TRUE(FileUtils::fileExists(filename));

        Options ops;
        ops.add("filename", filename);

        LasReader r;
        r.setOptions(ops);

        PointTable t;
        r.prepare(t);
        r.execute(t);
        const VlrList& vlrs = r.header().vlrs();
        EXPECT_EQ(vlrs.size(), 3U);
        EXPECT_NE(r.header().findVlr("laszip encoded", 22204), nullptr);
        EXPECT_NE(r.header().findVlr("PDAL", 12), nullptr);
        EXPECT_NE(r.header().findVlr("PDAL", 13), nullptr);
    }
}
#endif // PDAL_HAVE_LASZIP

void compareFiles(const std::string& name1, const std::string& name2,
    size_t increment = 100)
{
    Options o1;
    o1.add("filename", name1);

    Options o2;
    o2.add("filename", name2);

    LasReader r1;
    r1.setOptions(o1);

    LasReader r2;
    r2.setOptions(o2);

    PointTable t1;
    r1.prepare(t1);
    PointViewSet s1 = r1.execute(t1);
    EXPECT_EQ(s1.size(), 1u);
    PointViewPtr v1 = *s1.begin();
    DimTypeList d1 = v1->dimTypes();
    size_t size1 = v1->pointSize();
    std::vector<char> buf1(size1);

    PointTable t2;
    r2.prepare(t2);
    PointViewSet s2 = r2.execute(t2);
    EXPECT_EQ(s2.size(), 1u);
    PointViewPtr v2 = *s2.begin();
    DimTypeList d2 = v2->dimTypes();
    size_t size2 = v2->pointSize();
    std::vector<char> buf2(size2);

    EXPECT_EQ(v1->size(), v2->size());
    EXPECT_EQ(d1.size(), d2.size());
    EXPECT_EQ(size1, size2);

    for (PointId i = 0; i < (std::min)(size1, size2); i += increment)
    {
       v1->getPackedPoint(d1, i, buf1.data());
       v2->getPackedPoint(d2, i, buf2.data());
       EXPECT_EQ(memcmp(buf1.data(), buf2.data(), (std::min)(size1, size2)), 0);
    }
}

TEST(LasWriterTest, stream)
{
    std::string infile(Support::datapath("las/autzen_trim.las"));
    std::string outfile(Support::temppath("trimtest.las"));

    FileUtils::deleteFile(outfile);

    Options ops1;
    ops1.add("filename", infile);

    LasReader r;
    r.setOptions(ops1);

    Options ops2;
    ops2.add("filename", outfile);
    ops2.add("forward", "all");
    LasWriter w;
    w.setOptions(ops2);
    w.setInput(r);

    FixedPointTable t(100);
    w.prepare(t);
    w.execute(t);

    compareFiles(infile, outfile);
}

TEST(LasWriterTest, streamhashwrite)
{
    std::string infile(Support::datapath("las/autzen_trim.las"));
    std::string outfile(Support::temppath("trimtest#.las"));

    FileUtils::deleteFile(outfile);

    Options ops1;
    ops1.add("filename", infile);

    LasReader r;
    r.setOptions(ops1);

    Options ops2;
    ops2.add("filename", outfile);
    LasWriter w;
    w.setOptions(ops2);
    w.setInput(r);

    FixedPointTable t(100);
    EXPECT_THROW(w.prepare(t), pdal_error);
}

TEST(LasWriterTest, fix1063_1064_1065)
{
    std::string outfile = Support::temppath("out.las");
    std::string infile = Support::datapath("las/test1_4.las");

    FileUtils::deleteFile(outfile);

    std::string cmd = "pdal translate --writers.las.forward=all "
        "--writers.las.a_srs=\"EPSG:4326\" " + infile + " " + outfile;
    std::string output;
    Utils::run_shell_command(Support::binpath(cmd), output);

    Options o;
    o.add("filename", outfile);

    LasReader r;
    r.setOptions(o);

    PointTable t;
    r.prepare(t);
    PointViewSet s = r.execute(t);
    EXPECT_EQ(s.size(), 1u);
    PointViewPtr v = *s.begin();
    EXPECT_EQ(v->size(), 1000u);

    // https://github.com/PDAL/PDAL/issues/1063
    for (PointId idx = 0; idx < v->size(); ++idx)
        EXPECT_EQ(8, v->getFieldAs<int>(Dimension::Id::ClassFlags, idx));

    // https://github.com/PDAL/PDAL/issues/1064
    MetadataNode m = r.getMetadata();
    m = m.findChild("global_encoding");
    EXPECT_EQ(17, m.value<int>());

    // https://github.com/PDAL/PDAL/issues/1065
    SpatialReference ref = v->spatialReference();
    // This WKT is the leading common bit of WKT1 and WKT2 resolution.  When
    // we're just doing WKT2, this can be improved.
    std::string wkt {
        R"(GEOGCS["WGS 84",DATUM["WGS_1984",SPHEROID["WGS 84",6378137,298.257223563,AUTHORITY["EPSG","7030"]],AUTHORITY["EPSG","6326"]],PRIMEM["Greenwich",0,AUTHORITY["EPSG","8901"]],UNIT["degree",0.0174532925199433,AUTHORITY["EPSG","9122"]])"
    };
    EXPECT_TRUE(Utils::startsWith(ref.getWKT(), wkt));
}

TEST(LasWriterTest, pdal_metadata)
{
    PointTable table;

    std::string infile(Support::datapath("las/1.2-with-color.las"));
    std::string outfile(Support::temppath("simple.las"));

    // remove file from earlier run, if needed
    FileUtils::deleteFile(outfile);

    Options readerOpts;
    readerOpts.add("filename", infile);

    Options writerOpts;
    writerOpts.add("pdal_metadata", true);
    writerOpts.add("filename", outfile);

    LasReader reader;
    reader.setOptions(readerOpts);

    LasWriter writer;
    writer.setOptions(writerOpts);
    writer.setInput(reader);
    writer.prepare(table);
    writer.execute(table);

    PointTable t2;
    Options readerOpts2;
    readerOpts2.add("filename", outfile);
    LasReader reader2;
    reader2.setOptions(readerOpts2);

    reader2.prepare(t2);
    reader2.execute(t2);

    EXPECT_EQ(reader2.getMetadata().children("pdal_metadata").size(), 1UL);
    EXPECT_EQ(reader2.getMetadata().children("pdal_pipeline").size(), 1UL);

}


TEST(LasWriterTest, pdal_add_vlr)
{
    PointTable table;

    std::string infile(Support::datapath("las/1.2-with-color.las"));
    std::string outfile(Support::temppath("simple.las"));

    // remove file from earlier run, if needed
    FileUtils::deleteFile(outfile);

    Options readerOpts;
    readerOpts.add("filename", infile);

    std::string vlr( " [ { \"description\": \"A description under 32 bytes\", \"record_id\": 42, \"user_id\": \"hobu\", \"data\": \"dGhpcyBpcyBzb21lIHRleHQ=\" },  { \"description\": \"A description under 32 bytes\", \"record_id\": 43, \"user_id\": \"hobu\", \"data\": \"dGhpcyBpcyBzb21lIG1vcmUgdGV4dA==\" } ]");

    Options writerOpts;
    writerOpts.add("vlrs", vlr);
    writerOpts.add("filename", outfile);

    LasReader reader;
    reader.setOptions(readerOpts);

    LasWriter writer;
    writer.setOptions(writerOpts);
    writer.setInput(reader);
    writer.prepare(table);
    writer.execute(table);

    PointTable t2;
    Options readerOpts2;
    readerOpts2.add("filename", outfile);
    LasReader reader2;
    reader2.setOptions(readerOpts2);

    reader2.prepare(t2);
    reader2.execute(t2);

    MetadataNode forward = reader2.getMetadata();

    auto pred = [](MetadataNode temp)
        { return Utils::startsWith(temp.name(), "vlr_"); };
    MetadataNodeList nodes = forward.findChildren(pred);
    EXPECT_EQ(nodes.size(), 2UL);
}

TEST(LasWriterTest, evlroffset)
{
    std::string outfile(Support::temppath("evlr.las"));

    FileUtils::deleteFile(outfile);
    {
        LasWriter w;
        Options wo;
        std::vector<uint8_t> largeVlr(66000);
        std::string vlr =
            " [ { \"description\": \"A description under 32 bytes\", "
            "\"record_id\": 42, \"user_id\": \"hobu\", \"data\": \"" +
            Utils::base64_encode(largeVlr) + "\" }]";
        wo.add("vlrs", vlr);
        wo.add("minor_version", 4);
        wo.add("filename", outfile);
        w.setOptions(wo);

        PointTable t;
        w.prepare(t);
        w.execute(t);
        LasTester tester;
        LasHeader* h = tester.header(w);
        // No points in the file
        EXPECT_EQ(h->eVlrOffset(), h->pointOffset());
        EXPECT_EQ(h->eVlrCount(), 1u);
    }
}

// Make sure that we can forward the LAS_Spec/3 VLR
TEST(LasWriterTest, forward_spec_3)
{
    PointTable table;

    std::string infile(Support::datapath("las/spec_3.las"));
    std::string outfile(Support::temppath("out.las"));

    // remove file from earlier run, if needed
    FileUtils::deleteFile(outfile);

    Options readerOpts;
    readerOpts.add("filename", infile);

    Options writerOpts;
    writerOpts.add("forward", "all,vlr");
    writerOpts.add("filename", outfile);

    LasReader reader;
    reader.setOptions(readerOpts);

    LasWriter writer;
    writer.setOptions(writerOpts);
    writer.setInput(reader);

    writer.prepare(table);
    writer.execute(table);

    PointTable t2;
    Options readerOpts2;
    readerOpts2.add("filename", outfile);
    LasReader reader2;
    reader2.setOptions(readerOpts2);

    reader2.prepare(t2);
    reader2.execute(t2);

    auto pred = [](MetadataNode temp)
    {
        auto recPred = [](MetadataNode n)
        {
            return n.name() == "record_id" &&
                n.value() == "3";
        };

        auto userPred = [](MetadataNode n)
        {
            return n.name() == "user_id" &&
                n.value() == "LASF_Spec";
        };

        return Utils::startsWith(temp.name(), "vlr_") &&
            !temp.findChild(recPred).empty() &&
            !temp.findChild(userPred).empty();
    };
    MetadataNode origRoot = reader.getMetadata();
    MetadataNodeList origNodes = origRoot.findChildren(pred);
    EXPECT_EQ(origNodes.size(), 1u);
    MetadataNode origNode = origNodes[0];

    MetadataNode root = reader2.getMetadata();
    MetadataNodeList nodes = root.findChildren(pred);
    EXPECT_EQ(nodes.size(), 1u);
    MetadataNode node = nodes[0];

    // Also test that we're properly forwarding data.
    origNode = origNode.findChild("data");
    node = node.findChild("data");
    EXPECT_EQ(origNode.value().size(), 28u);
    EXPECT_EQ(node.value().size(), origNode.value().size());
    EXPECT_EQ(node.value(), origNode.value());
}

TEST(LasWriterTest, oversize_vlr)
{
    LasWriter w;
    Options o;

    o.add("filename", "out.las");
    w.addOptions(o);

    PointTable t;

    w.prepare(t);

    std::vector<uint8_t> data(100000, 32);
    LasTester tester;
    EXPECT_THROW(
        tester.addVlr(w, "USER ID", 555, "This is a description", data),
        pdal_error);
}


// Test auto scale/offset for streaming mode.
TEST(LasWriterTest, issue1940)
{
    StageFactory f;

    Stage& r = *(f.createStage("readers.faux"));
    Options ro;
    ro.add("mode", "constant");
    ro.add("bounds", "([55,55],[55,55],[55,55])");
    ro.add("count", 20);
    r.addOptions(ro);

    LasWriter w;
    Options wo;
    //LogPtr log(new Log("TEST", &std::clog));
    //log->setLevel((LogLevel)5);
    //w.setLog(log);
    wo.add("filename", Support::temppath("out.las"));
    wo.add("scale_x", "auto");
    wo.add("offset_y", "auto");
    w.addOptions(wo);
    w.setInput(r);

    FixedPointTable t(100);
    w.prepare(t);
    w.execute(t);

    LasTester tester;
    LasHeader *h = tester.header(w);
    EXPECT_DOUBLE_EQ(h->offsetX(), 0);
    EXPECT_DOUBLE_EQ(h->offsetY(), 55);
    EXPECT_DOUBLE_EQ(h->scaleX(), 1.0);
    EXPECT_DOUBLE_EQ(h->scaleY(), .01);
}

// Make sure that we can forward scale from multiple files if they match.
TEST(LasWRiterTest, issue2663)
{
    std::string outfile(Support::temppath("out.las"));
    {
        LasReader r1;
        Options ro1;
        ro1.add("filename", Support::datapath("las/prec3.las"));
        r1.setOptions(ro1);

        LasReader r2;
        Options ro2;
        ro2.add("filename", Support::datapath("las/prec3.las"));
        r2.setOptions(ro2);

        LasWriter w;
        Options wo;
        wo.add("filename", outfile);
        wo.add("forward", "all");
        w.setOptions(wo);
        w.setInput(r1);
        w.setInput(r2);

        PointTable t;
        w.prepare(t);
        w.execute(t);
    }

    LasReader r;
    Options ro;
    ro.add("filename", outfile);
    r.setOptions(ro);

    PointTable t;
    r.prepare(t);

    const LasHeader& h = r.header();
    EXPECT_EQ(h.scaleX(), .001);
    EXPECT_EQ(h.scaleY(), .001);
    EXPECT_EQ(h.scaleZ(), .001);
}

#if defined(PDAL_HAVE_LASZIP)
// Make sure that we can translate this special test data to 1.4, dataformat 6.
TEST(LasWriterTest, issue2320)
{
    std::string outfile(Support::temppath("2320.laz"));

    FileUtils::deleteFile(outfile);
    {
        LasReader r;
        Options ro;
        ro.add("filename", Support::datapath("las/wontcompress3.las"));
        r.setOptions(ro);

        LasWriter w;
        Options wo;
        wo.add("minor_version", 4);
        wo.add("dataformat_id", 6);
        wo.add("filename", outfile);
        w.setOptions(wo);
        w.setInput(r);

        PointTable t;
        w.prepare(t);
        w.execute(t);
    }

    // Check that we can read.
    {
        LasReader r;
        Options ro;
        ro.add("filename", outfile);
        r.setOptions(ro);

        PointTable t;
        r.prepare(t);
        PointViewSet s = r.execute(t);
        EXPECT_EQ(s.size(), 1U);
        PointViewPtr v = *s.begin();
        EXPECT_EQ(v->size(), 1000U);
    }
}
#endif

