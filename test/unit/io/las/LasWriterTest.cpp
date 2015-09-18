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

#include <pdal/util/FileUtils.hpp>
#include <pdal/BufferReader.hpp>
#include <LasHeader.hpp>
#include <LasReader.hpp>
#include <LasWriter.hpp>

#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/StageWrapper.hpp>

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
};

} // namespace pdal

using namespace pdal;

#ifdef PDAL_HAVE_LIBGEOTIFF
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
#endif


#ifdef PDAL_HAVE_LIBGEOTIFF
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
#endif


TEST(LasWriterTest, auto_offset)
{
    using namespace Dimension;

    const std::string FILENAME(Support::temppath("offset_test.las"));
    PointTable table;

    table.layout()->registerDim(Id::X);

    PointViewPtr view(new PointView(table));
    view->setField(Id::X, 0, 125000.00);
    view->setField(Id::X, 1, 74529.00);
    view->setField(Id::X, 2, 523523.02);

    Options writerOps;
    writerOps.add("filename", FILENAME);
    writerOps.add("offset_x", "auto");
    writerOps.add("scale_x", "auto");

    LasWriter writer;
    writer.setOptions(writerOps);

    writer.prepare(table);

    WriterWrapper::ready(writer, table);
    WriterWrapper::write(writer, view);
    WriterWrapper::done(writer, table);

    Options readerOps;
    readerOps.add("filename", FILENAME);

    PointTable readTable;

    LasReader reader;
    reader.setOptions(readerOps);

    reader.prepare(readTable);
    EXPECT_DOUBLE_EQ(74529.00, reader.header().offsetX());
    PointViewSet viewSet = reader.execute(readTable);
    EXPECT_EQ(viewSet.size(), 1u);
    view = *viewSet.begin();
    EXPECT_EQ(view->size(), 3u);
    EXPECT_NEAR(125000.00, view->getFieldAs<double>(Id::X, 0), .0001);
    EXPECT_NEAR(74529.00, view->getFieldAs<double>(Id::X, 1), .0001);
    EXPECT_NEAR(523523.02, view->getFieldAs<double>(Id::X, 2), .0001);
    FileUtils::deleteFile(FILENAME);
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
    std::shared_ptr<LasReader> reader2(new LasReader);
    reader2->setOptions(reader2Ops);

    PointTable readTable;
    reader2->prepare(readTable);
    viewSet = reader2->execute(readTable);
    pb = *viewSet.begin();
    Dimension::Id::Enum r1 = readTable.layout()->findDim("R1");
    EXPECT_TRUE(r1 != Dimension::Id::Unknown);
    Dimension::Id::Enum b1 = readTable.layout()->findDim("B1");
    EXPECT_TRUE(b1 != Dimension::Id::Unknown);
    Dimension::Id::Enum g1 = readTable.layout()->findDim("G1");
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
    EXPECT_EQ(n1.findChild("global_encoding").value<uint8_t>(), 0);
    EXPECT_EQ(n1.findChild("project_id").value<boost::uuids::uuid>(),
        boost::uuids::uuid());
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

/**
namespace
{

bool diffdump(const std::string& f1, const std::string& f2)
{
    auto dump = [](const std::string& temp, const std::string& in)
    {
        std::stringstream ss;
        ss << "lasdump -o " << temp << " " << in;
        system(ss.str().c_str());
    };

    std::string t1 = Support::temppath("lasdump1.tmp");
    std::string t2 = Support::temppath("lasdump2.tmp");

    dump(t1, f1);
    dump(t2, f2);

    std::string diffFile = Support::temppath("dumpdiff.tmp");
    std::stringstream ss;
    ss << "diff " << t1 << " " << t2 << " > " << diffFile;
    system(ss.str().c_str());

    return true;
}

} // Unnamed namespace

TEST(LasWriterTest, simple)
{
    PointTable table;

    std::string infile(Support::datapath("las/1.2-with-color.las"));
    std::string outfile(Support::temppath("simple.las"));

    // remove file from earlier run, if needed
    FileUtils::deleteFile(outfile);

    Options readerOpts;
    readerOpts.add("filename", infile);

    Options writerOpts;
    writerOpts.add("creation_year", 2014);
    writerOpts.add("filename", outfile);

    LasReader reader;
    reader.setOptions(readerOpts);

    LasWriter writer;
    writer.setOptions(writerOpts);
    writer.setInput(reader);
    writer.prepare(table);
    writer.execute(table);

    diffdump(infile, outfile);
}
**/

//ABELL
/**
TEST(LasWriterTest, LasWriterTest_test_simple_laz)
{
    PointTable table;

    WriterOpts writerOpts;
    writerOpts.add("compressed", true);
    writerOpts.add("creation_year", 0);
    writerOpts.add("creation_doy", 0);
    writerOpts.add("system_id", "");
    writerOpts.add("software_id", "TerraScan");


    // remove file from earlier run, if needed
    FileUtils::deleteFile("laszip/LasWriterTest_test_simple_laz.laz");

    LasReader reader(Support::datapath("laszip/basefile.las"));

    std::ostream* ofs = FileUtils::createFile(
        Support::temppath("LasWriterTest_test_simple_laz.laz"));

    // need to scope the writer, so that's it dtor can use the stream
    std::shared_ptr<LasWriter> writer(new LasWriter)(ofs);
    writer->setOptions(writer);
    writer->setInput(&reader);

    writer->prepare(table);
    writer->execute(table);

    FileUtils::closeFile(ofs);

    {
        LasReader reader(
            Support::temppath("LasWriterTest_test_simple_laz.laz"));
    }

    // these two files only differ by the description string in the VLR.
    // This now skips the entire LASzip VLR for comparison.
    const uint32_t numdiffs =Support::diff_files(
        Support::temppath("LasWriterTest_test_simple_laz.laz"),
        Support::datapath("laszip/laszip-generated.laz"),
        227, 106);
    EXPECT_EQ(numdiffs, 0u);

    if (numdiffs == 0)
        FileUtils::deleteFile(
            Support::temppath("LasWriterTest_test_simple_laz.laz"));
}
**/

//ABELL
/**
static void test_a_format(const std::string& refFile, uint8_t majorVersion,
    uint8_t minorVersion, int pointFormat)
{
    PointTable table;

    // remove file from earlier run, if needed
    FileUtils::deleteFile("temp.las");

    std::string directory = "las/permutations/";
    Options readerOpts;
    readerOpts.add("filename", Support::datapath(directory + "1.2_3.las"));

    std::shared_ptr<LasReader> reader(new LasReader);
    reader->setOptions(readerOpts);

    Options writerOpts;
    writerOpts.add("compression", false);
    writerOpts.add("creation_doy", 78);
    writerOpts.add("creation_year", 2008);
    writerOpts.add("format", pointFormat);
    writerOpts.add("minor_version", (unsigned)minorVersion);
    writerOpts.add("system_id", "libLAS");
    writerOpts.add("software_id", "libLAS 1.2");
    writerOpts.add<boost::uuids::uuuid>("project_id",
        "8388f1b8-aa1b-4108-bca3-6bc68e7b062e");
//    writerOpts.add("project_id", boost::lexical_cast<boost::uuids::uuid>(
//        "8388f1b8-aa1b-4108-bca3-6bc68e7b062e");

    std::ostream* ofs = FileUtils::createFile(Support::temppath("temp.las"));

    // need to scope the writer, so that's it dtor can use the stream
    std::shared_ptr<LasWriter> writer(new LasWriter)(ofs);
    writer->setOptions(writerOpts);
    writer->setInput(&reader);

    writer->prepare(table);
    writer->execute(table);

    bool filesSame = Support::compare_files("temp.las",
        Support::datapath(directory + refFile));
    EXPECT_TRUE(filesSame);
    if (filesSame)
        FileUtils::deleteFile(Support::temppath("temp.las"));
    else
        exit(0);
}
**/


//ABELL
/**
TEST(LasWriterTest, version1_0)
{
    test_a_format("1.0_0.las", 1, 0, 0);
    test_a_format("1.0_1.las", 1, 0, 1);
}


TEST(LasWriterTest, version1_1)
{
    test_a_format("1.1_0.las", 1, 1, 0);
    test_a_format("1.1_1.las", 1, 1, 1);
}


TEST(LasWriterTest, version1_2)
{
    test_a_format("1.2_0.las", 1, 2, 0);
    test_a_format("1.2_1.las", 1, 2, 1);
    test_a_format("1.2_2.las", 1, 2, 2);
    test_a_format("1.2_3.las", 1, 2, 3);
}
**/

