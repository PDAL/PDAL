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

#include "gtest/gtest.h"

#include <stdlib.h>

#include <pdal/util/FileUtils.hpp>
#include <LasHeader.hpp>
#include <LasReader.hpp>
#include <LasWriter.hpp>

#include <pdal/PointBuffer.hpp>

#include "StageTester.hpp"
#include "Support.hpp"

namespace pdal
{

//ABELL - Should probably be moved to its own file.
class LasTester
{
public:
    template <typename T>
    static T headerVal(std::shared_ptr<LasWriter> w, const std::string& s)
        { return w->headerVal<T>(s); }
    LasHeader *header(std::shared_ptr<LasWriter> w)
        { return &w->m_lasHeader; }
};

} // namespace pdal

using namespace pdal;

TEST(LasWriterTest, auto_offset)
{
    using namespace Dimension;

    const std::string FILENAME(Support::temppath("offset_test.las"));
    PointContext ctx;

    ctx.registerDim(Id::X);

    PointBufferPtr buf(new PointBuffer(ctx));
    buf->setField(Id::X, 0, 125000.00);
    buf->setField(Id::X, 1, 74529.00);
    buf->setField(Id::X, 2, 523523.02);

    Options writerOps;
    writerOps.add("filename", FILENAME);
    writerOps.add("offset_x", "auto");

    std::shared_ptr<LasWriter> writer(new LasWriter);
    writer->setOptions(writerOps);

    writer->prepare(ctx);

    WriterTester::ready(writer, ctx);
    WriterTester::write(writer, *buf);
    WriterTester::done(writer, ctx);

    Options readerOps;
    readerOps.add("filename", FILENAME);

    PointContext readCtx;

    std::shared_ptr<LasReader> reader(new LasReader);
    reader->setOptions(readerOps);

    reader->prepare(readCtx);
    EXPECT_FLOAT_EQ(reader->header().offsetX(), 74529.00);
    PointBufferSet pbSet = reader->execute(readCtx);
    EXPECT_EQ(pbSet.size(), 1u);
    buf = *pbSet.begin();
    EXPECT_EQ(buf->size(), 3u);
    EXPECT_FLOAT_EQ(buf->getFieldAs<double>(Id::X, 0), 125000.00);
    EXPECT_FLOAT_EQ(buf->getFieldAs<double>(Id::X, 1), 74529.00);
    EXPECT_FLOAT_EQ(buf->getFieldAs<double>(Id::X, 2), 523523.02);
    FileUtils::deleteFile(FILENAME);
}

TEST(LasWriterTest, extra_dims)
{
    Options readerOps;

    readerOps.add("filename", Support::datapath("las/1.2-with-color.las"));
    std::shared_ptr<LasReader> reader(new LasReader);
    reader->setOptions(readerOps);

    Options writerOps;
    writerOps.add("extra_dims", "Red=int32, Blue = int16, Green = int32_t");
    writerOps.add("filename", Support::temppath("simple.las"));
    std::shared_ptr<LasWriter> writer(new LasWriter);
    writer->setInput(reader);
    writer->setOptions(writerOps);

    PointContext ctx;
    writer->prepare(ctx);
    PointBufferSet pbSet = writer->execute(ctx);

    LasTester tester;
    LasHeader *header = tester.header(writer);
    EXPECT_EQ(header->pointLen(), header->basePointLen() + 10);
    PointBufferPtr pb = *pbSet.begin();

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

    PointContext ctx2;
    reader2->prepare(ctx2);
    pbSet = reader2->execute(ctx2);
    pb = *pbSet.begin();
    Dimension::Id::Enum r1 = ctx2.findDim("R1");
    EXPECT_TRUE(r1 != Dimension::Id::Unknown);
    Dimension::Id::Enum b1 = ctx2.findDim("B1");
    EXPECT_TRUE(b1 != Dimension::Id::Unknown);
    Dimension::Id::Enum g1 = ctx2.findDim("G1");
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

TEST(LasWriterTest, metadata_options)
{
    Options ops;

    Option metadataOp("metadata", "");

    Options metadataOps;
    metadataOps.add("format", 4);
    metadataOps.add("software_id", "MySoftwareId");
    metadataOps.add("system_id", "FORWARD");
    metadataOps.add("minor_version", "forward");
    metadataOp.setOptions(metadataOps);
    ops.add(metadataOp);

    std::shared_ptr<LasWriter> writer(new LasWriter);
    writer->setOptions(ops);

    PointContext ctx;
    writer->prepare(ctx);

    MetadataNode m = writer->getMetadata();
    m.add("minor_version", 56);

    uint8_t format = 
        (uint8_t)LasTester::headerVal<unsigned>(writer, "format");
    EXPECT_EQ(format, 4u);
    std::string softwareId =
        LasTester::headerVal<std::string>(writer, "software_id");
    EXPECT_EQ(softwareId, "MySoftwareId");
    std::string systemId =
        LasTester::headerVal<std::string>(writer, "system_id");

    // Since the option specifies forward and there is not associated
    // metadata, the value should be the default.
    LasHeader header;
    EXPECT_EQ(systemId, header.getSystemIdentifier());

    // In this case, we should have metadata to override the default.
    uint8_t minorVersion =
        (uint8_t)LasTester::headerVal<unsigned>(writer, "minor_version");
    EXPECT_EQ(minorVersion, 56u);
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
    PointContext ctx;

    std::string infile(Support::datapath("las/1.2-with-color.las"));
    std::string outfile(Support::temppath("simple.las"));

    // remove file from earlier run, if needed
    FileUtils::deleteFile(outfile);

    Options readerOpts;
    readerOpts.add("filename", infile);

    Options writerOpts;
    writerOpts.add("creation_year", 2014);
    writerOpts.add("filename", outfile);

    std::shared_ptr<LasReader> reader(new LasReader);
    reader->setOptions(readerOpts);

    std::shared_ptr<LasWriter> writer(new LasWriter);
    writer->setOptions(writerOpts);
    writer->setInput(&reader);
    writer->prepare(ctx);
    writer->execute(ctx);

    diffdump(infile, outfile);
}
**/

//ABELL
/**
TEST(LasWriterTest, LasWriterTest_test_simple_laz)
{
    PointContext ctx;

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

    writer->prepare(ctx);
    writer->execute(ctx);

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
    PointContext ctx;

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

    writer->prepare(ctx);
    writer->execute(ctx);

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


//ABELL
/**
TEST(LasWriterTest, test_summary_data_add_point)
{
    SummaryData summaryData;

    summaryData.addPoint(-95.329381929535259, 29.71948951835612,
        -17.515486778166398, 0);
    BOX3D b = summaryData.getBounds();
    EXPECT_EQ(b.minx, b.maxx);
    EXPECT_EQ(b.minz, b.maxz);
}
**/


//ABELL
/**
TEST(LasWriterTest, LasWriterTest_test_drop_extra_returns)
{
    using namespace pdal;

    PointContext ctx;

    // remove file from earlier run, if needed
    std::string temp_filename("temp-LasWriterTest_test_drop_extra_returns.las");
    FileUtils::deleteFile(Support::temppath(temp_filename));

    Options ops;

    BOX3D bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
    ops.add("bounds", bounds);
    ops.add("num_points", 100);
    ops.add("mode", "constant");
    ops.add("number_of_returns", 10);
    std::shared_ptr<FauxReader> reader(new FauxReader);
    reader->setOptions(ops);

    std::ostream* ofs = FileUtils::createFile(Support::temppath(temp_filename));

    Options writerOptions;
    writerOptions.add("discard_high_return_numbers", true);
    writerOptions.add("compression", false);
    writerOptions.add("creation_year", 0);
    writerOptions.add("creation_doy", 0);
    writerOptions.add("system_id", "");
    writerOptions.add("software_id", "TerraScan");

    std::shared_ptr<LasWriter> writer(new LasWriter)(ofs);
    writer->setOptions(writerOptions);
    writer->setInput(&reader);
    writer->prepare(ctx);
    writer->execute(ctx);

    Options readerOptions;
    readerOptions.add("filename", Support::temppath(temp_filename));
    readerOptions.add("count", 6);

    std::shared_ptr<LasReader> reader2(new LasReader);
    reader2->setOptions(readerOptions);

    PointContext ctx2;

    reader2->prepare(ctx2);
    PointBufferSet pbSet = reader2->execute(ctx2);
    EXPECT_EQ(pbSet.size(), 1u);
    PointBufferPtr buf = *pbSet.begin();

    uint8_t r1 = buf->getFieldAs<uint8_t>(Dimension::Id::ReturnNumber, 0);
    EXPECT_EQ(r1, 1u);
    uint8_t r2 = buf->getFieldAs<uint8_t>(Dimension::Id::ReturnNumber, 5);
    EXPECT_EQ(r2, 1u);
    uint8_t n1 = buf->getFieldAs<uint8_t>(Dimension::Id::NumberOfReturns, 0);
    EXPECT_EQ(n1, 5u);
    uint8_t n2 = buf->getFieldAs<uint8_t>(Dimension::Id::NumberOfReturns, 5);
    EXPECT_EQ(n1, 5u);

    FileUtils::closeFile(ofs);
    FileUtils::deleteFile(Support::temppath(temp_filename));
}
**/
