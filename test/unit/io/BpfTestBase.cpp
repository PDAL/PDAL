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

#include "Support.hpp"
#include "io/BpfSupport.hpp"

using namespace pdal;

TEST(BpfTestBase, test_point_major)
{
    test_file_type(
        Support::datapath("bpf/autzen-utm-chipped-25-v3-interleaved.bpf"));
}

TEST(BpfTestBase, test_dim_major)
{
    test_file_type(
        Support::datapath("bpf/autzen-utm-chipped-25-v3.bpf"));
}

TEST(BpfTestBase, test_byte_major)
{
    test_file_type(
        Support::datapath("bpf/autzen-utm-chipped-25-v3-segregated.bpf"));
}

TEST(BpfTestBase, roundtrip_byte)
{
    Options ops;

    ops.add("format", "BYTE");
    test_roundtrip(ops);
}

TEST(BpfTestBase, roundtrip_dimension)
{
    Options ops;

    ops.add("format", "DIMENSION");
    test_roundtrip(ops);
}

TEST(BpfTestBase, roundtrip_point)
{
    Options ops;

    ops.add("format", "POINT");
    test_roundtrip(ops);
}

TEST(BpfTestBase, roundtrip_scaling)
{
    Options ops;

    ops.add("format", "POINT");
    ops.add("offset_x", 494000.0);
    ops.add("offset_y", 487000.0);
    ops.add("offset_z", 130.0);
    ops.add("scale_x", .001);
    ops.add("scale_y", .01);
    ops.add("scale_z", 10.0);
    test_roundtrip(ops);
}

TEST(BpfTestBase, extra_bytes)
{
    std::string infile(
        Support::datapath("bpf/autzen-utm-chipped-25-v3-interleaved.bpf"));
    std::string outfile(Support::temppath("tmp.bpf"));

    PointTable table;

    Options readerOps;
    readerOps.add("filename", infile);
    BpfReader reader;
    reader.setOptions(readerOps);

    const unsigned char buf[] = "This is a test.";

    Options writerOps;
    writerOps.add("filename", outfile);
    writerOps.add("header_data", Utils::base64_encode(buf, sizeof(buf)));
    BpfWriter writer;
    writer.setOptions(writerOps);
    writer.setInput(reader);

    FileUtils::deleteFile(outfile);
    writer.prepare(table);
    writer.execute(table);

    test_file_type(outfile);

    Options readerOps2;
    readerOps2.add("filename", outfile);

    PointTable table2;
    BpfReader reader2;
    reader2.setOptions(readerOps2);
    reader2.prepare(table2);
    reader2.execute(table2);
    MetadataNode n = reader2.getMetadata();
    std::vector<uint8_t> outbuf =
        Utils::base64_decode(n.findChild("header_data").value());
    EXPECT_EQ(memcmp(outbuf.data(), buf, sizeof(buf)), 0);
}

TEST(BpfTestBase, bundled)
{
    std::string infile(
        Support::datapath("bpf/autzen-utm-chipped-25-v3-interleaved.bpf"));
    std::string outfile(Support::temppath("tmp.bpf"));

    PointTable table;

    Options readerOps;
    readerOps.add("filename", infile);
    BpfReader reader;
    reader.setOptions(readerOps);

    Options writerOps;
    writerOps.add("filename", outfile);
    writerOps.add("bundledfile", Support::datapath("bpf/bundle1"));
    writerOps.add("bundledfile", Support::datapath("bpf/bundle2"));

    BpfWriter writer;
    writer.setOptions(writerOps);
    writer.setInput(reader);

    FileUtils::deleteFile(outfile);
    writer.prepare(table);
    writer.execute(table);

    test_file_type(outfile);

    Options readerOps2;
    readerOps2.add("filename", outfile);

    PointTable table2;
    BpfReader reader2;
    reader2.setOptions(readerOps2);
    reader2.prepare(table2);
    reader2.execute(table2);
    MetadataNode n = reader2.getMetadata();
    std::vector<uint8_t> outbuf;
    auto findbundle = [](MetadataNode& m)
        { return m.name() == "bundled_file"; };
    MetadataNodeList nodes = n.findChildren(findbundle);
    EXPECT_EQ(nodes.size(), 2u);
    auto findbundle1 = [](const MetadataNode& m)
        { return m.name() == "bundle1"; };
    outbuf = Utils::base64_decode(n.find(findbundle1).value());
    EXPECT_EQ(memcmp(outbuf.data(), "This is a test",
        outbuf.size() - 1), 0);
    auto findbundle2 = [](const MetadataNode& m)
        { return m.name() == "bundle2"; };
    outbuf = Utils::base64_decode(n.find(findbundle2).value());
    EXPECT_EQ(memcmp(outbuf.data(), "This is another test",
        outbuf.size() - 1), 0);
}

TEST(BpfTestBase, inspect)
{
    Options ops;
    ops.add("filename", Support::datapath("bpf/autzen-dd.bpf"));

    BpfReader reader;
    reader.setOptions(ops);

    QuickInfo qi = reader.preview();

    std::string testWkt = "PROJCS[\"WGS 84 / UTM zone 1N\",GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]],PROJECTION[\"Transverse_Mercator\"],PARAMETER[\"latitude_of_origin\",0],PARAMETER[\"central_meridian\",-177],PARAMETER[\"scale_factor\",0.9996],PARAMETER[\"false_easting\",500000],PARAMETER[\"false_northing\",0],UNIT[\"metre\",1,AUTHORITY[\"EPSG\",\"9001\"]],AXIS[\"Easting\",EAST],AXIS[\"Northing\",NORTH],AUTHORITY[\"EPSG\",\"32601\"]]";
    EXPECT_EQ(qi.m_srs.getWKT(), testWkt);

    EXPECT_EQ(qi.m_pointCount, 1065u);

    BOX3D bounds(
        -13676090.610841721296, 4894836.9556098170578, 123.93000030517578125,
        -13674705.011110275984, 4896224.6888861842453, 178.7299957275390625);
    EXPECT_EQ(qi.m_bounds, bounds);

    const char *dims[] =
    {
        "Blue",
        "Classification",
        "GPSTime",
        "Green",
        "Intensity",
        "Number_of_Returns",
        "Red",
        "Return_Information",
        "Return_Number",
        "X",
        "Y",
        "Z"
    };

    std::sort(qi.m_dimNames.begin(), qi.m_dimNames.end());
    EXPECT_TRUE(CheckEqualCollections(qi.m_dimNames.begin(),
        qi.m_dimNames.end(), std::begin(dims)));
}

TEST(BpfTestBase, mueller)
{
    BpfMuellerMatrix xform;

    double x = 12.345;
    double y = 45.345;
    double z = 999.341;

    double xp = x;
    double yp = y;
    double zp = z;
    xform.apply(xp, yp, zp);
    EXPECT_DOUBLE_EQ(x, xp);
    EXPECT_DOUBLE_EQ(y, yp);
    EXPECT_DOUBLE_EQ(z, zp);

    // Test translation.
    xp = 1.0;
    yp = 1.0;
    zp = 1.0;

    BpfMuellerMatrix translate;

    translate.m_vals[3] = 2.0;
    translate.m_vals[7] = 2.0;
    translate.m_vals[11] = 1.0;
    translate.apply(xp, yp, zp);
    EXPECT_DOUBLE_EQ(xp, 3.0);
    EXPECT_DOUBLE_EQ(yp, 3.0);
    EXPECT_DOUBLE_EQ(zp, 2.0);

    BpfMuellerMatrix scale;
    xp = 1.0;
    yp = 1.0;
    zp = 1.0;
    scale.m_vals[0] = 2.0;
    scale.m_vals[5] = 7.0;
    scale.m_vals[10] = -3.0;
    scale.apply(xp, yp, zp);
    EXPECT_DOUBLE_EQ(xp, 2.0);
    EXPECT_DOUBLE_EQ(yp, 7.0);
    EXPECT_DOUBLE_EQ(zp, -3.0);
}


TEST(BpfTestBase, flex)
{
    std::array<std::string, 3> outname =
        {{ "test_1.bpf", "test_2.bpf", "test_3.bpf" }};

    Options readerOps;
    readerOps.add("filename",
        Support::datapath("bpf/autzen-utm-chipped-25-v3.bpf"));

    PointTable table;

    BpfReader reader;
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
    writerOps.add("filename", Support::temppath("test_#.bpf"));

    BpfWriter writer;
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

        BpfReader r;
        r.setOptions(ops);
        EXPECT_EQ(r.preview().m_pointCount, 355u);
    }
}

TEST(BpfTestBase, flex2)
{
    Options readerOps;
    readerOps.add("filename",
        Support::datapath("bpf/autzen-utm-chipped-25-v3.bpf"));

    PointTable table;

    BpfReader reader;
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

    std::string outfile(Support::temppath("test_flex.bpf"));
    FileUtils::deleteFile(outfile);

    BufferReader reader2;
    reader2.addView(v1);
    reader2.addView(v2);
    reader2.addView(v3);

    Options writerOps;
    writerOps.add("filename", outfile);

    BpfWriter writer;
    writer.setOptions(writerOps);
    writer.setInput(reader2);

    writer.prepare(table);
    writer.execute(table);

    EXPECT_TRUE(FileUtils::fileExists(outfile));

    Options ops;
    ops.add("filename", outfile);

    BpfReader r;
    r.setOptions(ops);
    EXPECT_EQ(r.preview().m_pointCount, 1065u);
}

TEST(BpfTestBase, outputdims)
{
    Options ops;
    ops.add("filename", Support::datapath("bpf/autzen-dd.bpf"));

    BpfReader reader;
    reader.setOptions(ops);

    std::string testfile(Support::temppath("test.bpf"));

    FileUtils::deleteFile(testfile);
    Options writerOps;
    writerOps.add("filename", testfile);
    writerOps.add("output_dims", "X, Y, Z, Red, Green");

    BpfWriter writer;
    writer.setInput(reader);
    writer.setOptions(writerOps);

    PointTable t;
    writer.prepare(t);
    writer.execute(t);

    Options o2;
    o2.add("filename", testfile);

    BpfReader r;
    r.setOptions(o2);

    PointTable t2;
    r.prepare(t2);

    StringList dimNames;
    Dimension::IdList dimList = t2.layout()->dims();
    EXPECT_EQ(dimList.size(), 5u);
    for (auto di : dimList)
        dimNames.push_back(t2.layout()->dimName(di));

    const char *dims[] =
    {
        "Green",
        "Red",
        "X",
        "Y",
        "Z"
    };

    std::sort(dimNames.begin(), dimNames.end());
    EXPECT_TRUE(CheckEqualCollections(dimNames.begin(),
        dimNames.end(), std::begin(dims)));

    Options o3;
    o3.add("filename", testfile);
    o3.add("output_dims", "Y, Z, Red, Green");

    BpfWriter w3;
    w3.setOptions(o3);

    // Missing X dimension.
    PointTable t3;
    EXPECT_THROW(w3.prepare(t3), pdal_error);

}

TEST(BpfTestBase, autoutm)
{
    std::string sourcefile(
        Support::datapath("bpf/autzen-utm-chipped-25-v3.bpf"));
    std::string outfile(Support::temppath("tmp.bpf"));

    BpfWriter writer;

    // Make sure that we throw on invalid coord_id string.
    Options o1;
    o1.add("filename", outfile);
    o1.add("coord_id", "foobar");
    writer.setOptions(o1);

    PointTable table;

    EXPECT_THROW(writer.prepare(table), pdal_error);

    // Make sure that we throw on invalid coord_id valid (not a UTM zone).
    Options o2;
    o2.add("filename", outfile);
    o2.add("coord_id", "75");
    writer.setOptions(o2);

    EXPECT_THROW(writer.prepare(table), pdal_error);

    Options readerOps;
    readerOps.add("filename",sourcefile);

    BpfReader reader;
    reader.setOptions(readerOps);

    Options writerOps;
    writerOps.add("filename", outfile);
    writerOps.add("coord_id", "auto");

    writer.setOptions(writerOps);
    writer.setInput(reader);

    FileUtils::deleteFile(outfile);
    writer.prepare(table);
    writer.execute(table);

    Options readerOps2;
    readerOps2.replace("filename",outfile);
    BpfReader reader2;
    reader2.setOptions(readerOps2);

    PointTable table2;
    reader2.prepare(table2);
    PointViewSet views = reader2.execute(table2);
    SpatialReference srs = reader2.getSpatialReference();
    EXPECT_EQ(srs.getUTMZone(), 10);
}
