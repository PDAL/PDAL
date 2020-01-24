/******************************************************************************
 * Copyright (c) 2019, Helix Re Inc.
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
 *     * Neither the name of Helix Re Inc. nor the
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

#include "Support.hpp"
#include <pdal/pdal_test_main.hpp>
#include <stdio.h>

#include "io/LasReader.hpp"
#include "plugins/e57/io/E57Reader.hpp"
#include "plugins/e57/io/E57Writer.hpp"
#include "plugins/e57/io/Utils.hpp"

namespace pdal
{

TEST(E57Writer, testCtr)
{
    Options ops;
    std::string outfile = Support::datapath("e57/test.e57");
    ops.add("filename", outfile);
    E57Writer writer;
    writer.setOptions(ops);
    PointTable table;
    writer.prepare(table);
    remove(outfile.c_str());
}

PointViewSet writertest_readE57(std::string filename, PointTableRef table)
{
    Options ops;
    ops.add("filename", filename);
    E57Reader reader;
    reader.setOptions(ops);
    reader.prepare(table);
    return reader.execute(table);
}

TEST(E57Writer, testWrite)
{
    std::string outfile(Support::datapath("e57/test.e57"));
    std::string infile(Support::datapath("e57/A4.e57"));

    remove(outfile.c_str());

    E57Reader r;
    Options ro;

    ro.add("filename", infile);
    r.setOptions(ro);

    {
        E57Writer w;
        Options wo;

        wo.add("filename", outfile);
        w.setOptions(wo);
        w.setInput(r);

        PointTable t;

        w.prepare(t);
        w.execute(t);
    }
    PointTable tablein;
    auto viewin = writertest_readE57(infile, tablein);
    auto cloudin = *viewin.begin();
    PointTable tableout;
    auto viewout = writertest_readE57(outfile, tableout);
    auto cloudout = *viewout.begin();

    auto expectedDimensions = {Dimension::Id::X,        Dimension::Id::Y,
                               Dimension::Id::Z,        Dimension::Id::Red,
                               Dimension::Id::Green,    Dimension::Id::Blue,
                               Dimension::Id::Intensity
                              };
    for (point_count_t i = 0; i < cloudout->size(); i++)
    {
        auto ptB = cloudin->point(i);
        auto pt = cloudout->point(i);
        for (auto& dim : expectedDimensions)
        {
            ASSERT_TRUE(pt.hasDim(dim));
            ASSERT_FLOAT_EQ(pt.getFieldAs<float>(dim),
                            ptB.getFieldAs<float>(dim));
        }
    }

    remove(outfile.c_str());
}

void writerTest_testColorRanges(pdal::Reader* r, std::string infile, int min, int max)
{
    std::string outfile(Support::datapath("e57/test.e57"));
    remove(outfile.c_str());

    Options ro;

    ro.add("filename", infile);
    r->setOptions(ro);

    {
        E57Writer w;
        Options wo;

        wo.add("filename", outfile);
        w.setOptions(wo);
        w.setInput(*r);

        PointTable t;

        w.prepare(t);
        w.execute(t);
    }
    e57::ImageFile imf(outfile, "r");

    e57::VectorNode data3D(imf.root().get("/data3D"));
    auto colorLimits = (e57::StructureNode)((e57::StructureNode)data3D.get(0))
                       .get("colorLimits");
    std::vector<std::string> minDims{"colorRedMinimum", "colorGreenMinimum",
                                     "colorBlueMinimum"};
    std::vector<std::string> maxDims{"colorRedMaximum", "colorGreenMaximum",
                                     "colorBlueMaximum"};

    for (auto& dim : minDims)
    {
        ASSERT_EQ(((e57::IntegerNode)colorLimits.get(dim)).value(), min);
    }

    for (auto& dim : maxDims)
    {
        ASSERT_EQ(((e57::IntegerNode)colorLimits.get(dim)).value(), max);
    }

    remove(outfile.c_str());
}

TEST(E57Writer, testWriteRanges)
{
    writerTest_testColorRanges(new LasReader(), Support::datapath("las/autzen_trim.las"), 0, 255);
    writerTest_testColorRanges(new E57Reader(), Support::datapath("e57/A4.e57"), 0, 65535);
}

TEST(E57Writer, testExtraDims)
{
    std::string infile = Support::datapath("las/autzen_trim.las");
    std::string outfile(Support::datapath("e57/test.e57"));
    remove(outfile.c_str());

    Options ro;

    LasReader r;
    ro.add("filename", infile);
    r.setOptions(ro);

    {
        E57Writer w;
        Options wo;

        wo.add("filename", outfile);
        wo.add("extra_dims", "PointSourceId=int,testDim=double");
        w.setOptions(wo);
        w.setInput(r);

        PointTable t;

        w.prepare(t);
        w.execute(t);
    }
    e57::ImageFile imf(outfile, "r");

    e57::VectorNode data3D(imf.root().get("/data3D"));

    // Dimension which is present in input pointcloud
    auto limits = (e57::StructureNode)((e57::StructureNode)data3D.get(0))
                  .get("PointSourceIdLimits");
    ASSERT_EQ(((e57::FloatNode)limits.get("PointSourceIdMinimum")).value(), 0);
    ASSERT_EQ(((e57::FloatNode)limits.get("PointSourceIdMaximum")).value(), 7326);

    // Dimension which is not present in input point cloud.
    // This dimension should not be there in output E57.
    ASSERT_THROW((e57::StructureNode)((e57::StructureNode)data3D.get(0))
                 .get("testDimLimits"), e57::E57Exception);

    // Classification dimension, This will be written if available in input otherwise ignored. Not configurable through extra_dims.
    limits = (e57::StructureNode)((e57::StructureNode)data3D.get(0))
             .get("classificationLimits");
    ASSERT_EQ(((e57::IntegerNode)limits.get("classificationMinimum")).value(), 0);
    ASSERT_EQ(((e57::IntegerNode)limits.get("classificationMaximum")).value(), 255);

    remove(outfile.c_str());
}
} // namespace pdal
