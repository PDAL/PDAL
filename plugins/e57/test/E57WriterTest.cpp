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

#include "plugins/e57/io/E57Reader.hpp"
#include "plugins/e57/io/E57Writer.hpp"
#include "plugins/e57/io/Utils.hpp"

using namespace pdal;

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

TEST(E57WRiter, testWrite)
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

    auto expectedDimensions = {
        pdal::Dimension::Id::X,        pdal::Dimension::Id::Y,
        pdal::Dimension::Id::Z,        pdal::Dimension::Id::Red,
        pdal::Dimension::Id::Green,    pdal::Dimension::Id::Blue,
        pdal::Dimension::Id::Intensity};
    for (pdal::point_count_t i = 0; i < cloudout->size(); i++)
    {
        auto ptB = cloudin->point(i);
        auto pt = cloudout->point(i);
        for (auto& dim : expectedDimensions)
        {
            ASSERT_TRUE(pt.hasDim(dim));
            ASSERT_FLOAT_EQ(pt.getFieldAs<double>(dim),
                            ptB.getFieldAs<double>(dim));
        }
    }

    remove(outfile.c_str());
}