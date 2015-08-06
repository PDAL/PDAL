/******************************************************************************
* Copyright (c) 2013, Howard Butler (hobu.inc@gmail.com)
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

#include <pdal/SpatialReference.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/PointView.hpp>

#include "Support.hpp"

using namespace pdal;

void printChildren(std::ostream& out, MetadataNode m, int depth = 0)
{
    std::vector<MetadataNode> children = m.children();
    for (auto mi = children.begin(); mi != children.end(); ++mi)
    {
        MetadataNode& c = *mi;
        for (int i = 0; i < depth; i++)
            out << "\t";
        out << c.name() << " : " << c.value() << "\n";
        printChildren(out, c, depth + 1);
    }
}

TEST(HexbinFilterTest, HexbinFilterTest_test_1)
{
    StageFactory f;

    Options options;
    options.add("filename", Support::datapath("las/hextest.las"));
    options.add("output_tesselation", true);
    options.add("sample_size", 5000, "Number of samples to use "
        "when estimating hexagon edge size. Specify 0.0 for edge_size if "
        "you want to compute one.");
    options.add("threshold", 1, "Number of points necessary inside "
        "a hexagon to be considered full");
    options.add("edge_length", 0.666666666, "The edge size of the hexagon to "
        "use in situations where you do not want to estimate based on "
        "a sample");

    std::unique_ptr<Stage> reader(f.createStage("readers.las"));
    EXPECT_TRUE(reader.get());
    reader->setOptions(options);

    std::unique_ptr<Stage> hexbin(f.createStage("filters.hexbin"));
    EXPECT_TRUE(hexbin.get());
    hexbin->setOptions(options);
    hexbin->setInput(*reader);

    PointTable table;

    hexbin->prepare(table);
    hexbin->execute(table);

    MetadataNode m = table.metadata();
    m = m.findChild(hexbin->getName());

    std::string filename = Support::temppath("hexbin.txt");
    std::ofstream out(filename);
    printChildren(out, m);
    out.close();
    FileUtils::deleteFile(filename);
}
