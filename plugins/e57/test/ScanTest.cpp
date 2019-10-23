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

#include <gtest/gtest.h>
#include "Support.hpp"

#include "plugins/e57/io/Scan.hpp"
#include "plugins/e57/io/E57Reader.hpp"

namespace pdal
{

TEST(ScanTest,testScanCtr)
{
    e57::ImageFile imf(Support::datapath("e57/A4.e57"), "r");

    const e57::ustring normalsExtension(
        "http://www.libe57.org/E57_NOR_surface_normals.txt");
    e57::ustring _normalsExtension;

    // the extension may already be registered
    if (!imf.extensionsLookupPrefix("nor", _normalsExtension))
        imf.extensionsAdd("nor", normalsExtension);

    e57::VectorNode data3D(imf.root().get("/data3D"));
    auto scan = new e57::Scan((e57::StructureNode)data3D.get(0));
    ASSERT_EQ(scan->getNumPoints(), (pdal::point_count_t)4);
    imf.close();
}

TEST(ScanTest,getDimension)
{
    e57::ImageFile imf(Support::datapath("e57/A4.e57"), "r");
    const e57::ustring normalsExtension(
        "http://www.libe57.org/E57_NOR_surface_normals.txt");
    e57::ustring _normalsExtension;

    // the extension may already be registered
    if (!imf.extensionsLookupPrefix("nor", _normalsExtension))
        imf.extensionsAdd("nor", normalsExtension);

    e57::VectorNode data3D(imf.root().get("/data3D"));
    auto scan = new e57::Scan((e57::StructureNode)data3D.get(0));

    auto dimensions = scan->getDimensions();
    ASSERT_EQ(dimensions.size(),(unsigned long)7);
    std::vector<std::string> expectedDimensions = {"cartesianX","cartesianY","cartesianZ",
                                                   "colorRed","colorGreen","colorBlue","intensity"
                                                  };
    for (auto dim: expectedDimensions)
    {
        ASSERT_TRUE(dimensions.find(dim) != dimensions.end());
    }
    imf.close();
}

TEST(ScanTest,testGetPoints)
{
    e57::ImageFile imf(Support::datapath("e57/A_B.e57"), "r");
    const e57::ustring normalsExtension(
        "http://www.libe57.org/E57_NOR_surface_normals.txt");
    e57::ustring _normalsExtension;

    // the extension may already be registered
    if (!imf.extensionsLookupPrefix("nor", _normalsExtension))
        imf.extensionsAdd("nor", normalsExtension);

    e57::VectorNode data3D(imf.root().get("/data3D"));
    auto scan = new e57::Scan((e57::StructureNode)data3D.get(0));

    auto pts = scan->getPoints();
    ASSERT_EQ((pdal::point_count_t)pts.childCount(), scan->getNumPoints());
    ASSERT_EQ(pts.childCount(),2);
    auto secondScan = new e57::Scan((e57::StructureNode)data3D.get(1));
    pts = secondScan->getPoints();
    ASSERT_EQ((pdal::point_count_t)pts.childCount(),secondScan->getNumPoints());
    ASSERT_EQ(pts.childCount(),(int64_t)4);
    imf.close();
}

TEST(ScanTest,testBbox)
{
    e57::ImageFile imf(Support::datapath("e57/A4.e57"), "r");
    const e57::ustring normalsExtension(
        "http://www.libe57.org/E57_NOR_surface_normals.txt");
    e57::ustring _normalsExtension;

    // the extension may already be registered
    if (!imf.extensionsLookupPrefix("nor", _normalsExtension))
        imf.extensionsAdd("nor", normalsExtension);

    e57::VectorNode data3D(imf.root().get("/data3D"));
    auto scan = new e57::Scan((e57::StructureNode)data3D.get(0));
    BOX3D box = scan->getBoundingBox();
    ASSERT_DOUBLE_EQ(box.minx, -4.49522018432617188e+01);
    ASSERT_DOUBLE_EQ(box.maxx, -4.43000984191894531e+01);
    ASSERT_DOUBLE_EQ(box.miny, -1.34930002689361572);
    ASSERT_DOUBLE_EQ(box.maxy, -8.85999977588653564e-01);
    ASSERT_DOUBLE_EQ(box.minz, -6.07699990272521973e-01);
    ASSERT_DOUBLE_EQ(box.maxz, 3.69399994611740112e-01);
    imf.close();
}

} // namespace pdal
