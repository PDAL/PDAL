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

#include <ChipperFilter.hpp>
#include <LasWriter.hpp>
#include <LasReader.hpp>
#include <pdal/Options.hpp>
#include <pdal/StageWrapper.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(ChipperTest, test_construction)
{
    PointTable table;

    Options ops1;
    std::string filename(Support::datapath("las/1.2-with-color.las"));
    ops1.add("filename", filename);
    LasReader reader;
    reader.setOptions(ops1);

    {
        // need to scope the writer, so that's it dtor can use the stream

        Options options;
        Option capacity("capacity", 15, "capacity");
        options.add(capacity);

        ChipperFilter chipper;
        chipper.setInput(reader);
        chipper.setOptions(options);
        chipper.prepare(table);
        PointViewSet viewSet = chipper.execute(table);
        EXPECT_EQ(viewSet.size(), 71u);

        std::vector<PointViewPtr> views;
        for (auto it = viewSet.begin(); it != viewSet.end(); ++it)
            views.push_back(*it);

        auto sorter = [](PointViewPtr p1, PointViewPtr p2)
        {
            //This is super inefficient, but we're doing tests.
            BOX2D b1;
            BOX2D b2;

            p1->calculateBounds(b1);
            p2->calculateBounds(b2);

            return b1.minx < b2.minx ?  true :
                b1.minx > b2.minx ? false :
                b1.miny < b2.miny;
        };

        std::sort(views.begin(), views.end(), sorter);

        PointViewPtr view = views[2];
        BOX2D bounds;
        view->calculateBounds(bounds);

        EXPECT_NEAR(bounds.minx, 635674.05, 0.05);
        EXPECT_NEAR(bounds.maxx, 635993.93, 0.05);
        EXPECT_NEAR(bounds.miny, 848992.45, 0.05);
        EXPECT_NEAR(bounds.maxy, 849427.07, 0.05);

        for (size_t i = 0; i < views.size(); ++i)
            EXPECT_EQ(views[i]->size(), 15u);
    }
}


// Make sure things don't crash if the point buffer is empty.
TEST(ChipperTest, empty_buffer)
{
    PointTable table;
    PointViewPtr view(new PointView(table));

    Options ops;

    ChipperFilter chipper;
    chipper.prepare(table);
    StageWrapper::ready(chipper, table);
    PointViewSet viewSet = StageWrapper::run(chipper, view);
    StageWrapper::done(chipper, table);

    EXPECT_EQ(viewSet.size(), 0u);
}

//ABELL
/**
TEST(ChipperTest, test_ordering)
{
    std::string candidate_filename(Support::datapath("autzen-utm.las"));
    std::string source_filename(Support::datapath("autzen-utm-chipped-25.las"));

    Options options;
    Option filename("filename", source_filename, "");
    options.add(filename);

    Option capacity("capacity", 25,"capacity");
    options.add(capacity);

    LasReader candidate_reader(options);
    std::shared_ptr<ChipperFilter> chipper(new ChipperFilter)(options);
    chipper->setInput(&candidate_reader);
    chipper->prepare();

    Option& query = options.getOptionByRef("filename");
    query.setValue<std::string>(source_filename);

    LasReader source_reader(options);
    source_reader.prepare();

    EXPECT_EQ(chipper->getNumPoints(), source_reader.getNumPoints());

    PointView candidate(chipper->getSchema(), chipper->getNumPoints());
    PointView patch(chipper->getSchema(), chipper->getNumPoints());

    StageSequentialIterator* iter_c = chipper->createSequentialIterator(patch);
    uint64_t numRead(0);

    while (true)
    {
        numRead = iter_c->read(patch);
        if (! numRead)
            break;
        candidate.copyPointsFast(candidate.getNumPoints(), 0, patch, patch.getNumPoints());
        candidate.setNumPoints(candidate.getNumPoints() + patch.getNumPoints());
    }
    EXPECT_EQ(candidate.getNumPoints(), chipper->getNumPoints());

    PointView source(source_reader.getSchema(), source_reader.getNumPoints());

    StageSequentialIterator* iter_s = source_reader.createSequentialIterator(source);
    numRead = iter_s->read(source);
    EXPECT_EQ(numRead, source_reader.getNumPoints());



    Schema const& cs = candidate.getSchema();
    Schema const& ss = source.getSchema();

    Dimension const& sdimX = ss.getDimension("X");
    Dimension const& sdimY = ss.getDimension("Y");
    Dimension const& sdimZ = ss.getDimension("Z");
    Dimension const& sdimIntensity = ss.getDimension("Intensity");
    Dimension const& sdimRed = ss.getDimension("Red");
    Dimension const& sdimGreen = ss.getDimension("Green");
    Dimension const& sdimBlue = ss.getDimension("Blue");

    Dimension const& cdimX = cs.getDimension("X");
    Dimension const& cdimY = cs.getDimension("Y");
    Dimension const& cdimZ = cs.getDimension("Z");
    Dimension const& cdimIntensity = cs.getDimension("Intensity");
    Dimension const& cdimRed = cs.getDimension("Red");
    Dimension const& cdimGreen = cs.getDimension("Green");
    Dimension const& cdimBlue = cs.getDimension("Blue");
    //
    // int X[] = { 49405730, 49413382, 49402110, 494192890, 49418622, 49403411 };
    // int Y[] = { 487743335, 487743982, 487743983, 487744219, 487744254, 487745019 };
    // int Z[] = { 13063, 13044, 13046, 13050, 13049, 13066 };
    // int I[] = { 134, 75, 153, 93, 67, 167 };
    // int R[] = { 142, 152, 146, 104, 113, 163 };
    // int G[] = { 102, 108, 104, 96, 97, 118 };
    // int B[] = { 137, 134, 140, 120, 123, 150 };
    //
    for (unsigned i = 0; i < candidate.getNumPoints(); ++i)
    {
        int32_t sx = source.getField<int32_t>(sdimX, i);
        int32_t sy = source.getField<int32_t>(sdimY, i);
        int32_t sz = source.getField<int32_t>(sdimZ, i);
        uint16_t sintensity = source.getField<uint16_t>(sdimIntensity, i);
        uint16_t sred = source.getField<uint16_t>(sdimRed, i);
        uint16_t sgreen = source.getField<uint16_t>(sdimGreen, i);
        uint16_t sblue = source.getField<uint16_t>(sdimBlue, i);

        int32_t cx = candidate.getField<int32_t>(cdimX, i);
        int32_t cy = candidate.getField<int32_t>(cdimY, i);
        int32_t cz = candidate.getField<int32_t>(cdimZ, i);
        uint16_t cintensity = candidate.getField<uint16_t>(cdimIntensity, i);
        uint16_t cred = candidate.getField<uint16_t>(cdimRed, i);
        uint16_t cgreen = candidate.getField<uint16_t>(cdimGreen, i);
        uint16_t cblue = candidate.getField<uint16_t>(cdimBlue, i);


        EXPECT_EQ(sx, cx);
        EXPECT_EQ(sy, cy);
        EXPECT_EQ(sz, cz);
        EXPECT_EQ(sintensity, cintensity);
        EXPECT_EQ(sred, cred);
        EXPECT_EQ(sgreen, cgreen);
        EXPECT_EQ(sblue, cblue);
    }
    delete iter_c;
    delete iter_s;

}
**/
