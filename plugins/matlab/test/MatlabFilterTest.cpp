/******************************************************************************
* Copyright (c) 2017, Howard Butler (howard@hobu.co)
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

#include "../filters/Environment.hpp"
#include "../filters/MatlabFilter.hpp"
#include <pdal/StageFactory.hpp>
#include <io/FauxReader.hpp>
#include <io/LasReader.hpp>
#include <Support.hpp>

using namespace pdal;
using namespace pdal::mlang;
//
// TEST(MatlabFilterTest, MatlabFilterTest_basic)
// {
//     Engine* engine(0);
//     engine = mlang::Environment::get()->m_engine;
//     ASSERT_TRUE(engine != 0);
// }

TEST(MatlabFilterTest, simple_fetch)
{

    Options reader_opts;
    {
        BOX3D bounds(1.0, 2.0, 3.0, 101.0, 102.0, 103.0);
        Option opt1("bounds", bounds);
        Option opt2("count", 10);
        Option opt3("mode", "constant");

        reader_opts.add(opt1);
        reader_opts.add(opt2);
        reader_opts.add(opt3);
    }

    Options xfilter_opts;
    {
        const Option source("source",
            "PDAL.X = PDAL.X + 10;PDAL.X(1,1); PDAL.metadata"
            );
        xfilter_opts.add(source);
        const Option args("pdalargs","pdalargs");
        xfilter_opts.add(args);
    }

    StageFactory f;
    {
        FauxReader reader;

        reader.setOptions(reader_opts);

        Stage* xfilter(f.createStage("filters.matlab"));
        LogPtr log = Log::makeLog("matlab.log", "stderr");
        log->setLevel(LogLevel::Debug2);
        xfilter->setLog(log);
        xfilter->setOptions(xfilter_opts);
        xfilter->setInput(reader);

        PointTable table;
        xfilter->prepare(table);
        PointViewSet pvSet = xfilter->execute(table);

        PointLayoutPtr layout(table.layout());
        MetadataNode m = table.metadata();
        m = m.findChild("filters.matlab");
        MetadataNodeList l = m.children();

        EXPECT_EQ(l.size(), 0u);
        if (l.size())
        {
            // Matlab filter doesn't i/o metadata
            EXPECT_EQ(l[0].name(), "filters.matlab");
            EXPECT_EQ(l[0].value(), "52");
            EXPECT_EQ(l[0].description(), "a filter description");

        }


        EXPECT_EQ(pvSet.size(), 1u);
        PointViewPtr view = *pvSet.begin();
        EXPECT_EQ(view->size(), 10u);
        EXPECT_EQ(view->getFieldAs<double>(pdal::Dimension::Id::X, 0), 11u);
    }

}

TEST(MatlabFilterTest, logical)
{

    Options lasReadOpts;
    lasReadOpts.add("filename", Support::datapath("las/1.2-with-color.las"));

    Options xfilter_opts;
    {
        const Option source("source",
            "PDAL.Mask = PDAL.Intensity < 30"
            );
        xfilter_opts.add(source);
    }

    StageFactory f;
    {
        LasReader reader;
        reader.setOptions(lasReadOpts);


        Stage* xfilter(f.createStage("filters.matlab"));
        LogPtr log = Log::makeLog("matlab.log", "stderr");
        log->setLevel(LogLevel::Debug2);
        xfilter->setLog(log);
        xfilter->setOptions(xfilter_opts);
        xfilter->setInput(reader);

        PointTable table;
        xfilter->prepare(table);
        PointViewSet pvSet = xfilter->execute(table);
        EXPECT_EQ(pvSet.size(), 1u);
        PointViewPtr view = *pvSet.begin();
        EXPECT_EQ(view->size(), 357u);
    }

}
