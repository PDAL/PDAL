/******************************************************************************
* Copyright (c) 2023, Antoine Lavenant (antoine.lavenant@ign.fr)
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

#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <io/FauxReader.hpp>
#include <filters/GridDecimationFilter.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(GridDecimationFilterTest, create)
{
    StageFactory f;
    Stage* filter(f.createStage("filters.gridDecimation"));
    EXPECT_TRUE(filter);
}

TEST(DecimationFilterTest, GridDecimationFilterTest_test1)
{
    Options ro;
    ro.add("filename", Support::datapath("las/4_6.las"));

    StageFactory factory;
    Stage& r = *(factory.createStage("readers.las"));
    r.setOptions(ro);

    Options gdOps;
    gdOps.add("output_type", "max");
    gdOps.add("resolution", 1.);
    gdOps.add("where", "Classification==2");
    gdOps.add("value", "Classification=3");

    GridDecimationFilter filter;
    filter.setOptions(gdOps);
    filter.setInput(r);

    PointTable table;

    filter.prepare(table);
    PointViewSet viewSet = filter.execute(table);

    EXPECT_EQ(viewSet.size(), 1u);

    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(view->size(), 198975UL);

    int nbThreadPts (0);
    for (PointId i = 0; i < view->size(); ++i)
    {
        PointRef point = view->point(i);
        uint8_t classification = point.getFieldAs<uint8_t>(Dimension::Id::Classification);
        if (classification==3) nbThreadPts++;
    }

    EXPECT_EQ(nbThreadPts, 65067);
}
