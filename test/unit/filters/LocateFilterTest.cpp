/******************************************************************************
* Copyright (c) 2016, Bradley J Chambers (brad.chambers@gmail.com)
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

#include <pdal/Options.hpp>
#include <filters/LocateFilter.hpp>
#include <io/LasReader.hpp>

#include "Support.hpp"

using namespace pdal;

TEST(LocateTest, locate_max)
{
    PointTable table;

    Options ro;
    ro.add("filename", Support::datapath("las/1.2-with-color.las"));
    LasReader r;
    r.setOptions(ro);

    Options fo;
    fo.add("dimension", "Z");
    fo.add("minmax", "max");

    LocateFilter f;
    f.setInput(r);
    f.setOptions(fo);
    f.prepare(table);
    PointViewSet viewSet = f.execute(table);
    EXPECT_EQ(1u, viewSet.size());
    
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(1u, view->size());
    
    EXPECT_NEAR(586.38, view->getFieldAs<double>(Dimension::Id::Z, 0), 0.0001);
}

TEST(LocateTest, locate_min)
{
    PointTable table;

    Options ro;
    ro.add("filename", Support::datapath("las/1.2-with-color.las"));
    LasReader r;
    r.setOptions(ro);

    Options fo;
    fo.add("dimension", "Z");
    fo.add("minmax", "min");

    LocateFilter f;
    f.setInput(r);
    f.setOptions(fo);
    f.prepare(table);
    PointViewSet viewSet = f.execute(table);
    EXPECT_EQ(1u, viewSet.size());
    
    PointViewPtr view = *viewSet.begin();
    EXPECT_EQ(1u, view->size());
    
    EXPECT_NEAR(406.59, view->getFieldAs<double>(Dimension::Id::Z, 0), 0.0001);
}
