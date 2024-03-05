/******************************************************************************
* Copyright (c) 2024, Howard Butler (info@hobu.co)
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
#include <io/LasReader.hpp>
#include <io/TextReader.hpp>

#include "../filters/H3Filter.hpp"


#include "Support.hpp"

using namespace pdal;

TEST(H3FilterTest, createStage)
{
    StageFactory f;
    Stage* filter(f.createStage("filters.h3"));
    EXPECT_TRUE(filter);
}

TEST(H3FilterTest, stream_test_2)
{
    // Fill table 1 with UTM17 data.
    Options ops1;
    ops1.add("filename", Support::datapath("las/test_utm17.las"));
    LasReader reader1;
    reader1.setOptions(ops1);

    Options ops1a;
    H3Filter h3;
    ops1a.add("resolution", 12);
    h3.setInput(reader1);
    h3.setOptions(ops1a);

    PointTable table1;
    h3.prepare(table1);
    PointViewSet s1 = h3.execute(table1);
    PointViewPtr v1 = *(s1.begin());
}

