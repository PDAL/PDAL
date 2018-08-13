/******************************************************************************
* Copyright (c) 2018, Hobu Inc. (info@hobu.co)
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

#include <pdal/StageFactory.hpp>

#include "Support.hpp"

namespace pdal
{

TEST(VoxelTest, center)
{
    StageFactory fac;

    Stage *reader = fac.createStage("readers.las");
    Options ro;
    ro.add("filename", Support::datapath("las/autzen_trim.las"));
    reader->setOptions(ro);

    Stage *filter = fac.createStage("filters.voxelcenternearestneighbor");
    Options fo;
    fo.add("cell", 10);
    filter->setOptions(fo);
    filter->setInput(*reader);

    PointTable t;
    filter->prepare(t);
    PointViewSet set = filter->execute(t);
    EXPECT_EQ(set.size(), 1U);
    PointViewPtr v = *set.begin();
    EXPECT_EQ(v->size(), 7820U);

    PointId sums[] = { 39811200, 35547988, 38700040, 43845452, 52001563,
        69596800, 75166285, 50947904 };
    PointId sum;
    PointId id;
    size_t iter = 0;
    for (id = 0; id < v->size(); ++id)
    {
        if (id % 1000 == 0)
        {
            if (id)
            {
            EXPECT_EQ(sums[iter++], sum);
            }
            sum = 0;
        }
        sum += v->index(id);
    }
    EXPECT_EQ(sums[iter], sum);
}

} // namespace
