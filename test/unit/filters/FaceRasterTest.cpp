/******************************************************************************
* Copyright (c) 2019, Hobu Inc. (info@hobu.co)
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
*     * Neither the name of Hobu, Inc. nor the
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
#include <pdal/private/Raster.hpp>

#include "Support.hpp"

// NOTE: The test data has an accompanying jpg that depicts the points,
//  their triangulation and the interesting barycentric calculation.

namespace pdal
{

TEST(FaceRasterTest, basic)
{
    Options ro;
    ro.add("filename", Support::datapath("filters/hagtest.txt"));

    StageFactory factory;
    Stage& r = *(factory.createStage("readers.text"));
    r.setOptions(ro);

    Options rngO;
    rngO.add("limits", "Classification[2:2]");
    Stage& rng = *(factory.createStage("filters.range"));
    rng.setOptions(rngO);
    rng.setInput(r);

    Stage& d = *(factory.createStage("filters.delaunay"));
    d.setInput(rng);

    Options fo;
    fo.add("resolution", 1);
    Stage& f = *(factory.createStage("filters.faceraster"));
    f.setInput(d);
    f.setOptions(fo);

    PointTable t1;
    f.prepare(t1);
    PointViewSet s = f.execute(t1);
    PointViewPtr v = *s.begin();

    const std::vector<double> expected
    {
        0, 10, 0, 0, 0, 0,
        0, 9, 7, 0, 0, 0,
        10, 8, 6, 4, 0, 0,
        6.66667, 4.66667, 2.66667, 6, 7, 0,
        3.33333, 1.33333, 4.66667, 8, 9, 10,
        0, 3.33333, 6.66667, 10, 0, 0 
    };
    const double *k = expected.data();

    Rasterd *raster = v->raster();
    for (int j = raster->height() - 1; j >= 0; j--)
        for (int i = 0; i < raster->width(); ++i)
            EXPECT_NEAR(raster->at(i, j), *k++, .00001);
}

} // namespace pdal
