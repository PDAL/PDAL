/******************************************************************************
* Copyright (c) 2016, Howard Butler <howard@hobu.co>
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
#include <pdal/private/MathUtils.hpp>

#include "Support.hpp"

namespace pdal
{

TEST(MathUtilsTest, barycentric)
{
    using namespace math;

    // Triangle (1, 1), (5, 4), (3, 6) -- z's 55, 25, 100
    double x1 = 1;
    double y1 = 1;
    double z1 = 55;
    double x2 = 5;
    double y2 = 4;
    double z2 = 25;
    double x3 = 3;
    double y3 = 6;
    double z3 = 100;

    struct point
    {
        double x;
        double y;
    };

    std::vector<point> points { { 3, 3 }, {4, 2}, {5, 4}, {4, 6}, {1, 5},
        {4, 5}, {2, 7.0 / 4}, {3, 4}, {3, 6} };

    double inf = std::numeric_limits<double>::infinity();
    std::vector<double> results { 48.57142857142, inf, 25, inf, inf,
        62.5, 47.5, 65.7142857142, 100 };

    double z;
    for (size_t i = 0; i < points.size(); ++i)
    {
        const point& p = points[i];
        z = barycentricInterpolation(x1, y1, z1, x2, y2, z2, x3, y3, z3, p.x, p.y);
        if (std::isinf(results[i]))
            EXPECT_TRUE(std::isinf(z));
        else
            EXPECT_NEAR(z, results[i], .0000000001);
    }

    // Re-order triangle points (x2 is before x1 in input).  Results should be the same.
    for (size_t i = 0; i < points.size(); ++i)
    {
        const point& p = points[i];
        z = barycentricInterpolation(x2, y2, z2, x1, y1, z1, x3, y3, z3, p.x, p.y);
        if (std::isinf(results[i]))
            EXPECT_TRUE(std::isinf(z));
        else
            EXPECT_NEAR(z, results[i], .0000000001);
    }
}

} // namespace pdal
