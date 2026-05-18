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

#include "io/BufferReader.hpp"
#include "filters/VoxelCentroidNearestNeighborFilter.hpp"

namespace pdal
{

    // Helper function to check if a given point is present in a PointView.
    bool pointFoundInOutput(PointViewPtr view, const std::array<double, 3>& pt, double eps = 1e-6)
    {
        for (PointId i = 0; i < view->size(); ++i)
        {
            PointRef p(*view, i);
            double x = p.getFieldAs<double>(Dimension::Id::X);
            double y = p.getFieldAs<double>(Dimension::Id::Y);
            double z = p.getFieldAs<double>(Dimension::Id::Z);
            if (std::fabs(x - pt[0]) < eps &&
                std::fabs(y - pt[1]) < eps &&
                std::fabs(z - pt[2]) < eps)
            {
                return true;
            }
        }
        return false;
    }

    TEST(VoxelCentroidNearestNeighborFilterTest, SyntheticCloudKnownOutput)
    {
        // Set up the point table and register dimensions.
        PointTable table;
        table.layout()->registerDims({Dimension::Id::X, Dimension::Id::Y, Dimension::Id::Z});
        PointViewPtr inputView(new PointView(table));

        // We use (5,5,5) as the first point which defines the grid origin.
        // Points are chosen such that when downsampled with cell=10:
        //   - Points (5,5,5) & (11,11,11) fall in one voxel, with the candidate expected to be (11,11,11)
        //   - Points (0,0,0) & (1,1,1) (which fall to the left/below the origin) fall in the negative voxel,
        //     with the candidate expected to be (0,0,0) (i.e. closer to the voxel center).
        //   - (21,21,21) falls in another voxel and is kept.
        std::vector<std::array<double, 3>> inputPoints = {
            {5, 5, 5},       // First point, defines origin.
            {0, 0, 0},       // Negative side.
            {1, 1, 1},       // Same voxel as above.
            {11, 11, 11},    // Same voxel as (5,5,5) but closer to its voxel center.
            {21, 21, 21}     // Different voxel.
        };

        PointId id = 0;
        for (const auto &pt : inputPoints)
        {
            inputView->setField(Dimension::Id::X, id, pt[0]);
            inputView->setField(Dimension::Id::Y, id, pt[1]);
            inputView->setField(Dimension::Id::Z, id, pt[2]);
            ++id;
        }

        // Create a BufferReader and add our synthetic view.
        BufferReader reader;
        reader.addView(inputView);

        // Set up the filter. With a cell size of 10, the voxel grid is built relative to (5,5,5).
        VoxelCentroidNearestNeighborFilter filter;
        Options opts;
        opts.add("cell", 10);
        filter.setOptions(opts);
        filter.setInput(reader);

        filter.prepare(table);
        PointViewSet outputViews = filter.execute(table);

        // We expect one output view.
        ASSERT_EQ(outputViews.size(), 1u);
        PointViewPtr outputView = *outputViews.begin();

        // Expected output:
        //   * For the voxel containing (5,5,5) and (11,11,11) the candidate is (11,11,11)
        //   * For the voxel containing (0,0,0) and (1,1,1) the candidate is (0,0,0)
        //   * For the voxel containing (21,21,21) the candidate is (21,21,21)
        std::vector<std::array<double, 3>> expected = {
            {11, 11, 11},
            {0,  0,   0},
            {21, 21, 21}
        };

        // Ensure that the number of points in the output matches the expected count.
        EXPECT_EQ(outputView->size(), expected.size());

        // Verify that each expected point exists in the output.
        for (const auto& expPt : expected)
        {
            EXPECT_TRUE(pointFoundInOutput(outputView, expPt))
                << "Expected point (" << expPt[0] << ", " << expPt[1]
                << ", " << expPt[2] << ") not found in the output.";
        }
    }

} // namespace pdal
