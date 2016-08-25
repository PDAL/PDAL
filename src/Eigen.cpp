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

#include <pdal/Eigen.hpp>
#include <pdal/PointView.hpp>
#include <pdal/util/Bounds.hpp>

#include <Eigen/Dense>

#include <vector>

namespace pdal
{

Eigen::Vector3f computeCentroid(PointView& view, std::vector<PointId> ids)
{
    using namespace Eigen;

    auto n = ids.size();

    double mx, my, mz;
    mx = my = mz = 0.0;
    for (auto const& j : ids)
    {
        mx += view.getFieldAs<double>(Dimension::Id::X, j);
        my += view.getFieldAs<double>(Dimension::Id::Y, j);
        mz += view.getFieldAs<double>(Dimension::Id::Z, j);
    }

    Vector3f centroid;
    centroid << mx/n, my/n, mz/n;

    return centroid;
}

Eigen::Matrix3f computeCovariance(PointView& view, std::vector<PointId> ids)
{
    using namespace Eigen;

    auto n = ids.size();

    Vector3f centroid = computeCentroid(view, ids);

    // demean the neighborhood
    MatrixXf A(3, n);
    size_t k = 0;
    for (auto const& j : ids)
    {
        A(0, k) = view.getFieldAs<double>(Dimension::Id::X, j) - centroid[0];
        A(1, k) = view.getFieldAs<double>(Dimension::Id::Y, j) - centroid[1];
        A(2, k) = view.getFieldAs<double>(Dimension::Id::Z, j) - centroid[2];
        k++;
    }

    return A * A.transpose();
}

uint8_t computeRank(PointView& view, std::vector<PointId> ids, double threshold)
{
    using namespace Eigen;

    Matrix3f B = computeCovariance(view, ids);

    JacobiSVD<Matrix3f> svd(B);
    svd.setThreshold(threshold);
    
    return static_cast<uint8_t>(svd.rank());
}

Eigen::MatrixXd createDSM(PointView& view, int rows, int cols, double cell_size,
                          BOX2D bounds)
{
    using namespace Dimension;
    using namespace Eigen;
    
    MatrixXd ZImin(rows, cols);
    ZImin.setConstant(std::numeric_limits<double>::quiet_NaN());

    int maxrow = bounds.miny + rows * cell_size;
    
    auto clamp = [](int t, int min, int max)
    {
        return ((t < min) ? min : ((t > max) ? max : t));
    };
    
    auto getColIndex = [&bounds, &cell_size](double x)
    {
        return static_cast<int>(floor((x - bounds.minx) / cell_size));
    };

    auto getRowIndex = [&maxrow, &cell_size](double y)
    {
        return static_cast<int>(floor((maxrow - y) / cell_size));
    };
    
    for (PointId i = 0; i < view.size(); ++i)
    {
        double x = view.getFieldAs<double>(Id::X, i);
        double y = view.getFieldAs<double>(Id::Y, i);
        double z = view.getFieldAs<double>(Id::Z, i);

        int c = clamp(getColIndex(x), 0, cols-1);
        int r = clamp(getRowIndex(y), 0, rows-1);

        if (z < ZImin(r, c) || std::isnan(ZImin(r, c)))
            ZImin(r, c) = z;
    }

    return ZImin;
}

} // namespace pdal
