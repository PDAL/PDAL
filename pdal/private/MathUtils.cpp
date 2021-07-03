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

#include <array>
#include <cfloat>
#include <numeric>
#include <vector>

#include <pdal/PointView.hpp>
#include <pdal/SpatialReference.hpp>
#include <pdal/util/Bounds.hpp>
#include <pdal/util/Utils.hpp>
#include <pdal/private/gdal/Raster.hpp>

#include "MathUtils.hpp"

namespace pdal
{
namespace math
{

#pragma warning (push)
#pragma warning (disable: 4244)

PointViewPtr demeanPointView(const PointView& view)
{
    using namespace Eigen;
    using namespace Dimension;

    PointIdList ids(view.size());
    std::iota(ids.begin(), ids.end(), 0);
    Vector3d centroid = computeCentroid(view, ids);
    PointViewPtr outView = view.makeNew();

    for (PointId idx = 0; idx < view.size(); idx++)
    {
        double x = view.getFieldAs<double>(Id::X, idx) - centroid.x();
        double y = view.getFieldAs<double>(Id::Y, idx) - centroid.y();
        double z = view.getFieldAs<double>(Id::Z, idx) - centroid.z();
        outView->setField(Id::X, idx, x);
        outView->setField(Id::Y, idx, y);
        outView->setField(Id::Z, idx, z);
    }
    return outView;
}

PointViewPtr demeanPointView(const PointView& view, double* centroid)
{
    using namespace Eigen;
    using namespace Dimension;

    PointViewPtr outView = view.makeNew();

    for (PointId idx = 0; idx < view.size(); idx++)
    {
        double x = view.getFieldAs<double>(Id::X, idx) - centroid[0];
        double y = view.getFieldAs<double>(Id::Y, idx) - centroid[1];
        double z = view.getFieldAs<double>(Id::Z, idx) - centroid[2];
        outView->setField(Id::X, idx, x);
        outView->setField(Id::Y, idx, y);
        outView->setField(Id::Z, idx, z);
    }
    return outView;
}

PointViewPtr transform(const PointView& view, double* matrix)
{
    using namespace Dimension;

    PointViewPtr outView = view.makeNew();
    for (PointId idx = 0; idx < view.size(); idx++)
    {
        double x = view.getFieldAs<double>(Id::X, idx);
        double y = view.getFieldAs<double>(Id::Y, idx);
        double z = view.getFieldAs<double>(Id::Z, idx);
        outView->setField(Id::X, idx,
                          x * matrix[0] + y * matrix[4] + z * matrix[8] + matrix[12]);
        outView->setField(Id::Y, idx,
                          x * matrix[1] + y * matrix[5] + z * matrix[9] + matrix[13]);
        outView->setField(Id::Z, idx,
                          x * matrix[2] + y * matrix[6] + z * matrix[10] + matrix[14]);
    }
    return outView;
}


void transformInPlace(PointView& view, double* matrix)
{
    using namespace Dimension;

    for (PointId idx = 0; idx < view.size(); idx++)
    {
        double x = view.getFieldAs<double>(Id::X, idx);
        double y = view.getFieldAs<double>(Id::Y, idx);
        double z = view.getFieldAs<double>(Id::Z, idx);
        view.setField(Id::X, idx,
                      x * matrix[0] + y * matrix[4] + z * matrix[8] + matrix[12]);
        view.setField(Id::Y, idx,
                      x * matrix[1] + y * matrix[5] + z * matrix[9] + matrix[13]);
        view.setField(Id::Z, idx,
                      x * matrix[2] + y * matrix[6] + z * matrix[10] + matrix[14]);
    }
}

Eigen::Vector3d computeCentroid(const PointView& view,
    const PointIdList& ids)
{
    using namespace Eigen;

    double mx, my, mz;
    mx = my = mz = 0.0;
    point_count_t n(0);
    for (auto const& j : ids)
    {
        auto update = [&n](double value, double average)
        {
            double delta, delta_n;
            delta = value - average;
            delta_n = delta / n;
            return average + delta_n;
        };
        n++;
        mx = update(view.getFieldAs<double>(Dimension::Id::X, j), mx);
        my = update(view.getFieldAs<double>(Dimension::Id::Y, j), my);
        mz = update(view.getFieldAs<double>(Dimension::Id::Z, j), mz);
    }

    Vector3d centroid;
    centroid << mx, my, mz;

    return centroid;
}

Eigen::Matrix3d computeCovariance(const PointView& view,
    const PointIdList& ids)
{
    using namespace Eigen;

    auto n = ids.size();

    Vector3d centroid = computeCentroid(view, ids);

    // demean the neighborhood
    MatrixXd A(3, n);
    size_t k = 0;
    for (auto const& j : ids)
    {
        A(0, k) =
            static_cast<float>(view.getFieldAs<double>(Dimension::Id::X, j) -
                centroid[0]);
        A(1, k) =
            static_cast<float>(view.getFieldAs<double>(Dimension::Id::Y, j) -
                centroid[1]);
        A(2, k) =
            static_cast<float>(view.getFieldAs<double>(Dimension::Id::Z, j) -
                centroid[2]);
        k++;
    }

    return A * A.transpose() / (ids.size()-1);
}

uint8_t computeRank(const PointView& view, const PointIdList& ids,
    double threshold)
{
    using namespace Eigen;

    Matrix3d B = computeCovariance(view, ids);

    JacobiSVD<Matrix3d> svd(B);
    svd.setThreshold((float)threshold);

    return static_cast<uint8_t>(svd.rank());
}

Eigen::MatrixXd extendedLocalMinimum(const PointView& view, int rows, int cols,
                                     double cell_size, BOX2D bounds)
{
    using namespace Dimension;
    using namespace Eigen;

    // Index elevation values by row and column.
    std::map<uint32_t, std::vector<double>> hash;
    for (PointId i = 0; i < view.size(); ++i)
    {
        double x = view.getFieldAs<double>(Id::X, i);
        double y = view.getFieldAs<double>(Id::Y, i);
        double z = view.getFieldAs<double>(Id::Z, i);

        int c = Utils::clamp(static_cast<int>(floor(x-bounds.minx)/cell_size), 0, cols-1);
        int r = Utils::clamp(static_cast<int>(floor(y-bounds.miny)/cell_size), 0, rows-1);

        hash[r*cols+c].push_back(z);
    }

    // For each grid cell, sort elevations and detect local minimum, rejecting
    // low outliers.
    MatrixXd ZImin(rows, cols);
    ZImin.setConstant(std::numeric_limits<double>::quiet_NaN());
    for (int c = 0; c < cols; ++c)
    {
        for (int r = 0; r < rows; ++r)
        {
            std::vector<double> cp(hash[r*cols+c]);
            if (cp.empty())
                continue;
            std::sort(cp.begin(), cp.end());
            if (cp.size() == 1)
            {
                ZImin(r, c) = cp[0];
                continue;
            }
            for (size_t i = 0; i < cp.size()-1; ++i)
            {
                if (std::fabs(cp[i] - cp[i+1]) < 1.0)
                {
                    ZImin(r, c) = cp[i];
                    break;
                }
            }
        }
    }

    return ZImin;
}

void dilateDiamond(std::vector<double>& data, size_t rows, size_t cols, int iterations)
{
    std::vector<double> out(data.size(), std::numeric_limits<double>::lowest());
    std::array<size_t, 5> idx;

    for (int iter = 0; iter < iterations; ++iter)
    {
        for (size_t col = 0; col < cols; ++col)
        {
            size_t index = col*rows;
            for (size_t row = 0; row < rows; ++row)
            {
                // Find the index into the vector of the current cell.  Then
                // find the index of the cells to the right/left/above/below
                // if they exist.
                size_t j = 0;
                idx[j++] = index+row;
                if (row > 0)
                    idx[j++] = idx[0]-1;
                if (row < rows-1)
                    idx[j++] = idx[0]+1;
                if (col > 0)
                    idx[j++] = idx[0]-rows;
                if (col < cols-1)
                    idx[j++] = idx[0]+rows;
                // If the data at the test cell pos is greater than that
                // from the last iteration, set the value to the maximum of
                // the value of those cells.
                for (size_t i = 0; i < j; ++i)
                {
                    if (data[idx[i]] > out[index+row])
                        out[index+row] = data[idx[i]];
                }
            }
        }
        data.swap(out);
    }
}

void erodeDiamond(std::vector<double>& data, size_t rows, size_t cols,
    int iterations)
{
    std::vector<double> out(data.size(), (std::numeric_limits<double>::max)());
    std::array<size_t, 5> idx;

    for (int iter = 0; iter < iterations; ++iter)
    {
        for (size_t col = 0; col < cols; ++col)
        {
            size_t index = col*rows;
            for (size_t row = 0; row < rows; ++row)
            {
                size_t j = 0;
                idx[j++] = index+row;
                if (row > 0)
                    idx[j++] = idx[0]-1;
                if (row < rows-1)
                    idx[j++] = idx[0]+1;
                if (col > 0)
                    idx[j++] = idx[0]-rows;
                if (col < cols-1)
                    idx[j++] = idx[0]+rows;
                for (size_t i = 0; i < j; ++i)
                {
                    if (data[idx[i]] < out[index+row])
                        out[index+row] = data[idx[i]];
                }
            }
        }
        data.swap(out);
    }
}

Eigen::MatrixXd pointViewToEigen(const PointView& view)
{
    Eigen::MatrixXd matrix(view.size(), 3);
    for (PointId i = 0; i < view.size(); ++i)
    {
        matrix(i, 0) = view.getFieldAs<double>(Dimension::Id::X, i);
        matrix(i, 1) = view.getFieldAs<double>(Dimension::Id::Y, i);
        matrix(i, 2) = view.getFieldAs<double>(Dimension::Id::Z, i);
    }
    return matrix;
}

Eigen::MatrixXd pointViewToEigen(const PointView& view, const PointIdList& ids)
{
    Eigen::MatrixXd matrix(ids.size(), 3);
    for (size_t i = 0; i < ids.size(); ++i)
    {
        matrix(i, 0) = view.getFieldAs<double>(Dimension::Id::X, ids[i]);
        matrix(i, 1) = view.getFieldAs<double>(Dimension::Id::Y, ids[i]);
        matrix(i, 2) = view.getFieldAs<double>(Dimension::Id::Z, ids[i]);
    }

    return matrix;
}

void writeMatrix(Eigen::MatrixXd data, const std::string& filename,
                 const std::string& driver, double cell_size, BOX2D bounds,
                 SpatialReference srs)
{
    std::array<double, 6> pixelToPos;
    pixelToPos[0] = bounds.minx;
    pixelToPos[1] = cell_size;
    pixelToPos[2] = 0.0;
    pixelToPos[3] = bounds.miny;
    pixelToPos[4] = 0.0;
    pixelToPos[5] = cell_size;
    gdal::Raster raster(filename, driver, srs, pixelToPos);

    gdal::GDALError err = raster.open(data.cols(), data.rows(), 1,
                                      Dimension::Type::Float, -9999.0);

    if (err != gdal::GDALError::None)
        throw pdal_error(raster.errorMsg());

    // Two things going on here. First, Eigen defaults to column major order,
    // but GDALUtils expects row major, so we can convert it. Also, double
    // doesn't seem to work for some reason, so maybe we go back and make the
    // incoming matrix always be a float, but for now just cast it.
    using namespace Eigen;
    Eigen::Matrix<float, Dynamic, Dynamic, RowMajor> dataRowMajor;
    dataRowMajor = data.cast<float>();

    raster.writeBand((float*)dataRowMajor.data(), -9999.0f, 1);
}
#pragma warning (pop)

Eigen::Vector3d rotate(const Eigen::Vector3d& v, const Eigen::Quaterniond& rot)
{
    Eigen::Quaterniond p;
    p.w() = 0;
    p.vec() = v;
    p = rot * p * rot.inverse();
    return p.vec();
}

// https://en.wikipedia.org/wiki/Barycentric_coordinate_system
// http://blackpawn.com/texts/pointinpoly/default.html
//
/// Return the interpolated Z value at location X/Y. If the X/Y location is not
/// in the triangle, return infinity.  If the determinant is 0, the input points
/// aren't a triangle (they're collinear).
/// \param x1, y1, z1  Coordinates of point 1.
/// \param x2, y2, z2  Coordinates of point 2.
/// \param x3, y3, z3  Coordinates of point 3.
/// \param x, y  X and Y coordinates of location to find an interpolated Z
/// \return  Interpolated Z value or infinity.
double barycentricInterpolation(double x1, double y1, double z1,
    double x2, double y2, double z2, double x3, double y3, double z3,
    double x, double y)
{
    double z = std::numeric_limits<double>::infinity();

    double detT = ((y2-y3) * (x1-x3)) + ((x3-x2) * (y1-y3));

    //ABELL - should probably check something close to 0, rather than
    // exactly 0.
    if (detT != 0.0)
    {
        // Compute the barycentric coordinates of x,y (relative to
        // x1/y1, x2/y2, x3/y3).  Essentially the weight that each
        // corner of the triangle contributes to the point in question.

        // Another way to think about this is that we're making a basis
        // for the system with the basis vectors being two sides of
        // the triangle.  You can rearrange the z calculation below in
        // terms of lambda1 and lambda2 to see this.  Also note that
        // since lambda1 and lambda2 are coefficients of the basis vectors,
        // any values outside of the range [0,1] are necessarily out of the
        // triangle.
        double lambda1 = ((y2-y3) * (x-x3) + (x3-x2) * (y-y3)) / detT;
        double lambda2 = ((y3-y1) * (x-x3) + (x1-x3) * (y-y3)) / detT;
        if (lambda1 >= 0 && lambda1 <= 1 && lambda2 >= 0 && lambda2 <= 1)
        {
            double sum = lambda1 + lambda2;
            if (sum <= 1)
            {
                double lambda3 = 1 - sum;
                z = (lambda1 * z1) + (lambda2 * z2) + (lambda3 * z3);
            }
        }
    }
    return z;
}

} // namespace math
} // namespace pdal
