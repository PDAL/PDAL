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

#include <pdal/EigenUtils.hpp>
#include <pdal/GDALUtils.hpp>
#include <pdal/KDIndex.hpp>
#include <pdal/PointView.hpp>
#include <pdal/SpatialReference.hpp>
#include <pdal/util/Bounds.hpp>
#include <pdal/util/Utils.hpp>

#include <Eigen/Dense>

#include <cfloat>
#include <numeric>
#include <vector>

namespace pdal
{

namespace eigen
{
#pragma warning (push)
#pragma warning (disable: 4244)

Eigen::Vector3d computeCentroid(PointView& view,
    const std::vector<PointId>& ids)
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

Eigen::Matrix3f computeCovariance(PointView& view,
    const std::vector<PointId>& ids)
{
    using namespace Eigen;

    auto n = ids.size();

    Vector3d centroid = computeCentroid(view, ids);

    // demean the neighborhood
    MatrixXf A(3, n);
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

uint8_t computeRank(PointView& view, const std::vector<PointId>& ids,
    double threshold)
{
    using namespace Eigen;

    Matrix3f B = computeCovariance(view, ids);

    JacobiSVD<Matrix3f> svd(B);
    svd.setThreshold((float)threshold);

    return static_cast<uint8_t>(svd.rank());
}

Eigen::MatrixXd computeSpline(Eigen::MatrixXd x, Eigen::MatrixXd y,
                              Eigen::MatrixXd z, Eigen::MatrixXd xx,
                              Eigen::MatrixXd yy)
{
    using namespace Eigen;

    int num_rows;
    int num_cols;

    if (!Utils::numericCast(xx.rows(), num_rows) ||
        !Utils::numericCast(xx.cols(), num_cols))
        throw pdal_error("Too many columns/rows for spline computation");

    MatrixXd S = MatrixXd::Zero(num_rows, num_cols);

    for (auto col = 0; col < num_cols; ++col)
    {
        for (auto row = 0; row < num_rows; ++row)
        {
            // Further optimizations are achieved by estimating only the
            // interpolated surface within a local neighbourhood (e.g. a 7 x 7
            // neighbourhood is used in our case) of the cell being filtered.
            int radius = 3;

            int c = static_cast<int>(std::floor(col/2));
            int r = static_cast<int>(std::floor(row/2));

            int cs = Utils::clamp(c-radius, 0, static_cast<int>(z.cols()-1));
            int ce = Utils::clamp(c+radius, 0, static_cast<int>(z.cols()-1));
            int col_size = ce - cs + 1;
            int rs = Utils::clamp(r-radius, 0, static_cast<int>(z.rows()-1));
            int re = Utils::clamp(r+radius, 0, static_cast<int>(z.rows()-1));
            int row_size = re - rs + 1;

            MatrixXd Xn = x.block(rs, cs, row_size, col_size);
            MatrixXd Yn = y.block(rs, cs, row_size, col_size);
            MatrixXd Hn = z.block(rs, cs, row_size, col_size);

            int nsize = row_size * col_size;
            VectorXd T = VectorXd::Zero(nsize);
            MatrixXd P = MatrixXd::Zero(nsize, 3);
            MatrixXd K = MatrixXd::Zero(nsize, nsize);

            for (auto id = 0; id < nsize; ++id)
            {
                double xj = Xn(id);
                double yj = Yn(id);
                double zj = Hn(id);
                if (std::isnan(xj) || std::isnan(yj) || std::isnan(zj))
                    continue;
                T(id) = zj;
                P.row(id) << 1, xj, yj;
                for (auto id2 = 0; id2 < nsize; ++id2)
                {
                    if (id == id2)
                        continue;
                    double xk = Xn(id2);
                    double yk = Yn(id2);
                    double zk = Hn(id2);
                    if (std::isnan(xk) || std::isnan(yk) || std::isnan(zk))
                        continue;
                    double rsqr = (xj - xk) * (xj - xk) + (yj - yk) * (yj - yk);
                    if (rsqr == 0.0)
                        continue;
                    K(id, id2) = rsqr * std::log10(std::sqrt(rsqr));
                }
            }

            MatrixXd A = MatrixXd::Zero(nsize+3, nsize+3);
            A.block(0,0,nsize,nsize) = K;
            A.block(0,nsize,nsize,3) = P;
            A.block(nsize,0,3,nsize) = P.transpose();

            VectorXd b = VectorXd::Zero(nsize+3);
            b.head(nsize) = T;

            VectorXd x = A.fullPivHouseholderQr().solve(b);

            Vector3d a = x.tail(3);
            VectorXd w = x.head(nsize);

            double sum = 0.0;
            double xi2 = xx(row, col);
            double yi2 = yy(row, col);
            for (auto j = 0; j < nsize; ++j)
            {
                double xj = Xn(j);
                double yj = Yn(j);
                double zj = Hn(j);
                if (std::isnan(xj) || std::isnan(yj) || std::isnan(zj))
                    continue;
                double rsqr = (xj - xi2) * (xj - xi2) + (yj - yi2) * (yj - yi2);
                if (rsqr == 0.0)
                    continue;
                sum += w(j) * rsqr * std::log10(std::sqrt(rsqr));
            }

            S(row, col) = a(0) + a(1)*xi2 + a(2)*yi2 + sum;
        }
    }

    return S;
}

Eigen::MatrixXd createMinMatrix(PointView& view, int rows, int cols,
                                double cell_size, BOX2D bounds)
{
    using namespace Dimension;
    using namespace Eigen;

    MatrixXd ZImin(rows, cols);
    ZImin.setConstant(std::numeric_limits<double>::quiet_NaN());

    for (PointId i = 0; i < view.size(); ++i)
    {
        double x = view.getFieldAs<double>(Id::X, i);
        double y = view.getFieldAs<double>(Id::Y, i);
        double z = view.getFieldAs<double>(Id::Z, i);

        int c = Utils::clamp(static_cast<int>(floor(x-bounds.minx)/cell_size), 0, cols-1);
        int r = Utils::clamp(static_cast<int>(floor(y-bounds.miny)/cell_size), 0, rows-1);

        if (z < ZImin(r, c) || std::isnan(ZImin(r, c)))
            ZImin(r, c) = z;
    }

    return ZImin;
}

Eigen::MatrixXd createMaxMatrix(PointView& view, int rows, int cols,
                                double cell_size, BOX2D bounds)
{
    using namespace Dimension;
    using namespace Eigen;

    MatrixXd ZImax(rows, cols);
    ZImax.setConstant(std::numeric_limits<double>::quiet_NaN());

    for (PointId i = 0; i < view.size(); ++i)
    {
        double x = view.getFieldAs<double>(Id::X, i);
        double y = view.getFieldAs<double>(Id::Y, i);
        double z = view.getFieldAs<double>(Id::Z, i);

        int c = Utils::clamp(static_cast<int>(floor(x-bounds.minx)/cell_size), 0, cols-1);
        int r = Utils::clamp(static_cast<int>(floor(y-bounds.miny)/cell_size), 0, rows-1);

        if (z > ZImax(r, c) || std::isnan(ZImax(r, c)))
            ZImax(r, c) = z;
    }

    return ZImax;
}

Eigen::MatrixXd createMaxMatrix2(PointView& view, int rows, int cols,
                                 double cell_size, BOX2D bounds)
{
    using namespace Dimension;
    using namespace Eigen;

    KD2Index kdi(view);
    kdi.build();

    MatrixXd ZImax(rows, cols);
    ZImax.setConstant(std::numeric_limits<double>::quiet_NaN());

    // for each grid center, search PointView for neighbors, and find max of those
    for (int c = 0; c < cols; ++c)
    {
        double x = bounds.minx + (c + 0.5) * cell_size;

        for (int r = 0; r < rows; ++r)
        {
            double y = bounds.miny + (r + 0.5) * cell_size;

            auto neighbors = kdi.radius(x, y, cell_size * std::sqrt(2.0));

            double val(std::numeric_limits<double>::lowest());
            for (auto const& n : neighbors)
            {
                double z(view.getFieldAs<double>(Id::Z, n));
                if (z > val)
                    val = z;
            }
            if (val > std::numeric_limits<double>::lowest())
                ZImax(r, c) = val;
        }
    }

    return ZImax;
}

Eigen::MatrixXd extendedLocalMinimum(PointView& view, int rows, int cols,
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


Eigen::MatrixXd matrixClose(Eigen::MatrixXd data, int radius)
{
    using namespace Eigen;

    MatrixXd data2 = padMatrix(data, radius);

    int nrows = static_cast<int>(data2.rows());
    int ncols = data2.cols();

    MatrixXd minZ(nrows, ncols);
    minZ.setConstant((std::numeric_limits<double>::max)());
    MatrixXd maxZ(nrows, ncols);
    maxZ.setConstant(std::numeric_limits<double>::lowest());
    for (auto c = 0; c < ncols; ++c)
    {
        int cs = Utils::clamp(c-radius, 0, ncols-1);
        int ce = Utils::clamp(c+radius, 0, ncols-1);

        for (auto r = 0; r < nrows; ++r)
        {
            int rs = Utils::clamp(r-radius, 0, nrows-1);
            int re = Utils::clamp(r+radius, 0, nrows-1);

            for (auto col = cs; col <= ce; ++col)
            {
                for (auto row = rs; row <= re; ++row)
                {
                    if ((row-r)*(row-r)+(col-c)*(col-c) > radius*radius)
                        continue;
                    if (data2(row, col) > maxZ(r, c))
                        maxZ(r, c) = data2(row, col);
                }
            }
        }
    }
    for (auto c = 0; c < ncols; ++c)
    {
        int cs = Utils::clamp(c-radius, 0, ncols-1);
        int ce = Utils::clamp(c+radius, 0, ncols-1);

        for (auto r = 0; r < nrows; ++r)
        {
            int rs = Utils::clamp(r-radius, 0, nrows-1);
            int re = Utils::clamp(r+radius, 0, nrows-1);

            for (auto col = cs; col <= ce; ++col)
            {
                for (auto row = rs; row <= re; ++row)
                {
                    if ((row-r)*(row-r)+(col-c)*(col-c) > radius*radius)
                        continue;
                    if (maxZ(row, col) < minZ(r, c))
                        minZ(r, c) = maxZ(row, col);
                }
            }
        }
    }

    return minZ.block(radius, radius, data.rows(), data.cols());
}

Eigen::MatrixXd matrixOpen(Eigen::MatrixXd data, int radius)
{
    using namespace Eigen;

    MatrixXd data2 = padMatrix(data, radius);

    int nrows = data2.rows();
    int ncols = data2.cols();

    MatrixXd minZ(nrows, ncols);
    minZ.setConstant((std::numeric_limits<double>::max)());
    MatrixXd maxZ(nrows, ncols);
    maxZ.setConstant(std::numeric_limits<double>::lowest());
    for (auto c = 0; c < ncols; ++c)
    {
        int cs = Utils::clamp(c-radius, 0, ncols-1);
        int ce = Utils::clamp(c+radius, 0, ncols-1);

        for (auto r = 0; r < nrows; ++r)
        {
            int rs = Utils::clamp(r-radius, 0, nrows-1);
            int re = Utils::clamp(r+radius, 0, nrows-1);

            for (auto col = cs; col <= ce; ++col)
            {
                for (auto row = rs; row <= re; ++row)
                {
                    if ((row-r)*(row-r)+(col-c)*(col-c) > radius*radius)
                        continue;
                    if (data2(row, col) < minZ(r, c))
                        minZ(r, c) = data2(row, col);
                }
            }
        }
    }
    for (auto c = 0; c < ncols; ++c)
    {
        int cs = Utils::clamp(c-radius, 0, ncols-1);
        int ce = Utils::clamp(c+radius, 0, ncols-1);

        for (auto r = 0; r < nrows; ++r)
        {
            int rs = Utils::clamp(r-radius, 0, nrows-1);
            int re = Utils::clamp(r+radius, 0, nrows-1);

            for (auto col = cs; col <= ce; ++col)
            {
                for (auto row = rs; row <= re; ++row)
                {
                    if ((row-r)*(row-r)+(col-c)*(col-c) > radius*radius)
                        continue;
                    if (minZ(row, col) > maxZ(r, c))
                        maxZ(r, c) = minZ(row, col);
                }
            }
        }
    }

    return maxZ.block(radius, radius, data.rows(), data.cols());
}

std::vector<double> dilateDiamond(std::vector<double> data, size_t rows, size_t cols, int iterations)
{
    std::vector<double> out(data.size(), std::numeric_limits<double>::lowest());
    std::vector<size_t> idx(5);

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
    return data;
}

std::vector<double> erodeDiamond(std::vector<double> data, size_t rows, size_t cols, int iterations)
{
    std::vector<double> out(data.size(), (std::numeric_limits<double>::max)());
    std::vector<size_t> idx(5);

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
    return data;
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

void writeMatrix(Eigen::MatrixXd data, const std::string& filename,
                 const std::string& driver, double cell_size, BOX2D bounds,
                 SpatialReference srs)
{
    using namespace Eigen;

    gdal::registerDrivers();

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
    Eigen::Matrix<float, Dynamic, Dynamic, RowMajor> dataRowMajor;
    dataRowMajor = data.cast<float>();

    raster.writeBand((float*)dataRowMajor.data(), -9999.0f, 1);
}

Eigen::MatrixXd cleanDSM(Eigen::MatrixXd data)
{
    using namespace Eigen;

    auto CleanRasterScanLine = [](Eigen::MatrixXd data, Eigen::VectorXd datarow,
                                  int mDim, int row, bool* prevSetCols,
                                  bool* curSetCols)
    {
        auto InterpolateRasterPixelScanLine = [](Eigen::MatrixXd data, int mDim,
                                              int x, int y, bool* prevSetCols)
        {
            const float c_background = FLT_MIN;
            int yMinus, yPlus, xMinus, xPlus;
            float tInterpValue;
            bool tPrevInterp;

            yMinus = y - 1;
            yPlus = y + 1;
            xMinus = x - 1;
            xPlus = x + 1;

            //North
            tInterpValue = data(yMinus, x);
            tPrevInterp = prevSetCols[x];
            if (tInterpValue != c_background && tPrevInterp != true)
                return tInterpValue;

            //South
            tInterpValue = data(yPlus, x);
            if (tInterpValue != c_background)
                return tInterpValue;

            //East
            tInterpValue = data(y, xPlus);
            if (tInterpValue != c_background)
                return tInterpValue;

            //West
            tInterpValue = data(y, xMinus);
            if (tInterpValue != c_background)
                return tInterpValue;

            //NorthWest
            tInterpValue = data(yMinus, xMinus);
            tPrevInterp = prevSetCols[xMinus];
            if (tInterpValue != c_background && tPrevInterp != true)
                return tInterpValue;

            //NorthWest
            tInterpValue = data(yMinus, xPlus);
            tPrevInterp = prevSetCols[xPlus];
            if (tInterpValue != c_background && tPrevInterp != true)
                return tInterpValue;

            //SouthWest
            tInterpValue = data(yPlus, xMinus);
            if (tInterpValue != c_background)
                return tInterpValue;

            //SouthEast
            tInterpValue = data(yPlus, xPlus);
            if (tInterpValue != c_background)
                return tInterpValue;

            return 0.0f;
        };

        float tInterpValue;
        float tValue;

        int y = row;
        for (int x = 1; x < mDim-1; ++x)
        {
            tValue = datarow(x);
            const float c_background = FLT_MIN;

            if (tValue == c_background)
            {
                tInterpValue = InterpolateRasterPixelScanLine(data, mDim, x, y,
                               prevSetCols);
                if (tInterpValue != c_background)
                {
                    curSetCols[x] = true;
                    datarow(x) = tInterpValue;
                }
            }
        }
    };

    {
        int rows = data.rows();
        int cols = data.cols();

        std::unique_ptr<bool> prevSetCols(new bool[cols]);
        std::unique_ptr<bool> curSetCols(new bool[cols]);

        for (int y = 1; y < rows-1; ++y)
        {
            CleanRasterScanLine(data, data.row(1), cols, y,
                                prevSetCols.get(), curSetCols.get());
            memcpy(prevSetCols.get(), curSetCols.get(), cols);
            memset(curSetCols.get(), 0, cols);
        }
    }

    return data;
}
#pragma warning (pop)

} // namespace eigen

} // namespace pdal
