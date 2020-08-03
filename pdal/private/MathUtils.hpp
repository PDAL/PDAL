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

#pragma once

#include <pdal/pdal_internal.hpp>
#include <pdal/Metadata.hpp>

#if (__GNUC__ > 9)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-copy"
#endif

#include <Eigen/Dense>
#include <Eigen/Geometry>

#if (__GNUC__ > 9)
#pragma GCC diagnostic pop
#endif  // GNUC

#include <memory>
#include <vector>

namespace pdal
{

class BOX2D;
class PointView;
class SpatialReference;

typedef std::shared_ptr<PointView> PointViewPtr;

namespace math
{

PointViewPtr demeanPointView(const PointView& view);
PointViewPtr demeanPointView(const PointView& ,double* centroid);
PointViewPtr transform(const PointView&, double* matrix);
void transformInPlace(PointView&, double* matrix);
double barycentricInterpolation(double x1, double y1, double z1,
    double x2, double y2, double z2, double x3, double y3, double z3,
    double x, double y);

/**
  Compute the centroid of a collection of points.

  Computes the 3D centroid of a collection of points (specified by PointId)
  sampled from the input PointView.

  \code
  // build 3D kd-tree
  KD3Index kdi(view);
  kdi.build();

  // find the k-nearest neighbors of the first point (k=8)
  auto ids = kdi.neighbors(0, 8);

  // compute the centroid
  auto centroid = computeCentroid(view, ids);
  \endcode

  \param view the source PointView.
  \param ids a vector of PointIds specifying a subset of points.
  \return the 3D centroid of the XYZ dimensions.
*/
Eigen::Vector3d computeCentroid(const PointView& view,
    const PointIdList& ids);

/**
    Rotate a point using the given quaternion.
    NOTE: quaternion must be normalized before this call.

    \param p     Point to rotate about origin.
    \param quat  Quaternion specification for rotation.
    \return  Coordinates of rotated point.
*/
Eigen::Vector3d rotate(const Eigen::Vector3d& p, const Eigen::Quaterniond& rot);

/**
  Compute the covariance matrix of a collection of points.

  Computes the covariance matrix of a collection of points (specified by
  PointId) sampled from the input PointView.

  \code
  // build 3D kd-tree
  KD3Index kdi(view);
  kdi.build();

  // find the k-nearest neighbors of the first point (k=8)
  auto ids = kdi.neighbors(0, 8);

  // compute the covariance
  auto cov = computeCovariance(view, ids);
  \endcode

  \param view the source PointView.
  \param ids a vector of PointIds specifying a subset of points.
  \return the covariance matrix of the XYZ dimensions.
*/
Eigen::Matrix3d computeCovariance(const PointView& view,
    const PointIdList& ids);

/**
  Compute the rank of a collection of points.

  Computes the rank of a collection of points (specified by PointId) sampled
  from the input PointView. This method uses Eigen's JacobiSVD class to solve
  the singular value decomposition and to estimate the rank using the given
  threshold. A singular value will be considered nonzero if its absolute value
  is greater than the product of the user-supplied threshold and the absolute
  value of the maximum singular value.

  More on JacobiSVD can be found at
  https://eigen.tuxfamily.org/dox/classEigen_1_1JacobiSVD.html.

  \code
  // build 3D kd-tree
  KD3Index kdi(view);
  kdi.build();

  // find the k-nearest neighbors of the first point (k=8)
  auto ids = kdi.neighbors(0, 8);

  // compute the rank using threshold of 0.01
  auto rank = computeRank(view, ids, 0.01);
  \endcode

  \param view the source PointView.
  \param ids a vector of PointIds specifying a subset of points.
  \return the estimated rank.
*/
uint8_t computeRank(const PointView& view,
    const PointIdList& ids, double threshold);

/**
  Find local minimum elevations by extended local minimum.

  Extended local minimum can be used to select seed points for ground return
  segmentation. Several low-lying points are considered for each grid cell. The
  difference between the lowest and second lowest points is evaluated and, if
  the difference exceeds 1.0, the lowest points is considered an outlier. The
  process continues with the next pair of lowest points (second and third
  lowest). When the points under consideration are within the given tolerance,
  the lowest is retained as a seed point.

  \param view the input PointView.
  \param rows the number of rows.
  \param cols the cnumber of columns.
  \param cell_size the edge length of raster cell.
  \param bounds the 2D bounds of the PointView.
  \return the matrix of minimum Z values (ignoring low outliers).
*/
Eigen::MatrixXd extendedLocalMinimum(const PointView& view, int rows,
    int cols, double cell_size, BOX2D bounds);

/**
  Perform a morphological dilation of the input raster.

  Performs a morphological dilation of the input raster using a diamond
  structuring element. Larger structuring elements are approximated by applying
  multiple iterations of the opening operation. The input and output rasters are
  stored in column major order.

  \param data the input raster.
  \param rows the number of rows.
  \param cols the number of cols.
  \param iterations the number of iterations used to approximate a larger
         structuring element.
  \return the morphological dilation of the input raster.
*/
void dilateDiamond(std::vector<double>& data, size_t rows, size_t cols, int iterations);

/**
  Perform a morphological erosion of the input raster.

  Performs a morphological erosion of the input raster using a diamond
  structuring element. Larger structuring elements are approximated by applying
  multiple iterations of the opening operation. The input and output rasters are
  stored in column major order.

  \param data the input raster.
  \param rows the number of rows.
  \param cols the number of cols.
  \param iterations the number of iterations used to approximate a larger
         structuring element.
  \return the morphological erosion of the input raster.
*/
void erodeDiamond(std::vector<double>& data, size_t rows, size_t cols, int iterations);

/**
  Converts a PointView into an Eigen::MatrixXd.

  This method exists (as of this writing) purely as a convenience method in the
  API. It is not currently used in the PDAL codebase itself.
*/
Eigen::MatrixXd pointViewToEigen(const PointView& view);
Eigen::MatrixXd pointViewToEigen(const PointView& view, const PointIdList& ids);

/**
  Write Eigen Matrix as a GDAL raster.

  \param data the Eigen matrix to write.
  \param filename the filename of the output raster.
  \param cell_size the edge length of raster cell.
  \param bounds the 2D bounds of the data.
  \param srs the spatial reference system of the data.
*/
void writeMatrix(Eigen::MatrixXd data, const std::string& filename,
                          const std::string& driver, double cell_size,
                          BOX2D bounds, SpatialReference srs);

/**
  Compute the numerical gradient in the X direction.

  This is meant to mimic MATLAB's gradient function. The spacing between points
  in each direction is assumed to be one.

  \param data the input matrix.
  \return the X component of the two-dimensional gradient.
*/
template <typename Derived>
Derived gradX(const Eigen::MatrixBase<Derived>& A)
{
    Derived out = Derived::Zero(A.rows(), A.cols());

    // Interior points are obtained by central differences.
    out.block(0, 1, A.rows(), A.cols()-2) =
        0.5 * (A.rightCols(A.cols()-2) - A.leftCols(A.cols()-2));

    // Edge columns are obtained by single-sided differences.
    out.col(0) = A.col(1) - A.col(0);
    out.col(out.cols()-1) = A.col(A.cols()-1) - A.col(A.cols()-2);

    return out;
}

/**
  Compute the numerical gradient in the Y direction.

  This is meant to mimic MATLAB's gradient function. The spacing between points
  in each direction is assumed to be one.

  \param data the input matrix.
  \return the Y component of the two-dimensional gradient.
*/
template <typename Derived>
Derived gradY(const Eigen::MatrixBase<Derived>& A)
{
    Derived out = Derived::Zero(A.rows(), A.cols());

    // Interior points are obtained by central differences.
    out.block(1, 0, A.rows()-2, A.cols()) =
        0.5 * (A.bottomRows(A.rows()-2) - A.topRows(A.rows()-2));

    // Edge rows are obtained by single-sided differences.
    out.row(0) = A.row(1) - A.row(0);
    out.row(out.rows()-1) = A.row(A.rows()-1) - A.row(A.rows()-2);

    return out;
}

} // namespace math

namespace Utils
{

template <>
inline StatusWithReason fromString(const std::string& s,
    Eigen::MatrixXd& matrix)
{
    std::stringstream ss(s);
    std::string line;
    std::vector<std::vector<double>> rows;
    while (std::getline(ss, line)) {
        std::vector<double> row;
        std::stringstream ss(line);
        double n;
        while (ss >> n) {
            row.push_back(n);
            if (ss.peek() == ',' || ss.peek() == ' ') {
                ss.ignore();
            }
        }
        if (!rows.empty() && rows.back().size() != row.size()) {
            return false;
        }
        rows.push_back(row);
    }
    if (rows.empty()) {
        return true;
    }
    size_t nrows = rows.size();
    size_t ncols = rows[0].size();
    matrix.resize(nrows, ncols);
    for (size_t i = 0; i < nrows; ++i) {
        for (size_t j = 0; j < ncols; ++j) {
            matrix(i, j) = rows[i][j];
        }
    }
    return true;
}

} // namespace Utils

template <>
inline void MetadataNodeImpl::setValue(const Eigen::MatrixXd& matrix)
{
    m_type = "matrix";
    m_value = Utils::toString(matrix);
}

} // namespace pdal
