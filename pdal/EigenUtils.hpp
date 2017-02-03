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
#include <pdal/util/Bounds.hpp>

#include <Eigen/Dense>

#include <memory>
#include <vector>

namespace pdal
{
class PointView;
class SpatialReference;

typedef std::shared_ptr<PointView> PointViewPtr;

namespace eigen
{

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
PDAL_DLL Eigen::Vector3f computeCentroid(PointView& view,
        std::vector<PointId> ids);

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
PDAL_DLL Eigen::Matrix3f computeCovariance(PointView& view,
        std::vector<PointId> ids);

/**
  Compute second derivative in X direction using central difference method.

  \param data the incoming Eigen matrix.
  \param spacing the grid spacing of the matrix.
  \return the second derivative in X.
*/
template <typename Derived>
PDAL_DLL double centralDiffX2(const Eigen::MatrixBase<Derived>& data,
                              double spacing)
{
    if (data.rows() != 3 || data.cols() != 3)
        throw pdal_error("Must provide 3x3 matrix to centralDiffX2");
    if (spacing <= 0.0)
        throw pdal_error("Must provide positive spacing to centralDiffX2");
    return (data(1, 2) - 2.0 * data(1, 1) + data(1, 0)) / (spacing * spacing);
}

/**
  Compute second derivative in Y direction using central difference method.

  \param data the incoming Eigen matrix.
  \param spacing the grid spacing of the matrix.
  \return the second derivative in Y.
*/
template <typename Derived>
PDAL_DLL double centralDiffY2(const Eigen::MatrixBase<Derived>& data,
                              double spacing)
{
    if (data.rows() != 3 || data.cols() != 3)
        throw pdal_error("Must provide 3x3 matrix to centralDiffY2");
    if (spacing <= 0.0)
        throw pdal_error("Must provide positive spacing to centralDiffY2");
    return (data(0, 1) - 2.0 * data(1, 1) + data(2, 1)) / (spacing * spacing);
}

/**
  Compute mixed second derivative of X in the Y direction using central
  difference method.

  \param data the incoming Eigen matrix.
  \param spacing the grid spacing of the matrix.
  \return the second derivative of X in the Y direction.
*/
template <typename Derived>
PDAL_DLL double centralDiffXY(const Eigen::MatrixBase<Derived>& data,
                              double spacing)
{
    if (data.rows() != 3 || data.cols() != 3)
        throw pdal_error("Must provide 3x3 matrix to centralDiffXY");
    if (spacing <= 0.0)
        throw pdal_error("Must provide positive spacing to centralDiffXY");
    return ((-1.0 * data(0, 0)) + data(0, 2) + data(2, 0) - data(2, 2)) /
           (4.0 * spacing * spacing);
}

/**
  Compute first derivative in X direction using central difference method.

  \param data the incoming Eigen matrix.
  \param spacing the grid spacing of the matrix.
  \return the first derivative in X.
*/
template <typename Derived>
PDAL_DLL double centralDiffX(const Eigen::MatrixBase<Derived>& data,
                             double spacing)
{
    if (data.rows() != 3 || data.cols() != 3)
        throw pdal_error("Must provide 3x3 matrix to centralDiffX");
    if (spacing <= 0.0)
        throw pdal_error("Must provide positive spacing to centralDiffX");
    return (data(1, 2) - data(1, 0)) / (2 * spacing);
}

/**
  Compute first derivative in Y direction using central difference method.

  \param data the incoming Eigen matrix.
  \param spacing the grid spacing of the matrix.
  \return the first derivative in Y.
*/
template <typename Derived>
PDAL_DLL double centralDiffY(const Eigen::MatrixBase<Derived>& data,
                             double spacing)
{
    if (data.rows() != 3 || data.cols() != 3)
        throw pdal_error("Must provide 3x3 matrix to centralDiffY");
    if (spacing <= 0.0)
        throw pdal_error("Must provide positive spacing to centralDiffY");
    return (data(0, 1) - data(2, 1)) / (2 * spacing);
}

/**
  Clean a raster.

  \param data the Eigen matrix to clean.
  \return the cleaned raster.
*/
PDAL_DLL Eigen::MatrixXd cleanDSM(Eigen::MatrixXd data);

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
PDAL_DLL uint8_t computeRank(PointView& view, std::vector<PointId> ids,
                             double threshold);

/**
  Create matrix of maximum Z values.

  Create a DSM from the provided PointVieew, where each cell contains the
  maximum Z value of all contributing elevations.

  \param view the input PointView.
  \param rows the number of rows.
  \param cols the number of columns.
  \param cell_size the edge length of raster cell.
  \param bounds the 2D bounds of the PointView.
  \return the matrix of maximum Z values.
*/
PDAL_DLL Eigen::MatrixXd createMaxMatrix(PointView& view, int rows, int cols,
        double cell_size, BOX2D bounds);

PDAL_DLL Eigen::MatrixXd createMaxMatrix2(PointView& view, int rows, int cols,
        double cell_size, BOX2D bounds);

/**
  Create matrix of minimum Z values.

  Create a DTM from the provided PointVieew, where each cell contains the
  minimum Z value of all contributing elevations.

  \param view the input PointView.
  \param rows the number of rows.
  \param cols the number of columns.
  \param cell_size the edge length of raster cell.
  \param bounds the 2D bounds of the PointView.
  \return the matrix of minimum Z values.
*/
PDAL_DLL Eigen::MatrixXd createMinMatrix(PointView& view, int rows, int cols,
        double cell_size, BOX2D bounds);

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
PDAL_DLL Eigen::MatrixXd extendedLocalMinimum(PointView& view, int rows,
        int cols, double cell_size, BOX2D bounds);

/**
  Perform a morphological closing of the input matrix.

  Performs a morphological closing of the input matrix using a circular
  structuring element of given radius. Data will be symmetrically padded at its
  edges.

  \param data the input matrix.
  \param radius the radius of the circular structuring element.
  \return the morphological closing of the input radius.
*/
PDAL_DLL Eigen::MatrixXd matrixClose(Eigen::MatrixXd data, int radius);

/**
  Perform a morphological opening of the input matrix.

  Performs a morphological opening of the input matrix using a circular
  structuring element of given radius. Data will be symmetrically padded at its
  edges.

  \param data the input matrix.
  \param radius the radius of the circular structuring element.
  \return the morphological opening of the input radius.
*/
PDAL_DLL Eigen::MatrixXd matrixOpen(Eigen::MatrixXd data, int radius);

/**
  Pad input matrix symmetrically.

  Symmetrically pads the input matrix with given radius.

  \param d the input matrix.
  \param r the radius of the padding.
  \return the padded matrix.
*/
template <typename Derived>
PDAL_DLL Derived padMatrix(const Eigen::MatrixBase<Derived>& d, int r)
{
    using namespace Eigen;

    Derived out = Derived::Zero(d.rows()+2*r, d.cols()+2*r);
    out.block(r, r, d.rows(), d.cols()) = d;
    out.block(r, 0, d.rows(), r) =
        d.block(0, 0, d.rows(), r).rowwise().reverse();
    out.block(r, d.cols()+r, d.rows(), r) =
        d.block(0, d.cols()-r, d.rows(), r).rowwise().reverse();
    out.block(0, 0, r, out.cols()) =
        out.block(r, 0, r, out.cols()).colwise().reverse();
    out.block(d.rows()+r, 0, r, out.cols()) =
        out.block(out.rows()-r-1, 0, r, out.cols()).colwise().reverse();

    return out;
}

/**
  Converts a PointView into an Eigen::MatrixXd.

  This method exists (as of this writing) purely as a convenience method in the
  API. It is not currently used in the PDAL codebase itself.
*/
PDAL_DLL Eigen::MatrixXd pointViewToEigen(const PointView& view);

/**
  Replace NaNs with mean.

  \param data the incoming Eigen matrix.
  \return the updated Eigen matrix.
*/
template <typename Derived>
PDAL_DLL Eigen::MatrixXd replaceNaNs(const Eigen::MatrixBase<Derived>& data)
{
    Derived out(data);

    double mean(0.0);
    int nv(0);
    for (int i = 0; i < data.size(); ++i)
    {
        if (std::isnan(data(i)))
            continue;
        mean += data(i);
        nv++;
    }
    mean /= nv;

    // replace NaNs with mean
    for (int i = 0; i < data.size(); ++i)
    {
        if (std::isnan(data(i)))
            out(i) = mean;
    }

    return out;
}

/**
  Write Eigen Matrix as a GDAL raster.

  \param data the Eigen matrix to write.
  \param filename the filename of the output raster.
  \param cell_size the edge length of raster cell.
  \param bounds the 2D bounds of the data.
  \param srs the spatial reference system of the data.
*/
PDAL_DLL void writeMatrix(Eigen::MatrixXd data, const std::string& filename,
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
PDAL_DLL Derived gradX(const Eigen::MatrixBase<Derived>& A)
{
    Derived out = Derived::Zero(A.rows(), A.cols());

    // Interior points are obtained by central differences.
    out.block(0, 1, A.rows(), A.cols()-2) =
        0.5 * (A.rightCols(A.cols()-2) - A.leftCols(A.cols()-2));

    // Edge columns are obtained by single-sided differences.
    out.col(0) = A.col(1) - A.col(0);
    out.col(out.cols()-1) = A.col(A.cols()-1) - A.col(A.cols()-2);

    return out;
};

/**
  Compute the numerical gradient in the Y direction.

  This is meant to mimic MATLAB's gradient function. The spacing between points
  in each direction is assumed to be one.

  \param data the input matrix.
  \return the Y component of the two-dimensional gradient.
*/
template <typename Derived>
PDAL_DLL Derived gradY(const Eigen::MatrixBase<Derived>& A)
{
    Derived out = Derived::Zero(A.rows(), A.cols());

    // Interior points are obtained by central differences.
    out.block(1, 0, A.rows()-2, A.cols()) =
        0.5 * (A.bottomRows(A.rows()-2) - A.topRows(A.rows()-2));

    // Edge rows are obtained by single-sided differences.
    out.row(0) = A.row(1) - A.row(0);
    out.row(out.rows()-1) = A.row(A.rows()-1) - A.row(A.rows()-2);

    return out;
};

/**
  Compute contour curvature for a single 3x3 matrix.

  \param data the incoming Eigen matrix.
  \param spacing the spacing between cells (or edge length of a cell).
  \return the contour curvature.
*/
template <typename Derived>
PDAL_DLL double computeContour(const Eigen::MatrixBase<Derived>& data,
                               double spacing)
{
    if (data.rows() != 3 || data.cols() != 3)
        throw pdal_error("Must provide 3x3 matrix to computeContour");
    if (spacing <= 0.0)
        throw pdal_error("Must provide positive spacing to computeContour");
    Derived filled = replaceNaNs(data);
    double zXX = centralDiffX2(filled, spacing);
    double zYY = centralDiffY2(filled, spacing);
    double zXY = centralDiffXY(filled, spacing);
    double zX = centralDiffX(filled, spacing);
    double zY = centralDiffY(filled, spacing);
    double p = (zX * zX) + (zY * zY);
    double q = p + 1;
    return ((zXX*zX*zX)-(2*zXY*zX*zY)+(zYY*zY*zY))/(p*std::sqrt(q*q*q));
}

/**
  Compute total curvature for a single 3x3 matrix.

  \param data the incoming Eigen matrix.
  \param spacing the spacing between cells (or edge length of a cell).
  \return the total curvature.
*/
template <typename Derived>
PDAL_DLL double computeTotal(const Eigen::MatrixBase<Derived>& data,
                             double spacing)
{
    if (data.rows() != 3 || data.cols() != 3)
        throw pdal_error("Must provide 3x3 matrix to computeTotal");
    if (spacing <= 0.0)
        throw pdal_error("Must provide positive spacing to computeTotal");
    Derived filled = replaceNaNs(data);
    double zXX = centralDiffX2(filled, spacing);
    double zYY = centralDiffY2(filled, spacing);
    double zXY = centralDiffXY(filled, spacing);
    return (zXX * zXX) + (2.0 * zXY * zXY) + (zYY * zYY);
}

/**
  Compute profile curvature for a single 3x3 matrix.

  \param data the incoming Eigen matrix.
  \param spacing the spacing between cells (or edge length of a cell).
  \return the profile curvature.
*/
template <typename Derived>
PDAL_DLL double computeProfile(const Eigen::MatrixBase<Derived>& data,
                               double spacing)
{
    if (data.rows() != 3 || data.cols() != 3)
        throw pdal_error("Must provide 3x3 matrix to computeProfile");
    if (spacing <= 0.0)
        throw pdal_error("Must provide positive spacing to computeProfile");
    Derived filled = replaceNaNs(data);
    double zXX = centralDiffX2(filled, spacing);
    double zYY = centralDiffY2(filled, spacing);
    double zXY = centralDiffXY(filled, spacing);
    double zX = centralDiffX(filled, spacing);
    double zY = centralDiffY(filled, spacing);
    double p = (zX * zX) + (zY * zY);
    double q = p + 1;
    return ((zXX*zX*zX)+(2*zXY*zX*zY)+(zYY*zY*zY))/(p*std::sqrt(q*q*q));
}

/**
  Compute tangential curvature for a single 3x3 matrix.

  \param data the incoming Eigen matrix.
  \param spacing the spacing between cells (or edge length of a cell).
  \return the tangential curvature.
*/
template <typename Derived>
PDAL_DLL double computeTangential(const Eigen::MatrixBase<Derived>& data,
                                  double spacing)
{
    if (data.rows() != 3 || data.cols() != 3)
        throw pdal_error("Must provide 3x3 matrix to computeTangential");
    if (spacing <= 0.0)
        throw pdal_error("Must provide positive spacing to computeTangential");
    Derived filled = replaceNaNs(data);
    double zXX = centralDiffX2(filled, spacing);
    double zYY = centralDiffY2(filled, spacing);
    double zXY = centralDiffXY(filled, spacing);
    double zX = centralDiffX(filled, spacing);
    double zY = centralDiffY(filled, spacing);
    double p = (zX * zX) + (zY * zY);
    double q = p + 1;
    return ((zXX*zY*zY)-(2*zXY*zX*zY)+(zYY*zX*zX))/(p*std::sqrt(q));
}

/**
  Compute first derivative in X using 3x3 window.

  \param data the incoming Eigen matrix.
  \param spacing the grid spacing.
  \return the first derivative in X.
*/
template <typename Derived>
PDAL_DLL double computeDZDX(const Eigen::MatrixBase<Derived>& data,
                            double spacing)
{
    if (data.rows() != 3 || data.cols() != 3)
        throw pdal_error("Must provide 3x3 matrix to computeDZDX");
    if (spacing <= 0.0)
        throw pdal_error("Must provide positive spacing to computeDZDX");
    return ((data(0, 2) + 2.0 * data(1, 2) + data(2, 2)) -
            (data(0, 0) + 2.0 * data(1, 0) + data(2, 0))) / (8.0 * spacing);
}

/**
  Compute first derivative in Y using 3x3 window.

  \param data the incoming Eigen matrix.
  \param spacing the grid spacing.
  \return the first derivative in Y.
*/
template <typename Derived>
PDAL_DLL double computeDZDY(const Eigen::MatrixBase<Derived>& data,
                            double spacing)
{
    if (data.rows() != 3 || data.cols() != 3)
        throw pdal_error("Must provide 3x3 matrix to computeDZDY");
    if (spacing <= 0.0)
        throw pdal_error("Must provide positive spacing to computeDZDY");
    return ((data(2, 0) + 2.0 * data(2, 1) + data(2, 2)) -
            (data(0, 0) + 2.0 * data(0, 1) + data(0, 2)))  / (8.0 * spacing);
}

inline
PDAL_DLL double computeSlopeRad(double dZdX, double dZdY)
{
    return std::atan(std::sqrt(std::pow(dZdX, 2) + std::pow(dZdY, 2)));
}

/**
  Compute hillshade.

  \param data the incoming Eigen matrix.
  \param spacing the grid spacing.
  \param illumAltitudeDegree the illumination altitude in degrees.
  \param illumAzimuthDegree the illumination azimuth in degrees.
  \return the local slope.
*/
template <typename Derived>
PDAL_DLL double computeHillshade(const Eigen::MatrixBase<Derived>& data,
                                 double spacing, double illumAltitudeDegree,
                                 double illumAzimuthDegree)
{
    if (data.rows() != 3 || data.cols() != 3)
        throw pdal_error("Must provide 3x3 matrix to computeHillshade");
    if (spacing <= 0.0)
        throw pdal_error("Must provide positive spacing to computeHillshade");

    double zenithRad = (90.0 - illumAltitudeDegree) *
                       (3.14159265358979323846 / 180.0);
    double azimuthMath = 360.0 - illumAzimuthDegree + 90.0;
    if (azimuthMath >= 360.0)
    {
        azimuthMath = azimuthMath - 360.0;
    }
    double azimuthRad = azimuthMath * (3.14159265358979323846 / 180.0);

    double dZdX = computeDZDX(data, spacing);
    double dZdY = computeDZDY(data, spacing);
    double slopeRad = computeSlopeRad(dZdX, dZdY);

    double aspectRad(0.0);
    if (dZdX == 0.0)
    {
        if (dZdY > 0.0)
        {
            aspectRad = 3.14159265358979323846 / 2.0;
        }
        else if (dZdY < 0.0)
        {
            aspectRad = (2 * 3.14159265358979323846) -
                        (3.14159265358979323846 / 2.0);
        }
    }
    else
    {
        aspectRad = std::atan2(dZdY, -dZdX);
        if (aspectRad < 0.0)
        {
            aspectRad = 2.0 * 3.14159265358979323846 + aspectRad;
        }
    }

    return std::cos(zenithRad) * std::cos(slopeRad) +
           std::sin(zenithRad) * std::sin(slopeRad) * std::cos(azimuthRad - aspectRad);
}

/**
  Compute aspect using finite difference method.

  \param data the incoming Eigen matrix.
  \param spacing the grid spacing.
  \return the local aspect.
*/
template <typename Derived>
PDAL_DLL double computeAspectFD(const Eigen::MatrixBase<Derived>& data,
                                double spacing)
{
    if (data.rows() != 3 || data.cols() != 3)
        throw pdal_error("Must provide 3x3 matrix to computeAspectFD");
    if (spacing <= 0.0)
        throw pdal_error("Must provide positive spacing to computeAspectFD");
    Derived filled = replaceNaNs(data);
    double zX = centralDiffX(filled, spacing);
    double zY = centralDiffY(filled, spacing);
    double p = (zX * zX) + (zY * zY);
    return 180.0 - std::atan(zY/zX) + 90.0 * (zX / std::fabs(zX));
}

/**
  Compute aspect using D8 method.

  \param data the incoming Eigen matrix.
  \param spacing the grid spacing.
  \return the local aspect.
*/
template <typename Derived>
PDAL_DLL double computeAspectD8(const Eigen::MatrixBase<Derived>& data,
                                double spacing)
{
    if (data.rows() != 3 || data.cols() != 3)
        throw pdal_error("Must provide 3x3 matrix to computeAspectD8");
    if (spacing <= 0.0)
        throw pdal_error("Must provide positive spacing to computeAspectD8");
    Derived submatrix(3, 3);
    submatrix.setConstant(data(1, 1));
    submatrix -= data;
    submatrix /= spacing;
    submatrix(0, 1) /= std::sqrt(2.0);
    submatrix(1, 0) /= std::sqrt(2.0);
    submatrix(1, 2) /= std::sqrt(2.0);
    submatrix(2, 1) /= std::sqrt(2.0);

    // find max and convert to degrees
    double maxval = std::numeric_limits<double>::lowest();
    int j(0);
    for (int i = 0; i < submatrix.size(); ++i)
    {
        if (std::isnan(submatrix(i)))
            continue;
        // skip center value, which will always be 0.0
        if (i == 5)
            continue;
        if (submatrix(i) > maxval)
        {
            maxval = submatrix(i);
            if (i == 1) j = 6;
            if (i == 2) j = 5;
            if (i == 3) j = 4;
            if (i == 4) j = 7;
            if (i == 6) j = 3;
            if (i == 7) j = 0;
            if (i == 8) j = 1;
            if (i == 9) j = 2;
        }
    }

    return std::pow(2.0, j);
}

/**
  Compute slope using D8 method.

  \param data the incoming Eigen matrix.
  \param spacing the grid spacing.
  \return the local slope.
*/
template <typename Derived>
PDAL_DLL double computeSlopeD8(const Eigen::MatrixBase<Derived>& data,
                               double spacing)
{
    if (data.rows() != 3 || data.cols() != 3)
        throw pdal_error("Must provide 3x3 matrix to computeSlopeD8");
    if (spacing <= 0.0)
        throw pdal_error("Must provide positive spacing to computeSlopeD8");
    Derived submatrix(3, 3);
    submatrix.setConstant(data(1, 1));
    submatrix -= data;
    submatrix /= spacing;
    submatrix(0, 0) /= std::sqrt(2.0);
    submatrix(0, 2) /= std::sqrt(2.0);
    submatrix(2, 0) /= std::sqrt(2.0);
    submatrix(2, 2) /= std::sqrt(2.0);

    // Why not just use Eigen's maxCoeff reduction to find the max? Well, as it
    // turns out, if there is a chance that we will have NaN's then maxCoeff
    // has no way to ignore the NaN.
    double maxval = std::numeric_limits<double>::lowest();
    for (int i = 0; i < submatrix.size(); ++i)
    {
        if (std::isnan(submatrix(i)))
            continue;
        if (submatrix(i) > maxval)
            maxval = submatrix(i);
    }
    return 100.0 * maxval;
}

/**
  Compute slope using finite difference method.

  \param data the incoming Eigen matrix.
  \param spacing the grid spacing.
  \return the local slope.
*/
template <typename Derived>
PDAL_DLL double computeSlopeFD(const Eigen::MatrixBase<Derived>& data,
                               double spacing)
{
    if (data.rows() != 3 || data.cols() != 3)
        throw pdal_error("Must provide 3x3 matrix to computeSlopeFD");
    if (spacing <= 0.0)
        throw pdal_error("Must provide positive spacing to computeSlopeFD");
    Derived filled = replaceNaNs(data);
    double zX = centralDiffX(filled, spacing);
    double zY = centralDiffY(filled, spacing);
    double p = (zX * zX) + (zY * zY);
    return 100.0 * std::sqrt(p);
}

/**
  Perform a morphological dilation of the input matrix.

  Performs a morphological dilation of the input matrix using a circular
  structuring element of given radius.

  \param data the input matrix.
  \param radius the radius of the circular structuring element.
  \return the morphological dilation of the input matrix.
*/
template <typename Derived>
PDAL_DLL Derived dilate(const Eigen::MatrixBase<Derived>& A, int radius)
{
    Derived B = Derived::Constant(A.rows(), A.cols(), 0);

    int length = 2 * radius + 1;
    bool match_flag;
    for (int c = 0; c < A.cols(); ++c)
    {
        for (int r = 0; r < A.rows(); ++r)
        {
            match_flag = false;
            for (int k = 0; k < length; ++k)
            {
                if (match_flag)
                    break;
                int cdiff = k-radius;
                int cpos = c+cdiff;
                if (cpos < 0 || cpos >= A.cols())
                    continue;
                for (int l = 0; l < length; ++l)
                {
                    int rdiff = l-radius;
                    int rpos = r+rdiff;
                    if (rpos < 0 || rpos >= A.rows())
                        continue;
                    if ((cdiff*cdiff+rdiff*rdiff) > radius*radius)
                        continue;
                    if (A(rpos, cpos) == 1)
                    {
                        match_flag = true;
                        break;
                    }
                }
            }
            // Assign value according to match flag
            B(r, c) = (match_flag) ? 1 : 0;
        }
    }

    return B;
}

/**
  Perform a morphological erosion of the input matrix.

  Performs a morphological erosion of the input matrix using a circular
  structuring element of given radius.

  \param data the input matrix.
  \param radius the radius of the circular structuring element.
  \return the morphological erosion of the input matrix.
*/
template <typename Derived>
PDAL_DLL Derived erode(const Eigen::MatrixBase<Derived>& A, int radius)
{
    Derived B = Derived::Constant(A.rows(), A.cols(), 1);

    int length = 2 * radius + 1;
    bool mismatch_flag;
    for (int c = 0; c < A.cols(); ++c)
    {
        for (int r = 0; r < A.rows(); ++r)
        {
            if (A(r, c) == 0)
            {
                B(r, c) = 0;
                continue;
            }
            mismatch_flag = false;
            for (int k = 0; k < length; k++)
            {
                if (mismatch_flag)
                    break;
                int cdiff = k-radius;
                int cpos = c+cdiff;
                if (cpos < 0 || cpos >= A.cols())
                    continue;
                for (int l = 0; l < length; l++)
                {
                    int rdiff = l-radius;
                    int rpos = r+rdiff;
                    if (rpos < 0 || rpos >= A.rows())
                        continue;
                    if ((cdiff*cdiff+rdiff*rdiff) > radius*radius)
                        continue;
                    if (A(rpos, cpos) == 0)
                    {
                        B(r, c) = 0;
                        mismatch_flag = true;
                        break;
                    }
                }
            }
            // Assign value according to mismatch flag
            B(r, c) = (mismatch_flag) ? 0 : 1;
        }
    }

    return B;
}

/**
  Thin Plate Spline interpolation.

  \param x the x coordinate of the input data.
  \param y the y coordinate of the input data.
  \param z the z coordinate of the input data.
  \param xx the x coordinate of the points to be interpolated.
  \param yy the y coordinate of the points to be interpolated.
  \return the values of the interpolated data at xx and yy.
*/
PDAL_DLL Eigen::MatrixXd computeSpline(Eigen::MatrixXd x, Eigen::MatrixXd y,
                                       Eigen::MatrixXd z, Eigen::MatrixXd xx,
                                       Eigen::MatrixXd yy);


} // namespace eigen

} // namespace pdal
