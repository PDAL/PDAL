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

#include "SMRFilter.hpp"

#include <pdal/Eigen.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/PipelineManager.hpp>
#include <buffer/BufferReader.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "gdal_priv.h" // For File I/O
#include "gdal_version.h" // For version info
#include "ogr_spatialref.h"  //For Geographic Information/Transformations

namespace pdal
{
using namespace Eigen;

static PluginInfo const s_info =
    PluginInfo("filters.smrf", "Pingel et al. (2013)",
               "http://pdal.io/stages/filters.smrf.html");

CREATE_STATIC_PLUGIN(1, 0, SMRFilter, Filter, s_info)

struct distElev
{
    double dist;
    double elev;
};

struct by_dist
{
    bool operator()(distElev const& a, distElev const& b)
    {
        return a.dist < b.dist;
    }
};

std::string SMRFilter::getName() const
{
    return s_info.name;
}

void SMRFilter::addArgs(ProgramArgs& args)
{
    args.add("classify", "Apply classification labels?", m_classify, true);
    args.add("extract", "Extract ground returns?", m_extract);
    args.add("cell", "Cell size?", m_cellSize, 1.0);
    args.add("slope", "Slope?", m_percentSlope, 0.15);
    args.add("window", "Max window size?", m_maxWindow, 21.0);
    args.add("threshold", "Threshold?", m_threshold, 0.15);
    args.add("cut", "Cut net size?", m_cutNet, 0.0);
}

void SMRFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Dimension::Id::Classification);
}

int SMRFilter::clamp(int t, int min, int max)
{
    return ((t < min) ? min : ((t > max) ? max : t));
}

int SMRFilter::getColIndex(double x, double cell_size)
{
    return static_cast<int>(floor((x - m_bounds.minx) / cell_size));
}

int SMRFilter::getRowIndex(double y, double cell_size)
{
    return static_cast<int>(floor((m_maxRow - y) / cell_size));
}

MatrixXd SMRFilter::matrixOpen(MatrixXd data, int radius)
{
    MatrixXd data2 = padMatrix(data, radius);

    int nrows = data2.rows();
    int ncols = data2.cols();

    // first min, then max of min
    MatrixXd minZ = MatrixXd::Constant(nrows, ncols,
                                       std::numeric_limits<double>::max());
    MatrixXd maxZ = MatrixXd::Constant(nrows, ncols,
                                       std::numeric_limits<double>::lowest());
    for (int c = 0; c < ncols; ++c)
    {
        for (int r = 0; r < nrows; ++r)
        {
            int cs = clamp(c-radius, 0, ncols-1);
            int ce = clamp(c+radius, 0, ncols-1);
            int rs = clamp(r-radius, 0, nrows-1);
            int re = clamp(r+radius, 0, nrows-1);

            for (int col = cs; col <= ce; ++col)
            {
                for (int row = rs; row <= re; ++row)
                {
                    if ((row-r)*(row-r)+(col-c)*(col-c) > radius*radius)
                        continue;
                    if (data2(row, col) < minZ(r, c))
                        minZ(r, c) = data2(row, col);
                }
            }
        }
    }
    for (int c = 0; c < ncols; ++c)
    {
        for (int r = 0; r < nrows; ++r)
        {
            int cs = clamp(c-radius, 0, ncols-1);
            int ce = clamp(c+radius, 0, ncols-1);
            int rs = clamp(r-radius, 0, nrows-1);
            int re = clamp(r+radius, 0, nrows-1);

            for (int col = cs; col <= ce; ++col)
            {
                for (int row = rs; row <= re; ++row)
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

MatrixXd SMRFilter::inpaintKnn(MatrixXd cx, MatrixXd cy, MatrixXd cz)
{
    MatrixXd out = cz;

    for (auto c = 0; c < m_numCols; ++c)
    {
        for (auto r = 0; r < m_numRows; ++r)
        {
            if (!std::isnan(cz(r, c)))
                continue;

            int radius = 1;
            bool enough = false;

            while (!enough)
            {
                // log()->get(LogLevel::Debug) << r << "\t" << c << "\t" << radius << std::endl;
                int cs = clamp(c-radius, 0, m_numCols-1);
                int ce = clamp(c+radius, 0, m_numCols-1);
                int col_size = ce - cs + 1;
                int rs = clamp(r-radius, 0, m_numRows-1);
                int re = clamp(r+radius, 0, m_numRows-1);
                int row_size = re - rs + 1;

                // MatrixXd Xn = cx.block(rs, cs, row_size, col_size);
                // MatrixXd Yn = cy.block(rs, cs, row_size, col_size);
                MatrixXd Zn = cz.block(rs, cs, row_size, col_size);

                auto notNaN = [](double x)
                {
                    return !std::isnan(x);
                };

                enough = Zn.unaryExpr(notNaN).count() >= 8;
                if (!enough)
                {
                    ++radius;
                    continue;
                }

                // auto zNotNaN = [](double x)
                // {
                //     if (!std::isnan(x))
                //         return x;
                //     else
                //         return 0.0;
                // };
                //
                // // proceed to find 8 nearest neighbors and average the z values
                // // std::cerr << Zn.unaryExpr(zNotNaN).sum() << "\t" << Zn.size() << "\t" << Zn.unaryExpr(zNotNaN).sum() / Zn.size() << std::endl;
                // out(r, c) = Zn.unaryExpr(zNotNaN).sum() / Zn.size();

                std::vector<distElev> de;

                for (auto cc = cs; cc <= ce; ++cc)
                {
                    for (auto rr = rs; rr <= re; ++rr)
                    {
                        if (std::isnan(cz(rr, cc)))
                            continue;

                        // compute distance to !isnan neighbor
                        double dx = cx(rr, cc) - cx(r, c);
                        double dy = cy(rr, cc) - cy(r, c);
                        double sqrdist = dx * dx + dy * dy;
                        de.push_back(distElev{sqrdist, cz(rr, cc)});
                    }
                }
                // sort dists
                std::sort(de.begin(), de.end(), by_dist());

                // average elevatio of lowest eight dists
                double sum = 0.0;
                for (auto i = 0; i < 8; ++i)
                {
                    sum += de[i].elev;
                }
                sum /= 8.0;

                out(r, c) = sum;
            }
        }
    }

    return out;
}

MatrixXd SMRFilter::padMatrix(MatrixXd data, int radius)
{
    MatrixXd data2 = MatrixXd::Zero(data.rows()+2*radius, data.cols()+2*radius);
    data2.block(radius, radius, data.rows(), data.cols()) = data;
    data2.block(radius, 0, data.rows(), radius) =
        data.block(0, 0, data.rows(), radius).rowwise().reverse();
    data2.block(radius, data.cols()+radius, data.rows(), radius) =
        data.block(0, data.cols()-radius, data.rows(), radius).rowwise().reverse();
    data2.block(0, 0, radius, data2.cols()) =
        data2.block(radius, 0, radius, data2.cols()).colwise().reverse();
    data2.block(data.rows()+radius, 0, radius, data2.cols()) =
        data2.block(data2.rows()-radius, 0, radius, data2.cols()).colwise().reverse();

    return data2;
}

std::vector<PointId> SMRFilter::processGround(PointViewPtr view)
{
    log()->get(LogLevel::Info) << "processGround: Running SMRF...\n";

    // The algorithm consists of four conceptually distinct stages. The first is
    // the creation of the minimum surface (ZImin). The second is the processing
    // of the minimum surface, in which grid cells from the raster are
    // identified as either containing bare earth (BE) or objects (OBJ). This
    // second stage represents the heart of the algorithm. The third step is the
    // creation of a DEM from these gridded points. The fourth step is the
    // identification of the original LIDAR points as either BE or OBJ based on
    // their relationship to the interpolated

    std::vector<PointId> groundIdx;
    view->calculateBounds(m_bounds);

    double extent_x = floor(m_bounds.maxx) - ceil(m_bounds.minx);
    double extent_y = floor(m_bounds.maxy) - ceil(m_bounds.miny);

    m_numCols = static_cast<int>(ceil(extent_x/m_cellSize)) + 1;
    m_numRows = static_cast<int>(ceil(extent_y/m_cellSize)) + 1;
    m_maxRow = m_bounds.miny + m_numRows * m_cellSize;

    MatrixXd cx(m_numRows, m_numCols);
    MatrixXd cy(m_numRows, m_numCols);
    for (auto c = 0; c < m_numCols; ++c)
    {
        for (auto r = 0; r < m_numRows; ++r)
        {
            cx(r, c) = m_bounds.minx + (c + 0.5) * m_cellSize;
            cy(r, c) = m_bounds.miny + (r + 0.5) * m_cellSize;
        }
    }

    // STEP 1:

    // As with many other ground filtering algorithms, the first step is
    // generation of ZImin from the cell size parameter and the extent of the
    // data. The two vectors corresponding to [min:cellSize:max] for each
    // coordinate – xi and yi – may be supplied by the user or may be easily and
    // automatically calculated from the data. Without supplied ranges, the SMRF
    // algorithm creates a raster from the ceiling of the minimum to the floor
    // of the maximum values for each of the (x,y) dimensions. If the supplied
    // cell size parameter is not an integer, the same general rule applies to
    // values evenly divisible by the cell size. For example, if cell size is
    // equal to 0.5 m, and the x values range from 52345.6 to 52545.4, the range
    // would be [52346 52545].

    // The minimum surface grid ZImin defined by vectors (xi,yi) is filled with
    // the nearest, lowest elevation from the original point cloud (x,y,z)
    // values, provided that the distance to the nearest point does not exceed
    // the supplied cell size parameter. This provision means that some grid
    // points of ZImin will go unfilled. To fill these values, we rely on
    // computationally inexpensive image inpainting techniques. Image inpainting
    // involves the replacement of the empty cells in an image (or matrix) with
    // values calculated from other nearby values. It is a type of interpolation
    // technique derived from artistic replacement of damaged portions of
    // photographs and paintings, where preservation of texture is an important
    // concern (Bertalmio et al., 2000). When empty values are spread through
    // the image, and the ratio of filled to empty pixels is quite high, most
    // methods of inpainting will produce satisfactory results. In an evaluation
    // of inpainting methods on ground identification from the final terrain
    // model, we found that Laplacian techniques produced error rates nearly
    // three times higher than either an average of the eight nearest neighbors
    // or D’Errico’s spring-metaphor inpainting technique (D’Errico, 2004). The
    // spring-metaphor technique imagines springs connecting each cell with its
    // eight adjacent neighbors, where the inpainted value corresponds to the
    // lowest energy state of the set, and where the entire (sparse) set of
    // linear equations is solved using partial differential equations. Both of
    // these latter techniques were nearly the same with regards to total error,
    // with the spring technique performing slightly better than the k-nearest
    // neighbor (KNN) approach.
    MatrixXd ZImin = createDSM(*view.get(), m_numRows, m_numCols, m_cellSize,
                               m_bounds);
    writeMatrix(ZImin, "zimin.tif", m_cellSize, view);

    // MatrixXd ZImin_painted = inpaintKnn(cx, cy, ZImin);
    // MatrixXd ZImin_painted = TPS(cx, cy, ZImin);
    MatrixXd ZImin_painted = expandingTPS(cx, cy, ZImin);
    writeMatrix(ZImin_painted, "zimin_painted.tif", m_cellSize, view);

    ZImin = ZImin_painted;

    // STEP 2:

    // The second stage of the ground identification algorithm involves the
    // application of a progressive morphological filter to the minimum surface
    // grid (ZImin). At the first iteration, the filter applies an image opening
    // operation to the minimum surface. An opening operation consists of an
    // application of an erosion filter followed by a dilation filter. The
    // erosion acts to snap relative high values to relative lows, where a
    // supplied window radius and shape (or structuring element) defines the
    // search neighborhood. The dilation uses the same window radius and
    // structuring element, acting to outwardly expand relative highs. Fig. 2
    // illustrates an opening operation on a cross section of a transect from
    // Sample 1–1 in the ISPRS LIDAR reference dataset (Sithole and Vosselman,
    // 2003), following Zhang et al. (2003).

    // paper has low point happening later, i guess it doesn't matter too much, this is where he does it in matlab code
    MatrixXi Low = progressiveFilter(-ZImin, m_cellSize, 5.0, 1.0);
    writeMatrix(Low.cast<double>(), "zilow.tif", m_cellSize, view);

    // matlab code has net cutting occurring here
    MatrixXd ZInet = ZImin;
    MatrixXi isNetCell = MatrixXi::Zero(m_numRows, m_numCols);
    if (m_cutNet > 0.0)
    {
        MatrixXd bigOpen = matrixOpen(ZImin, 2*std::ceil(m_cutNet / m_cellSize));
        for (auto c = 0; c < m_numCols; c += std::ceil(m_cutNet/m_cellSize))
        {
            for (auto r = 0; r < m_numRows; ++r)
            {
                isNetCell(r, c) = 1;
            }
        }
        for (auto c = 0; c < m_numCols; ++c)
        {
            for (auto r = 0; r < m_numRows; r += std::ceil(m_cutNet/m_cellSize))
            {
                isNetCell(r, c) = 1;
            }
        }
        for (auto c = 0; c < m_numCols; ++c)
        {
            for (auto r = 0; r < m_numRows; ++r)
            {
                if (isNetCell(r, c)==1)
                    ZInet(r, c) = bigOpen(r, c);
            }
        }
        writeMatrix(ZInet, "zinet.tif", m_cellSize, view);
    }

    // and finally object detection
    MatrixXi Obj = progressiveFilter(ZInet, m_cellSize, m_percentSlope, m_maxWindow);
    writeMatrix(Obj.cast<double>(), "ziobj.tif", m_cellSize, view);

    // STEP 3:

    // The end result of the iteration process described above is a binary grid
    // where each cell is classified as being either bare earth (BE) or object
    // (OBJ). The algorithm then applies this mask to the starting minimum
    // surface to eliminate nonground cells. These cells are then inpainted
    // according to the same process described previously, producing a
    // provisional DEM (ZIpro).

    // we currently aren't checking for net cells or empty cells (haven't i already marked empty cells as NaNs?)
    MatrixXd ZIpro = ZImin;
    for (int i = 0; i < Obj.size(); ++i)
    {
        if (Obj(i) == 1 || Low(i) == 1 || isNetCell(i) == 1)
            ZIpro(i) = std::numeric_limits<double>::quiet_NaN();
    }
    writeMatrix(ZIpro, "zipro.tif", m_cellSize, view);

    // MatrixXd ZIpro_painted = inpaintKnn(cx, cy, ZIpro);
    // MatrixXd ZIpro_painted = TPS(cx, cy, ZIpro);
    MatrixXd ZIpro_painted = expandingTPS(cx, cy, ZIpro);
    writeMatrix(ZIpro_painted, "zipro_painted.tif", m_cellSize, view);

    ZIpro = ZIpro_painted;

    // STEP 4:

    // The final step of the algorithm is the identification of ground/object
    // LIDAR points. This is accomplished by measuring the vertical distance
    // between each LIDAR point and the provisional DEM, and applying a
    // threshold calculation. While many authors use a single value for the
    // elevation threshold, we suggest that a second parameter be used to
    // increase the threshold on steep slopes, transforming the threshold to a
    // slope-dependent value. The total permissible distance is then equal to a
    // fixed elevation threshold plus the scaling value multiplied by the slope
    // of the DEM at each LIDAR point. The rationale behind this approach is
    // that small horizontal and vertical displacements yield larger errors on
    // steep slopes, and as a result the BE/OBJ threshold distance should be
    // more per- missive at these points.

    // The calculation requires that both elevation and slope are interpolated
    // from the provisional DEM. There are any number of interpolation
    // techniques that might be used, and even nearest neighbor approaches work
    // quite well, so long as the cell size of the DEM nearly corresponds to the
    // resolution of the LIDAR data. A comparison of how well these different
    // methods of interpolation perform is given in the next section. Based on
    // these results, we find that a splined cubic interpolation provides the
    // best results.

    // It is common in LIDAR point clouds to have a small number of outliers
    // which may be either above or below the terrain surface. While
    // above-ground outliers (e.g., a random return from a bird in flight) are
    // filtered during the normal algorithm routine, the below-ground outliers
    // (e.g., those caused by a reflection) require a separate approach. Early
    // in the routine and along a separate processing fork, the minimum surface
    // is checked for low outliers by inverting the point cloud in the z-axis
    // and applying the filter with parameters (slope = 500%, maxWindowSize =
    // 1). The resulting mask is used to flag low outlier cells as OBJ before
    // the inpainting of the provisional DEM. This outlier identification
    // methodology is functionally the same as that of Zhang et al. (2003).

    // The provisional DEM (ZIpro), created by removing OBJ cells from the
    // original minimum surface (ZImin) and then inpainting, tends to be less
    // smooth than one might wish, especially when the surfaces are to be used
    // to create visual products like immersive geographic virtual environments.
    // As a result, it is often worthwhile to reinter- polate a final DEM from
    // the identified ground points of the original LIDAR data (ZIfin). Surfaces
    // created from these data tend to be smoother and more visually satisfying
    // than those derived from the provisional DEM.

    // Very large (>40m in length) buildings can sometimes prove troublesome to
    // remove on highly differentiated terrain. To accommodate the removal of
    // such objects, we implemented a feature in the published SMRF algorithm
    // which is helpful in removing such features. We accomplish this by
    // introducing into the initial minimum surface a ‘‘net’’ of minimum values
    // at a spacing equal to the maximum window diameter, where these minimum
    // values are found by applying a morphological open operation with a disk
    // shaped structuring element of radius (2?wkmax). Since only one example in
    // this dataset had features this large (Sample 4–2, a trainyard) we did not
    // include this portion of the algorithm in the formal testing procedure,
    // though we provide a brief analysis of the effect of using this net filter
    // in the next section.

    auto diffX = [&](MatrixXd mat)
    {
        MatrixXd data = padMatrix(mat, 1);
        MatrixXd data2 = data.rightCols(data.cols()-1) - data.leftCols(data.cols()-1);
        return data2.block(1, 1, mat.rows(), mat.cols());
    };

    auto diffY = [&](MatrixXd mat)
    {
        MatrixXd data = padMatrix(mat, 1);
        MatrixXd data2 = data.bottomRows(data.rows()-1) - data.topRows(data.rows()-1);
        return data2.block(1, 1, mat.rows(), mat.cols());
    };

    MatrixXd gx = diffX(ZIpro / m_cellSize);
    writeMatrix(gx, "gx.tif", m_cellSize, view);
    MatrixXd gy = diffY(ZIpro / m_cellSize);
    writeMatrix(gy, "gy.tif", m_cellSize, view);
    MatrixXd gsurfs = (gx.cwiseProduct(gx) + gy.cwiseProduct(gy)).cwiseSqrt();
    writeMatrix(gsurfs, "gsurfs.tif", m_cellSize, view);

    // MatrixXd gsurfs_painted = inpaintKnn(cx, cy, gsurfs);
    // MatrixXd gsurfs_painted = TPS(cx, cy, gsurfs);
    MatrixXd gsurfs_painted = expandingTPS(cx, cy, gsurfs);
    writeMatrix(gsurfs_painted, "gsurfs_painted.tif", m_cellSize, view);

    gsurfs = gsurfs_painted;

    MatrixXd thresh = (m_threshold + 1.2 * gsurfs.array()).matrix();
    writeMatrix(thresh, "thresh.tif", m_cellSize, view);

    for (PointId i = 0; i < view->size(); ++i)
    {
        using namespace Dimension;
        double x = view->getFieldAs<double>(Id::X, i);
        double y = view->getFieldAs<double>(Id::Y, i);
        double z = view->getFieldAs<double>(Id::Z, i);

        int c = clamp(getColIndex(x, m_cellSize), 0, m_numCols-1);
        int r = clamp(getRowIndex(y, m_cellSize), 0, m_numRows-1);

        // author uses spline interpolation to get value from ZIpro and gsurfs

        if (std::isnan(ZIpro(r, c)))
            continue;

        // not sure i should just brush this under the rug...
        if (std::isnan(gsurfs(r, c)))
            continue;

        double ez = ZIpro(r, c);
        // double ez = interp2(r, c, cx, cy, ZIpro);
        // double si = gsurfs(r, c);
        // double si = interp2(r, c, cx, cy, gsurfs);
        // double reqVal = m_threshold + 1.2 * si;

        if (std::abs(ez - z) > thresh(r, c))
            continue;

        // if (std::abs(ZIpro(r, c) - z) > m_threshold)
        //     continue;

        groundIdx.push_back(i);
    }

    return groundIdx;
}

MatrixXi SMRFilter::progressiveFilter(MatrixXd const& ZImin, double cell_size,
                                      double slope, double max_window)
{
    log()->get(LogLevel::Info) << "progressiveFilter: Progressive filtering...\n";

    MatrixXi Obj(m_numRows, m_numCols);
    Obj.setZero();

    // In this case, we selected a disk-shaped structuring element, and the
    // radius of the element at each step was increased by one pixel from a
    // starting value of one pixel to the pixel equivalent of the maximum value
    // (wkmax). The maximum window radius is supplied as a distance metric
    // (e.g., 21 m), but is internally converted to a pixel equivalent by
    // dividing it by the cell size and rounding the result toward positive
    // infinity (i.e., taking the ceiling value). For example, for a supplied
    // maximum window radius of 21 m, and a cell size of 2m per pixel, the
    // result would be a maximum window radius of 11 pixels. While this
    // represents a relatively slow progression in the expansion of the window
    // radius, we believe that the high efficiency associated with the opening
    // operation mitigates the potential for computational waste. The
    // improvements in classification accuracy using slow, linear progressions
    // are documented in the next section.
    int max_radius = ceil(max_window/cell_size);
    MatrixXd ZIlocal = ZImin;
    for (int radius = 1; radius <= max_radius; ++radius)
    {
        // On the first iteration, the minimum surface (ZImin) is opened using a
        // disk-shaped structuring element with a radius of one pixel.
        MatrixXd mo = matrixOpen(ZIlocal, radius);

        // An elevation threshold is then calculated, where the value is equal
        // to the supplied slope tolerance parameter multiplied by the product
        // of the window radius and the cell size. For example, if the user
        // supplied a slope tolerance parameter of 15%, a cell size of 2m per
        // pixel, the elevation threshold would be 0.3m at a window of one pixel
        // (0.15 ? 1 ? 2).
        double threshold = slope * cell_size * radius;

        // This elevation threshold is applied to the difference of the minimum
        // and the opened surfaces.
        MatrixXd diff = ZIlocal - mo;

        // Any grid cell with a difference value exceeding the calculated
        // elevation threshold for the iteration is then flagged as an OBJ cell.
        for (int i = 0; i < diff.size(); ++i)
        {
            if (diff(i) > threshold)
                Obj(i) = 1;
        }
        // writeMatrix(Obj, "obj.tif", m_cellSize, view);

        // The algorithm then proceeds to the next window radius (up to the
        // maximum), and proceeds as above with the last opened surface acting
        // as the ‘‘minimum surface’’ for the next difference calculation.
        ZIlocal = mo;

        log()->get(LogLevel::Info) << "progressiveFilter: Radius = " << radius
                                   << ", " << Obj.sum() << " object pixels\n";
    }

    return Obj;
}

PointViewSet SMRFilter::run(PointViewPtr view)
{
    log()->get(LogLevel::Info) << "run: Process SMRFilter...\n";

    std::vector<PointId> idx = processGround(view);

    PointViewSet viewSet;

    if (!idx.empty() && (m_classify || m_extract))
    {

        if (m_classify)
        {
            log()->get(LogLevel::Info) << "run: Labeled " << idx.size() << " ground returns!\n";

            // set the classification label of ground returns as 2
            // (corresponding to ASPRS LAS specification)
            for (const auto& i : idx)
            {
                view->setField(Dimension::Id::Classification, i, 2);
            }

            viewSet.insert(view);
        }

        if (m_extract)
        {
            log()->get(LogLevel::Info) << "run: Extracted " << idx.size() << " ground returns!\n";

            // create new PointView containing only ground returns
            PointViewPtr output = view->makeNew();
            for (const auto& i : idx)
            {
                output->appendPoint(*view, i);
            }

            viewSet.erase(view);
            viewSet.insert(output);
        }
    }
    else
    {
        if (idx.empty())
            log()->get(LogLevel::Info) << "run: Filtered cloud has no ground returns!\n";

        if (!(m_classify || m_extract))
            log()->get(LogLevel::Info) << "run: Must choose --classify or --extract\n";

        // return the view buffer unchanged
        viewSet.insert(view);
    }

    return viewSet;
}

MatrixXd SMRFilter::TPS(MatrixXd cx, MatrixXd cy, MatrixXd cz)
{
    log()->get(LogLevel::Info) << "TPS: Reticulating splines...\n";

    MatrixXd S = cz;

    int num_nan_detect(0);
    int num_nan_replace(0);

    for (auto outer_col = 0; outer_col < m_numCols; ++outer_col)
    {
        for (auto outer_row = 0; outer_row < m_numRows; ++outer_row)
        {
            if (!std::isnan(S(outer_row, outer_col)))
                continue;

            num_nan_detect++;

            // Further optimizations are achieved by estimating only the
            // interpolated surface within a local neighbourhood (e.g. a 7 x 7
            // neighbourhood is used in our case) of the cell being filtered.
            int radius = 3;

            int cs = clamp(outer_col-radius, 0, m_numCols-1);
            int ce = clamp(outer_col+radius, 0, m_numCols-1);
            int col_size = ce - cs + 1;
            int rs = clamp(outer_row-radius, 0, m_numRows-1);
            int re = clamp(outer_row+radius, 0, m_numRows-1);
            int row_size = re - rs + 1;

            MatrixXd Xn = cx.block(rs, cs, row_size, col_size);
            MatrixXd Yn = cy.block(rs, cs, row_size, col_size);
            MatrixXd Hn = cz.block(rs, cs, row_size, col_size);

            int nsize = Hn.size();
            VectorXd T = VectorXd::Zero(nsize);
            MatrixXd P = MatrixXd::Zero(nsize, 3);
            MatrixXd K = MatrixXd::Zero(nsize, nsize);

            int numK(0);
            for (auto id = 0; id < Hn.size(); ++id)
            {
                double xj = Xn(id);
                double yj = Yn(id);
                double zj = Hn(id);
                if (std::isnan(zj))
                    continue;
                numK++;
                T(id) = zj;
                P.row(id) << 1, xj, yj;
                for (auto id2 = 0; id2 < Hn.size(); ++id2)
                {
                    if (id == id2)
                        continue;
                    double xk = Xn(id2);
                    double yk = Yn(id2);
                    double rsqr = (xj - xk) * (xj - xk) + (yj - yk) * (yj - yk);
                    if (rsqr == 0.0)
                        continue;
                    K(id, id2) = rsqr * std::log10(std::sqrt(rsqr));
                }
            }

            if (numK < 20)
                continue;

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
            double xi2 = cx(outer_row, outer_col);
            double yi2 = cy(outer_row, outer_col);
            for (auto j = 0; j < nsize; ++j)
            {
                double xj = Xn(j);
                double yj = Yn(j);
                double rsqr = (xj - xi2) * (xj - xi2) + (yj - yi2) * (yj - yi2);
                if (rsqr == 0.0)
                    continue;
                sum += w(j) * rsqr * std::log10(std::sqrt(rsqr));
            }

            S(outer_row, outer_col) = a(0) + a(1)*xi2 + a(2)*yi2 + sum;

            if (!std::isnan(S(outer_row, outer_col)))
                num_nan_replace++;

            // std::cerr << std::fixed;
            // std::cerr << std::setprecision(3)
            //           << std::left
            //           << "S(" << outer_row << "," << outer_col << "): "
            //           << std::setw(10)
            //           << S(outer_row, outer_col)
            //           // << std::setw(3)
            //           // << "\tz: "
            //           // << std::setw(10)
            //           // << zi
            //           // << std::setw(7)
            //           // << "\tzdiff: "
            //           // << std::setw(5)
            //           // << zi - S(outer_row, outer_col)
            //           // << std::setw(7)
            //           // << "\txdiff: "
            //           // << std::setw(5)
            //           // << xi2 - xi
            //           // << std::setw(7)
            //           // << "\tydiff: "
            //           // << std::setw(5)
            //           // << yi2 - yi
            //           << std::setw(7)
            //           << "\t# pts: "
            //           << std::setw(3)
            //           << nsize
            //           << std::setw(5)
            //           << "\tsum: "
            //           << std::setw(10)
            //           << sum
            //           << std::setw(9)
            //           << "\tw.sum(): "
            //           << std::setw(5)
            //           << w.sum()
            //           << std::setw(6)
            //           << "\txsum: "
            //           << std::setw(5)
            //           << w.dot(P.col(1))
            //           << std::setw(6)
            //           << "\tysum: "
            //           << std::setw(5)
            //           << w.dot(P.col(2))
            //           << std::setw(3)
            //           << "\ta: "
            //           << std::setw(8)
            //           << a.transpose()
            //           << std::endl;
        }
    }

    double frac = static_cast<double>(num_nan_replace);
    frac /= static_cast<double>(num_nan_detect);
    log()->get(LogLevel::Info) << "TPS: Filled " << num_nan_replace << " of "
                               << num_nan_detect << " holes ("
                               << frac * 100.0 << "%)\n";

    return S;
}

MatrixXd SMRFilter::expandingTPS(MatrixXd cx, MatrixXd cy, MatrixXd cz)
{
    log()->get(LogLevel::Info) << "TPS: Reticulating splines...\n";

    MatrixXd S = cz;

    int num_nan_detect(0);
    int num_nan_replace(0);

    for (auto outer_col = 0; outer_col < m_numCols; ++outer_col)
    {
        for (auto outer_row = 0; outer_row < m_numRows; ++outer_row)
        {
            if (!std::isnan(S(outer_row, outer_col)))
                continue;

            num_nan_detect++;

            // Further optimizations are achieved by estimating only the
            // interpolated surface within a local neighbourhood (e.g. a 7 x 7
            // neighbourhood is used in our case) of the cell being filtered.
            int radius = 3;
            bool solution = false;

            while (!solution)
            {
                // std::cerr << radius;
                int cs = clamp(outer_col-radius, 0, m_numCols-1);
                int ce = clamp(outer_col+radius, 0, m_numCols-1);
                int col_size = ce - cs + 1;
                int rs = clamp(outer_row-radius, 0, m_numRows-1);
                int re = clamp(outer_row+radius, 0, m_numRows-1);
                int row_size = re - rs + 1;

                MatrixXd Xn = cx.block(rs, cs, row_size, col_size);
                MatrixXd Yn = cy.block(rs, cs, row_size, col_size);
                MatrixXd Hn = cz.block(rs, cs, row_size, col_size);

                int nsize = Hn.size();
                VectorXd T = VectorXd::Zero(nsize);
                MatrixXd P = MatrixXd::Zero(nsize, 3);
                MatrixXd K = MatrixXd::Zero(nsize, nsize);

                int numK(0);
                for (auto id = 0; id < Hn.size(); ++id)
                {
                    double xj = Xn(id);
                    double yj = Yn(id);
                    double zj = Hn(id);
                    if (std::isnan(zj))
                        continue;
                    numK++;
                    T(id) = zj;
                    P.row(id) << 1, xj, yj;
                    for (auto id2 = 0; id2 < Hn.size(); ++id2)
                    {
                        if (id == id2)
                            continue;
                        double xk = Xn(id2);
                        double yk = Yn(id2);
                        double rsqr = (xj - xk) * (xj - xk) + (yj - yk) * (yj - yk);
                        if (rsqr == 0.0)
                            continue;
                        K(id, id2) = rsqr * std::log10(std::sqrt(rsqr));
                    }
                }

                // if (numK < 20)
                //     continue;

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
                double xi2 = cx(outer_row, outer_col);
                double yi2 = cy(outer_row, outer_col);
                for (auto j = 0; j < nsize; ++j)
                {
                    double xj = Xn(j);
                    double yj = Yn(j);
                    double rsqr = (xj - xi2) * (xj - xi2) + (yj - yi2) * (yj - yi2);
                    if (rsqr == 0.0)
                        continue;
                    sum += w(j) * rsqr * std::log10(std::sqrt(rsqr));
                }

                double val = a(0) + a(1)*xi2 + a(2)*yi2 + sum;
                solution = !std::isnan(val);

                if (!solution)
                {
                    std::cerr << "..." << radius << std::endl;;
                    ++radius;
                    continue;
                }

                S(outer_row, outer_col) = val;
                num_nan_replace++;

                // std::cerr << std::endl;

                // std::cerr << std::fixed;
                // std::cerr << std::setprecision(3)
                //           << std::left
                //           << "S(" << outer_row << "," << outer_col << "): "
                //           << std::setw(10)
                //           << S(outer_row, outer_col)
                //           // << std::setw(3)
                //           // << "\tz: "
                //           // << std::setw(10)
                //           // << zi
                //           // << std::setw(7)
                //           // << "\tzdiff: "
                //           // << std::setw(5)
                //           // << zi - S(outer_row, outer_col)
                //           // << std::setw(7)
                //           // << "\txdiff: "
                //           // << std::setw(5)
                //           // << xi2 - xi
                //           // << std::setw(7)
                //           // << "\tydiff: "
                //           // << std::setw(5)
                //           // << yi2 - yi
                //           << std::setw(7)
                //           << "\t# pts: "
                //           << std::setw(3)
                //           << nsize
                //           << std::setw(5)
                //           << "\tsum: "
                //           << std::setw(10)
                //           << sum
                //           << std::setw(9)
                //           << "\tw.sum(): "
                //           << std::setw(5)
                //           << w.sum()
                //           << std::setw(6)
                //           << "\txsum: "
                //           << std::setw(5)
                //           << w.dot(P.col(1))
                //           << std::setw(6)
                //           << "\tysum: "
                //           << std::setw(5)
                //           << w.dot(P.col(2))
                //           << std::setw(3)
                //           << "\ta: "
                //           << std::setw(8)
                //           << a.transpose()
                //           << std::endl;
            }
        }
    }

    double frac = static_cast<double>(num_nan_replace);
    frac /= static_cast<double>(num_nan_detect);
    log()->get(LogLevel::Info) << "TPS: Filled " << num_nan_replace << " of "
                               << num_nan_detect << " holes ("
                               << frac * 100.0 << "%)\n";

    return S;
}

void SMRFilter::writeMatrix(MatrixXd data, std::string filename,
                            double cell_size, PointViewPtr view)
{
    int cols = data.cols();
    int rows = data.rows();

    GDALAllRegister();

    GDALDataset *mpDstDS = NULL;

    char **papszMetadata;

    // parse the format driver, hardcoded for the time being
    std::string tFormat("GTIFF");
    const char *pszFormat = tFormat.c_str();
    GDALDriver* tpDriver = GetGDALDriverManager()->GetDriverByName(pszFormat);

    // try to create a file of the requested format
    if (tpDriver != NULL)
    {
        papszMetadata = tpDriver->GetMetadata();
        if (CSLFetchBoolean(papszMetadata, GDAL_DCAP_CREATE, FALSE))
        {
            char **papszOptions = NULL;

            mpDstDS = tpDriver->Create(filename.c_str(), cols, rows, 1,
                                       GDT_Float32, papszOptions);

            // set the geo transformation
            double adfGeoTransform[6];
            adfGeoTransform[0] = m_bounds.minx; // - 0.5*m_GRID_DIST_X;
            adfGeoTransform[1] = cell_size;
            adfGeoTransform[2] = 0.0;
            adfGeoTransform[3] = m_bounds.maxy; // + 0.5*m_GRID_DIST_Y;
            adfGeoTransform[4] = 0.0;
            adfGeoTransform[5] = -1 * cell_size;
            mpDstDS->SetGeoTransform(adfGeoTransform);

            // set the projection
            mpDstDS->SetProjection(view->spatialReference().getWKT().c_str());
        }
    }

    // if we have a valid file
    if (mpDstDS)
    {
        // loop over the raster and determine max slope at each location
        int cs = 1, ce = cols - 1;
        int rs = 1, re = rows - 1;
        float *poRasterData = new float[cols*rows];
        for (auto i=0; i<cols*rows; i++)
        {
            poRasterData[i] = std::numeric_limits<float>::min();
        }

        // #pragma omp parallel for
        for (auto c = cs; c < ce; ++c)
        {
            for (auto r = rs; r < re; ++r)
            {
                if (data(r, c) == 0.0 || std::isnan(data(r, c)) || data(r, c) == std::numeric_limits<double>::max())
                    continue;
                poRasterData[(r * cols) + c] =
                    data(r, c);
            }
        }

        // write the data
        if (poRasterData)
        {
            GDALRasterBand *tBand = mpDstDS->GetRasterBand(1);

            tBand->SetNoDataValue(std::numeric_limits<float>::min());

            if (cols > 0 && rows > 0)
#if GDAL_VERSION_MAJOR <= 1
                tBand->RasterIO(GF_Write, 0, 0, cols, rows,
                                poRasterData, cols, rows,
                                GDT_Float32, 0, 0);
#else

                int ret = tBand->RasterIO(GF_Write, 0, 0, cols, rows,
                                          poRasterData, cols, rows,
                                          GDT_Float32, 0, 0, 0);
#endif
        }

        GDALClose((GDALDatasetH) mpDstDS);

        delete [] poRasterData;
    }
}

} // namespace pdal
