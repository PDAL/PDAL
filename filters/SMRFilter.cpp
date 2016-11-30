/******************************************************************************
* Copyright (c) 2016-2017, Bradley J Chambers (brad.chambers@gmail.com)
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

// PDAL implementation of T. J. Pingel, K. C. Clarke, and W. A. McBride, “An
// improved simple morphological filter for the terrain classification of
// airborne LIDAR data,” ISPRS J. Photogramm. Remote Sens., vol. 77, pp. 21–30,
// 2013.

#include "SMRFilter.hpp"

#include <pdal/EigenUtils.hpp>
#include <pdal/KDIndex.hpp>
#include <pdal/pdal_macros.hpp>
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include <Eigen/Dense>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace pdal
{

using namespace Dimension;
using namespace Eigen;
using namespace eigen;

static PluginInfo const s_info =
    PluginInfo("filters.smrf",
               "Simple Morphological Filter (Pingel et al., 2013)",
               "http://pdal.io/stages/filters.smrf.html");

CREATE_STATIC_PLUGIN(1, 0, SMRFilter, Filter, s_info)

std::string SMRFilter::getName() const
{
    return s_info.name;
}

void SMRFilter::addArgs(ProgramArgs& args)
{
    args.add("cell", "Cell size?", m_cell, 1.0);
    args.add("slope", "Percent slope?", m_slope, 0.15);
    args.add("window", "Max window size?", m_window, 18.0);
    args.add("scalar", "Elevation scalar?", m_scalar, 1.25);
    args.add("threshold", "Elevation threshold?", m_threshold, 0.5);
    args.add("cut", "Cut net size?", m_cut, 0.0);
    args.add("dir", "Optional output directory for debugging", m_dir);
}

void SMRFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Id::Classification);
}

void SMRFilter::ready(PointTableRef table)
{
    if (m_dir.empty())
        return;

    if (!FileUtils::directoryExists(m_dir))
        throwError("Output directory '" + m_dir + "' does not exist");
}

PointViewSet SMRFilter::run(PointViewPtr view)
{
    PointViewSet viewSet;
    if (!view->size())
        return viewSet;

    m_srs = view->spatialReference();

    view->calculateBounds(m_bounds);
    m_cols = ((m_bounds.maxx - m_bounds.minx) / m_cell) + 1;
    m_rows = ((m_bounds.maxy - m_bounds.miny) / m_cell) + 1;

    // Create raster of minimum Z values per element.
    MatrixXd ZImin = createZImin(view);

    // Create raster mask of pixels containing low outlier points.
    MatrixXi Low = createLowMask(ZImin);

    // Create raster mask of net cuts. Net cutting is used to when a scene
    // contains large buildings in highly differentiated terrain.
    MatrixXi isNetCell = createNetMask();

    // Apply net cutting to minimum Z raster.
    MatrixXd ZInet = createZInet(ZImin, isNetCell);

    // Create raster mask of pixels containing object points. Note that we use
    // ZInet, the result of net cutting, to identify object pixels.
    MatrixXi Obj = createObjMask(ZInet);

    // Create raster representing the provisional DEM. Note that we use the
    // original ZImin (not ZInet), however the net cut mask will still force
    // interpolation at these pixels.
    MatrixXd ZIpro = createZIpro(view, ZImin, Low, isNetCell, Obj);

    // Classify ground returns by comparing elevation values to the provisional
    // DEM.
    classifyGround(view, ZIpro);

    viewSet.insert(view);

    return viewSet;
}

void SMRFilter::classifyGround(PointViewPtr view, Eigen::MatrixXd const& ZIpro)
{
    // "While many authors use a single value for the elevation threshold, we
    // suggest that a second parameter be used to increase the threshold on
    // steep slopes, transforming the threshold to a slope-dependent value. The
    // total permissible distance is then equal to a fixed elevation threshold
    // plus the scaling value multiplied by the slope of the DEM at each LIDAR
    // point. The rationale behind this approach is that small horizontal and
    // vertical displacements yield larger errors on steep slopes, and as a
    // result the BE/OBJ threshold distance should be more permissive at these
    // points."
    MatrixXd gsurfs(m_rows, m_cols);
    MatrixXd thresh(m_rows, m_cols);
    {
        MatrixXd scaled = ZIpro / m_cell;

        MatrixXd gx = gradX(scaled);
        MatrixXd gy = gradY(scaled);
        gsurfs = (gx.cwiseProduct(gx) + gy.cwiseProduct(gy)).cwiseSqrt();
        MatrixXd gsurfs_fill = knnfill(view, gsurfs);
        gsurfs = gsurfs_fill;
        thresh = (m_threshold + m_scalar * gsurfs.array()).matrix();

        if (!m_dir.empty())
        {
            std::string fname = FileUtils::toAbsolutePath("gx.tif", m_dir);
            writeMatrix(gx, fname, "GTiff", m_cell, m_bounds, m_srs);

            fname = FileUtils::toAbsolutePath("gy.tif", m_dir);
            writeMatrix(gy, fname, "GTiff", m_cell, m_bounds, m_srs);

            fname = FileUtils::toAbsolutePath("gsurfs.tif", m_dir);
            writeMatrix(gsurfs, fname, "GTiff", m_cell, m_bounds, m_srs);

            fname = FileUtils::toAbsolutePath("gsurfs_fill.tif", m_dir);
            writeMatrix(gsurfs_fill, fname, "GTiff", m_cell, m_bounds, m_srs);

            fname = FileUtils::toAbsolutePath("thresh.tif", m_dir);
            writeMatrix(thresh, fname, "GTiff", m_cell, m_bounds, m_srs);
        }
    }

    for (PointId i = 0; i < view->size(); ++i)
    {
        double x = view->getFieldAs<double>(Id::X, i);
        double y = view->getFieldAs<double>(Id::Y, i);
        double z = view->getFieldAs<double>(Id::Z, i);

        size_t c = static_cast<size_t>(std::floor(x - m_bounds.minx) / m_cell);
        size_t r = static_cast<size_t>(std::floor(y - m_bounds.miny) / m_cell);

        // TODO(chambbj): We don't quite do this by the book and yet it seems to
        // work reasonably well:
        // "The calculation requires that both elevation and slope are
        // interpolated from the provisional DEM. There are any number of
        // interpolation techniques that might be used, and even nearest
        // neighbor approaches work quite well, so long as the cell size of the
        // DEM nearly corresponds to the resolution of the LIDAR data. Based on
        // these results, we find that a splined cubic interpolation provides
        // the best results."
        if (std::isnan(ZIpro(r, c)))
            continue;

        if (std::isnan(gsurfs(r, c)))
            continue;

        // "The final step of the algorithm is the identification of
        // ground/object // LIDAR points. This is accomplished by measuring the
        // vertical distance // between each LIDAR point and the provisional
        // DEM, and applying a // threshold calculation."
        if (std::fabs(ZIpro(r, c) - z) > thresh(r, c))
            continue;

        view->setField(Id::Classification, i, 2);
    }
}

Eigen::MatrixXi SMRFilter::createLowMask(Eigen::MatrixXd const& ZImin)
{
    // "[The] minimum surface is checked for low outliers by inverting the point
    // cloud in the z-axis and applying the filter with parameters (slope =
    // 500%, maxWindowSize = 1). The resulting mask is used to flag low outlier
    // cells as OBJ before the inpainting of the provisional DEM."
    MatrixXi Low = progressiveFilter(-ZImin, 5.0, 1.0);

    if (!m_dir.empty())
    {
        std::string fname = FileUtils::toAbsolutePath("zilow.tif", m_dir);
        writeMatrix(Low.cast<double>(), fname, "GTiff", m_cell, m_bounds,
                    m_srs);
    }

    return Low;
}

Eigen::MatrixXi SMRFilter::createNetMask()
{
    // "To accommodate the removal of [very large buildings on highly
    // differentiated terrain], we implemented a feature in the published SMRF
    // algorithm which is helpful in removing such features. We accomplish this
    // by introducing into the initial minimum surface a "net" of minimum values
    // at a spacing equal to the maximum window diameter, where these minimum
    // values are found by applying a morphological open operation with a disk
    // shaped structuring element of radius (2*wkmax)."
    MatrixXi isNetCell = MatrixXi::Zero(m_rows, m_cols);
    if (m_cut > 0.0)
    {
        int v = std::ceil(m_cut / m_cell);

        for (auto c = 0; c < m_cols; c += v)
        {
            for (auto r = 0; r < m_rows; ++r)
            {
                isNetCell(r, c) = 1;
            }
        }
        for (auto c = 0; c < m_cols; ++c)
        {
            for (auto r = 0; r < m_rows; r += v)
            {
                isNetCell(r, c) = 1;
            }
        }
    }

    return isNetCell;
}

Eigen::MatrixXi SMRFilter::createObjMask(Eigen::MatrixXd const& ZImin)
{
    // "The second stage of the ground identification algorithm involves the
    // application of a progressive morphological filter to the minimum surface
    // grid (ZImin)."
    MatrixXi Obj = progressiveFilter(ZImin, m_slope, m_window);

    if (!m_dir.empty())
    {
        std::string fname = FileUtils::toAbsolutePath("ziobj.tif", m_dir);
        writeMatrix(Obj.cast<double>(), fname, "GTiff", m_cell, m_bounds,
                    m_srs);
    }

    return Obj;
}

Eigen::MatrixXd SMRFilter::createZImin(PointViewPtr view)
{
    // "As with many other ground filtering algorithms, the first step is
    // generation of ZImin from the cell size parameter and the extent of the
    // data."
    MatrixXd ZImin = createMinMatrix(*view.get(), m_rows, m_cols, m_cell,
                                     m_bounds);

    // "...some grid points of ZImin will go unfilled. To fill these values, we
    // rely on computationally inexpensive image inpainting techniques. Image
    // inpainting involves the replacement of the empty cells in an image (or
    // matrix) with values calculated from other nearby values."
    MatrixXd ZImin_fill = knnfill(view, ZImin);

    if (!m_dir.empty())
    {
        std::string fname = FileUtils::toAbsolutePath("zimin.tif", m_dir);
        writeMatrix(ZImin, fname, "GTiff", m_cell, m_bounds, m_srs);

        fname = FileUtils::toAbsolutePath("zimin_fill.tif", m_dir);
        writeMatrix(ZImin_fill, fname, "GTiff", m_cell, m_bounds, m_srs);
    }

    return ZImin_fill;
}

Eigen::MatrixXd SMRFilter::createZInet(Eigen::MatrixXd const& ZImin,
                                       Eigen::MatrixXi const& isNetCell)
{
    // "To accommodate the removal of [very large buildings on highly
    // differentiated terrain], we implemented a feature in the published SMRF
    // algorithm which is helpful in removing such features. We accomplish this
    // by introducing into the initial minimum surface a "net" of minimum values
    // at a spacing equal to the maximum window diameter, where these minimum
    // values are found by applying a morphological open operation with a disk
    // shaped structuring element of radius (2*wkmax)."
    MatrixXd ZInet = ZImin;
    if (m_cut > 0.0)
    {
        int v = std::ceil(m_cut / m_cell);
        MatrixXd bigOpen = openDiamond(ZImin, 2 * v);
        for (auto c = 0; c < m_cols; ++c)
        {
            for (auto r = 0; r < m_rows; ++r)
            {
                if (isNetCell(r, c)==1)
                {
                    ZInet(r, c) = bigOpen(r, c);
                }
            }
        }
    }

    if (!m_dir.empty())
    {
        std::string fname = FileUtils::toAbsolutePath("zinet.tif", m_dir);
        writeMatrix(ZInet, fname, "GTiff", m_cell, m_bounds, m_srs);
    }

    return ZInet;
}

Eigen::MatrixXd SMRFilter::createZIpro(PointViewPtr view,
                                       Eigen::MatrixXd const& ZImin,
                                       Eigen::MatrixXi const& Low,
                                       Eigen::MatrixXi const& isNetCell,
                                       Eigen::MatrixXi const& Obj)
{
    // "The end result of the iteration process described above is a binary grid
    // where each cell is classified as being either bare earth (BE) or object
    // (OBJ). The algorithm then applies this mask to the starting minimum
    // surface to eliminate nonground cells."
    MatrixXd ZIpro = ZImin;
    for (int i = 0; i < Obj.size(); ++i)
    {
        if (Obj(i) == 1 || Low(i) == 1 || isNetCell(i) == 1)
            ZIpro(i) = std::numeric_limits<double>::quiet_NaN();
    }

    // "These cells are then inpainted according to the same process described
    // previously, producing a provisional DEM (ZIpro)."
    MatrixXd ZIpro_fill = knnfill(view, ZIpro);

    if (!m_dir.empty())
    {
        std::string fname = FileUtils::toAbsolutePath("zipro.tif", m_dir);
        writeMatrix(ZIpro, fname, "GTiff", m_cell, m_bounds, m_srs);

        fname = FileUtils::toAbsolutePath("zipro_fill.tif", m_dir);
        writeMatrix(ZIpro_fill, fname, "GTiff", m_cell, m_bounds, m_srs);
    }

    return ZIpro_fill;
}

// Fill voids with the average of eight nearest neighbors.
Eigen::MatrixXd SMRFilter::knnfill(PointViewPtr view, Eigen::MatrixXd const& cz)
{
    // Create a temporary PointView that encodes our raster values so that we
    // can construct a 2D KDIndex and perform nearest neighbor searches.
    PointViewPtr temp = view->makeNew();
    PointId i(0);
    for (int c = 0; c < cz.cols(); ++c)
    {
        for (int r = 0; r < cz.rows(); ++r)
        {
            if (std::isnan(cz(r, c)))
                continue;

            temp->setField(Id::X, i, m_bounds.minx + (c + 0.5) * m_cell);
            temp->setField(Id::Y, i, m_bounds.miny + (r + 0.5) * m_cell);
            temp->setField(Id::Z, i, cz(r, c));
            i++;
        }
    }

    KD2Index kdi(*temp);
    kdi.build();

    // Where the raster has voids (i.e., NaN), we search for that cell's eight
    // nearest neighbors, and fill the void with the average value of the
    // neighbors.
    MatrixXd out = cz;
    for (int c = 0; c < cz.cols(); ++c)
    {
        for (int r = 0; r < cz.rows(); ++r)
        {
            if (!std::isnan(out(r, c)))
                continue;

            double x = m_bounds.minx + (c + 0.5) * m_cell;
            double y = m_bounds.miny + (r + 0.5) * m_cell;
            int k = 8;
            std::vector<PointId> neighbors(k);
            std::vector<double> sqr_dists(k);
            kdi.knnSearch(x, y, k, &neighbors, &sqr_dists);

            double M1(0.0);
            size_t j(0);
            for (auto const& n : neighbors)
            {
                j++;
                double delta = temp->getFieldAs<double>(Id::Z, n) - M1;
                M1 += (delta / j);
            }

            out(r, c) = M1;
        }
    }

    return out;
};

// Iteratively open the estimated surface. progressiveFilter can be used to
// identify both low points and object (i.e., non-ground) points, depending on
// the inputs.
MatrixXi SMRFilter::progressiveFilter(MatrixXd const& ZImin, double slope,
                                      double max_window)
{
    // "The maximum window radius is supplied as a distance metric (e.g., 21 m),
    // but is internally converted to a pixel equivalent by dividing it by the
    // cell size and rounding the result toward positive infinity (i.e., taking
    // the ceiling value)."
    int max_radius = std::ceil(max_window / m_cell);
    MatrixXd prevSurface = ZImin;
    MatrixXd prevErosion = ZImin;

    // "...the radius of the element at each step [is] increased by one pixel
    // from a starting value of one pixel to the pixel equivalent of the maximum
    // value."
    MatrixXi Obj = MatrixXi::Zero(m_rows, m_cols);
    for (int radius = 1; radius <= max_radius; ++radius)
    {
        // "On the first iteration, the minimum surface (ZImin) is opened using
        // a disk-shaped structuring element with a radius of one pixel."
        MatrixXd curErosion = erodeDiamond(prevErosion, 1);
        MatrixXd curOpening = dilateDiamond(curErosion, radius);
        prevErosion = curErosion;

        // "An elevation threshold is then calculated, where the value is equal
        // to the supplied slope tolerance parameter multiplied by the product
        // of the window radius and the cell size."
        double threshold = slope * m_cell * radius;

        // "This elevation threshold is applied to the difference of the minimum
        // and the opened surfaces."
        MatrixXd diff = prevSurface - curOpening;

        // "Any grid cell with a difference value exceeding the calculated
        // elevation threshold for the iteration is then flagged as an OBJ
        // cell."
        diff = diff.unaryExpr([threshold](double x)
        {
            return (x > threshold) ? 1 : 0;
        });
        Obj = Obj.cwiseMax(diff.cast<int>());

        // "The algorithm then proceeds to the next window radius (up to the
        // maximum), and proceeds as above with the last opened surface acting
        // as the minimum surface for the next difference calculation."
        prevSurface = curOpening;

        size_t ng(Obj.sum());
        size_t g(Obj.size() - ng);
        double p(100.0 * double(ng) / double(Obj.size()));
        log()->floatPrecision(2);
        log()->get(LogLevel::Debug) << "progressiveFilter: radius = " << radius
                                    << "\t" << g << " ground"
                                    << "\t" << ng << " non-ground"
                                    << "\t(" << p << "%)\n";
    }

    return Obj;
}

} // namespace pdal
