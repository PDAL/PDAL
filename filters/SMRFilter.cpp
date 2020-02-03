/******************************************************************************
 * Copyright (c) 2016-2017, 2020 Bradley J Chambers (brad.chambers@gmail.com)
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
#include <pdal/util/FileUtils.hpp>
#include <pdal/util/ProgramArgs.hpp>

#include "private/DimRange.hpp"
#include "private/Segmentation.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

namespace pdal
{

using namespace Dimension;
using namespace Eigen;

static StaticPluginInfo const s_info{
    "filters.smrf", "Simple Morphological Filter (Pingel et al., 2013)",
    "http://pdal.io/stages/filters.smrf.html"};

// Without the cast, MSVC complains, which is ridiculous when the output
// is, by definition, an int.
namespace
{
template <typename T> T ceil(double d)
{
    return static_cast<T>(std::ceil(d));
}
} // namespace

CREATE_STATIC_STAGE(SMRFilter, s_info)

struct SMRArgs
{
    double m_cell;
    double m_slope;
    double m_window;
    double m_scalar;
    double m_threshold;
    double m_cut;
    std::string m_dir;
    std::vector<DimRange> m_ignored;
    StringList m_returns;
};

SMRFilter::SMRFilter() : m_args(new SMRArgs) {}

SMRFilter::~SMRFilter() {}

std::string SMRFilter::getName() const
{
    return s_info.name;
}

void SMRFilter::addArgs(ProgramArgs& args)
{
    args.add("cell", "Cell size?", m_args->m_cell, 1.0);
    args.add("slope", "Percent slope?", m_args->m_slope, 0.15);
    args.add("window", "Max window size?", m_args->m_window, 18.0);
    args.add("scalar", "Elevation scalar?", m_args->m_scalar, 1.25);
    args.add("threshold", "Elevation threshold?", m_args->m_threshold, 0.5);
    args.add("cut", "Cut net size?", m_args->m_cut, 0.0);
    args.add("dir", "Optional output directory for debugging", m_args->m_dir);
    args.add("ignore", "Ignore values", m_args->m_ignored);
    args.add("returns", "Include last returns?", m_args->m_returns,
             {"last", "only"});
}

void SMRFilter::addDimensions(PointLayoutPtr layout)
{
    layout->registerDim(Id::Classification);
}

void SMRFilter::prepared(PointTableRef table)
{
    const PointLayoutPtr layout(table.layout());

    for (auto& r : m_args->m_ignored)
    {
        r.m_id = layout->findDim(r.m_name);
        if (r.m_id == Id::Unknown)
            throwError("Invalid dimension name in 'ignored' option: '" +
                       r.m_name + "'.");
    }
    if (m_args->m_returns.size())
    {
        for (auto& r : m_args->m_returns)
        {
            Utils::trim(r);
            if ((r != "first") && (r != "intermediate") && (r != "last") &&
                (r != "only"))
            {
                throwError("Unrecognized 'returns' value: '" + r + "'.");
            }
        }

        if (!layout->hasDim(Id::ReturnNumber) ||
            !layout->hasDim(Id::NumberOfReturns))
        {
            log()->get(LogLevel::Warning) << "Could not find ReturnNumber and "
                                             "NumberOfReturns. Skipping "
                                             "segmentation of last returns and "
                                             "proceeding with all returns.\n";
            m_args->m_returns.clear();
        }
    }
}

void SMRFilter::ready(PointTableRef table)
{
    if (m_args->m_dir.empty())
        return;

    if (!FileUtils::directoryExists(m_args->m_dir))
        throwError("Output directory '" + m_args->m_dir + "' does not exist");
}

PointViewSet SMRFilter::run(PointViewPtr view)
{
    PointViewSet viewSet;
    if (!view->size())
        return viewSet;

    // Segment input view into ignored/kept views.
    PointViewPtr ignoredView = view->makeNew();
    PointViewPtr keptView = view->makeNew();
    if (m_args->m_ignored.empty())
        keptView->append(*view);
    else
        Segmentation::ignoreDimRanges(m_args->m_ignored, view, keptView,
                                      ignoredView);

    // Check for 0's in ReturnNumber and NumberOfReturns
    bool nrOneZero(false);
    bool rnOneZero(false);
    bool nrAllZero(true);
    bool rnAllZero(true);
    for (PointId i = 0; i < keptView->size(); ++i)
    {
        uint8_t nr = keptView->getFieldAs<uint8_t>(Id::NumberOfReturns, i);
        uint8_t rn = keptView->getFieldAs<uint8_t>(Id::ReturnNumber, i);
        if ((nr == 0) && !nrOneZero)
            nrOneZero = true;
        if ((rn == 0) && !rnOneZero)
            rnOneZero = true;
        if (nr != 0)
            nrAllZero = false;
        if (rn != 0)
            rnAllZero = false;
    }

    if ((nrOneZero || rnOneZero) && !(nrAllZero && rnAllZero))
        throwError("Some NumberOfReturns or ReternNumber values were 0, but "
                   "not all. Check that all values in the input file are >= "
                   "1.");

    // Segment kept view into two views
    PointViewPtr modifiedReturnsView = keptView->makeNew();
    PointViewPtr staticReturnsView = keptView->makeNew();
    if (nrAllZero && rnAllZero)
    {
        log()->get(LogLevel::Warning)
            << "Both NumberOfReturns and ReturnNumber are filled with 0's. "
               "Proceeding without any further return filtering.\n";
        modifiedReturnsView->append(*keptView);
    }
    else
    {
        Segmentation::segmentReturns(keptView, modifiedReturnsView,
                                     staticReturnsView, m_args->m_returns);
    }

    if (!modifiedReturnsView->size())
    {
        throwError("No returns to process.");
    }

    // Classify remaining points with value of 1. SMRF processing will mark
    // ground returns as 2.
    for (PointId i = 0; i < modifiedReturnsView->size(); ++i)
        modifiedReturnsView->setField(Id::Classification, i,
                                      ClassLabel::Unclassified);

    m_srs = modifiedReturnsView->spatialReference();

    calculateBounds(*modifiedReturnsView, m_bounds);
    m_cols = static_cast<int>(
        ((m_bounds.maxx - m_bounds.minx) / m_args->m_cell) + 1);
    m_rows = static_cast<int>(
        ((m_bounds.maxy - m_bounds.miny) / m_args->m_cell) + 1);

    // Create raster of minimum Z values per element.
    std::vector<double> ZImin = createZImin(modifiedReturnsView);

    // Create raster mask of pixels containing low outlier points.
    std::vector<int> Low = createLowMask(ZImin);

    // Create raster mask of net cuts. Net cutting is used to when a scene
    // contains large buildings in highly differentiated terrain.
    std::vector<int> isNetCell = createNetMask();

    // Apply net cutting to minimum Z raster.
    std::vector<double> ZInet = createZInet(ZImin, isNetCell);

    // Create raster mask of pixels containing object points. Note that we use
    // ZInet, the result of net cutting, to identify object pixels.
    std::vector<int> Obj = createObjMask(ZInet);

    // Create raster representing the provisional DEM. Note that we use the
    // original ZImin (not ZInet), however the net cut mask will still force
    // interpolation at these pixels.
    std::vector<double> ZIpro =
        createZIpro(modifiedReturnsView, ZImin, Low, isNetCell, Obj);

    // Classify ground returns by comparing elevation values to the provisional
    // DEM.
    classifyGround(modifiedReturnsView, ZIpro);

    PointViewPtr outView = view->makeNew();
    // ignoredView and staticReturnsView are appended to the output untouched.
    outView->append(*ignoredView);
    outView->append(*staticReturnsView);
    // modifiedReturnsView is appended to the output, the only PointView whose
    // classifications may have been altered.
    outView->append(*modifiedReturnsView);
    viewSet.insert(outView);

    return viewSet;
}

void SMRFilter::classifyGround(PointViewPtr view, std::vector<double>& ZIpro)
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
        MatrixXd ZIproM = Map<MatrixXd>(ZIpro.data(), m_rows, m_cols);
        MatrixXd scaled = ZIproM / m_args->m_cell;

        MatrixXd gx = gradX(scaled);
        MatrixXd gy = gradY(scaled);
        gsurfs = (gx.cwiseProduct(gx) + gy.cwiseProduct(gy)).cwiseSqrt();
        std::vector<double> gsurfsV(gsurfs.data(),
                                    gsurfs.data() + gsurfs.size());
        std::vector<double> gsurfs_fillV = knnfill(view, gsurfsV);
        gsurfs = Map<MatrixXd>(gsurfs_fillV.data(), m_rows, m_cols);
        thresh =
            (m_args->m_threshold + m_args->m_scalar * gsurfs.array()).matrix();

        if (!m_args->m_dir.empty())
        {
            std::string fname =
                FileUtils::toAbsolutePath("gx.tif", m_args->m_dir);
            writeMatrix(gx, fname, "GTiff", m_args->m_cell, m_bounds, m_srs);

            fname = FileUtils::toAbsolutePath("gy.tif", m_args->m_dir);
            writeMatrix(gy, fname, "GTiff", m_args->m_cell, m_bounds, m_srs);

            fname = FileUtils::toAbsolutePath("gsurfs.tif", m_args->m_dir);
            writeMatrix(gsurfs, fname, "GTiff", m_args->m_cell, m_bounds,
                        m_srs);

            fname = FileUtils::toAbsolutePath("gsurfs_fill.tif", m_args->m_dir);
            MatrixXd gsurfs_fill =
                Map<MatrixXd>(gsurfs_fillV.data(), m_rows, m_cols);
            writeMatrix(gsurfs_fill, fname, "GTiff", m_args->m_cell, m_bounds,
                        m_srs);

            fname = FileUtils::toAbsolutePath("thresh.tif", m_args->m_dir);
            writeMatrix(thresh, fname, "GTiff", m_args->m_cell, m_bounds,
                        m_srs);
        }
    }

    for (PointId i = 0; i < view->size(); ++i)
    {
        double x = view->getFieldAs<double>(Id::X, i);
        double y = view->getFieldAs<double>(Id::Y, i);
        double z = view->getFieldAs<double>(Id::Z, i);

        size_t c =
            static_cast<size_t>(std::floor(x - m_bounds.minx) / m_args->m_cell);
        size_t r =
            static_cast<size_t>(std::floor(y - m_bounds.miny) / m_args->m_cell);

        // TODO(chambbj): We don't quite do this by the book and yet it seems to
        // work reasonably well:
        // "The calculation requires that both elevation and slope are
        // interpolated from the provisional DEM. There are any number of
        // interpolation techniques that might be used, and even nearest
        // neighbor approaches work quite well, so long as the cell size of the
        // DEM nearly corresponds to the resolution of the LIDAR data. Based on
        // these results, we find that a splined cubic interpolation provides
        // the best results."
        if (std::isnan(ZIpro[c * m_rows + r]))
            continue;

        if (std::isnan(gsurfs(r, c)))
            continue;

        // "The final step of the algorithm is the identification of
        // ground/object LIDAR points. This is accomplished by measuring the
        // vertical distance between each LIDAR point and the provisional
        // DEM, and applying a threshold calculation."
        if (std::fabs(ZIpro[c * m_rows + r] - z) > thresh(r, c))
            view->setField(Id::Classification, i, ClassLabel::Unclassified);
        else
            view->setField(Id::Classification, i, ClassLabel::Ground);
    }
}

std::vector<int> SMRFilter::createLowMask(std::vector<double> const& ZImin)
{
    // "[The] minimum surface is checked for low outliers by inverting the point
    // cloud in the z-axis and applying the filter with parameters (slope =
    // 500%, maxWindowSize = 1). The resulting mask is used to flag low outlier
    // cells as OBJ before the inpainting of the provisional DEM."

    // Need to add a step to negate ZImin
    std::vector<double> negZImin;
    std::transform(ZImin.begin(), ZImin.end(), std::back_inserter(negZImin),
                   [](double v) { return -v; });
    std::vector<int> LowV = progressiveFilter(negZImin, 5.0, 1.0);

    if (!m_args->m_dir.empty())
    {
        std::string fname =
            FileUtils::toAbsolutePath("zilow.tif", m_args->m_dir);
        MatrixXi Low = Map<MatrixXi>(LowV.data(), m_rows, m_cols);
        writeMatrix(Low.cast<double>(), fname, "GTiff", m_args->m_cell,
                    m_bounds, m_srs);
    }

    return LowV;
}

std::vector<int> SMRFilter::createNetMask()
{
    // "To accommodate the removal of [very large buildings on highly
    // differentiated terrain], we implemented a feature in the published SMRF
    // algorithm which is helpful in removing such features. We accomplish this
    // by introducing into the initial minimum surface a "net" of minimum values
    // at a spacing equal to the maximum window diameter, where these minimum
    // values are found by applying a morphological open operation with a disk
    // shaped structuring element of radius (2*wkmax)."
    std::vector<int> isNetCell(m_rows * m_cols, 0);
    if (m_args->m_cut > 0.0)
    {
        int v = ceil<int>(m_args->m_cut / m_args->m_cell);

        for (auto c = 0; c < m_cols; c += v)
        {
            for (auto r = 0; r < m_rows; ++r)
            {
                isNetCell[c * m_rows + r] = 1;
            }
        }
        for (auto c = 0; c < m_cols; ++c)
        {
            for (auto r = 0; r < m_rows; r += v)
            {
                isNetCell[c * m_rows + r] = 1;
            }
        }
    }

    return isNetCell;
}

std::vector<int> SMRFilter::createObjMask(std::vector<double> const& ZImin)
{
    // "The second stage of the ground identification algorithm involves the
    // application of a progressive morphological filter to the minimum surface
    // grid (ZImin)."
    std::vector<int> ObjV =
        progressiveFilter(ZImin, m_args->m_slope, m_args->m_window);

    if (!m_args->m_dir.empty())
    {
        std::string fname =
            FileUtils::toAbsolutePath("ziobj.tif", m_args->m_dir);
        MatrixXi Obj = Map<MatrixXi>(ObjV.data(), m_rows, m_cols);
        writeMatrix(Obj.cast<double>(), fname, "GTiff", m_args->m_cell,
                    m_bounds, m_srs);
    }

    return ObjV;
}

std::vector<double> SMRFilter::createZImin(PointViewPtr view)
{
    // "As with many other ground filtering algorithms, the first step is
    // generation of ZImin from the cell size parameter and the extent of the
    // data."
    std::vector<double> ZIminV(m_rows * m_cols,
                               std::numeric_limits<double>::quiet_NaN());

    for (PointId i = 0; i < view->size(); ++i)
    {
        double x = view->getFieldAs<double>(Id::X, i);
        double y = view->getFieldAs<double>(Id::Y, i);
        double z = view->getFieldAs<double>(Id::Z, i);

        int c = static_cast<int>(floor(x - m_bounds.minx) / m_args->m_cell);
        int r = static_cast<int>(floor(y - m_bounds.miny) / m_args->m_cell);

        if (z < ZIminV[c * m_rows + r] || std::isnan(ZIminV[c * m_rows + r]))
            ZIminV[c * m_rows + r] = z;
    }

    // "...some grid points of ZImin will go unfilled. To fill these values, we
    // rely on computationally inexpensive image inpainting techniques. Image
    // inpainting involves the replacement of the empty cells in an image (or
    // matrix) with values calculated from other nearby values."
    std::vector<double> ZImin_fillV = knnfill(view, ZIminV);

    if (!m_args->m_dir.empty())
    {
        std::string fname =
            FileUtils::toAbsolutePath("zimin.tif", m_args->m_dir);
        MatrixXd ZImin = Map<MatrixXd>(ZIminV.data(), m_rows, m_cols);
        writeMatrix(ZImin, fname, "GTiff", m_args->m_cell, m_bounds, m_srs);

        fname = FileUtils::toAbsolutePath("zimin_fill.tif", m_args->m_dir);
        MatrixXd ZImin_fill = Map<MatrixXd>(ZImin_fillV.data(), m_rows, m_cols);
        writeMatrix(ZImin_fill, fname, "GTiff", m_args->m_cell, m_bounds,
                    m_srs);
    }

    return ZImin_fillV;
}

std::vector<double> SMRFilter::createZInet(std::vector<double> const& ZImin,
                                           std::vector<int> const& isNetCell)
{
    // "To accommodate the removal of [very large buildings on highly
    // differentiated terrain], we implemented a feature in the published SMRF
    // algorithm which is helpful in removing such features. We accomplish this
    // by introducing into the initial minimum surface a "net" of minimum values
    // at a spacing equal to the maximum window diameter, where these minimum
    // values are found by applying a morphological open operation with a disk
    // shaped structuring element of radius (2*wkmax)."
    std::vector<double> ZInetV = ZImin;
    if (m_args->m_cut > 0.0)
    {
        int v = ceil<int>(m_args->m_cut / m_args->m_cell);
        std::vector<double> bigErode =
            erodeDiamond(ZImin, m_rows, m_cols, 2 * v);
        std::vector<double> bigOpen =
            dilateDiamond(bigErode, m_rows, m_cols, 2 * v);
        for (auto c = 0; c < m_cols; ++c)
        {
            for (auto r = 0; r < m_rows; ++r)
            {
                if (isNetCell[c * m_rows + r] == 1)
                {
                    ZInetV[c * m_rows + r] = bigOpen[c * m_rows + r];
                }
            }
        }
    }

    if (!m_args->m_dir.empty())
    {
        std::string fname =
            FileUtils::toAbsolutePath("zinet.tif", m_args->m_dir);
        MatrixXd ZInet = Map<MatrixXd>(ZInetV.data(), m_rows, m_cols);
        writeMatrix(ZInet, fname, "GTiff", m_args->m_cell, m_bounds, m_srs);
    }

    return ZInetV;
}

std::vector<double> SMRFilter::createZIpro(PointViewPtr view,
                                           std::vector<double> const& ZImin,
                                           std::vector<int> const& Low,
                                           std::vector<int> const& isNetCell,
                                           std::vector<int> const& Obj)
{
    // "The end result of the iteration process described above is a binary grid
    // where each cell is classified as being either bare earth (BE) or object
    // (OBJ). The algorithm then applies this mask to the starting minimum
    // surface to eliminate nonground cells."
    std::vector<double> ZIproV = ZImin;
    for (size_t i = 0; i < Obj.size(); ++i)
    {
        if (Obj[i] == 1 || Low[i] == 1 || isNetCell[i] == 1)
            ZIproV[i] = std::numeric_limits<double>::quiet_NaN();
    }

    // "These cells are then inpainted according to the same process described
    // previously, producing a provisional DEM (ZIpro)."
    std::vector<double> ZIpro_fillV = knnfill(view, ZIproV);

    if (!m_args->m_dir.empty())
    {
        std::string fname =
            FileUtils::toAbsolutePath("zipro.tif", m_args->m_dir);
        MatrixXd ZIpro = Map<MatrixXd>(ZIproV.data(), m_rows, m_cols);
        writeMatrix(ZIpro, fname, "GTiff", m_args->m_cell, m_bounds, m_srs);

        fname = FileUtils::toAbsolutePath("zipro_fill.tif", m_args->m_dir);
        MatrixXd ZIpro_fill = Map<MatrixXd>(ZIpro_fillV.data(), m_rows, m_cols);
        writeMatrix(ZIpro_fill, fname, "GTiff", m_args->m_cell, m_bounds,
                    m_srs);
    }

    return ZIpro_fillV;
}

// Fill voids with the average of eight nearest neighbors.
std::vector<double> SMRFilter::knnfill(PointViewPtr view,
                                       std::vector<double> const& cz)
{
    // Create a temporary PointView that encodes our raster values so that we
    // can construct a 2D KDIndex and perform nearest neighbor searches.
    PointViewPtr temp = view->makeNew();
    PointId i(0);
    for (int c = 0; c < m_cols; ++c)
    {
        for (int r = 0; r < m_rows; ++r)
        {
            if (std::isnan(cz[c * m_rows + r]))
                continue;

            temp->setField(Id::X, i,
                           m_bounds.minx + (c + 0.5) * m_args->m_cell);
            temp->setField(Id::Y, i,
                           m_bounds.miny + (r + 0.5) * m_args->m_cell);
            temp->setField(Id::Z, i, cz[c * m_rows + r]);
            i++;
        }
    }

    KD2Index& kdi = temp->build2dIndex();

    // Where the raster has voids (i.e., NaN), we search for that cell's eight
    // nearest neighbors, and fill the void with the average value of the
    // neighbors.
    std::vector<double> out = cz;
    for (int c = 0; c < m_cols; ++c)
    {
        for (int r = 0; r < m_rows; ++r)
        {
            if (!std::isnan(out[c * m_rows + r]))
                continue;

            double x = m_bounds.minx + (c + 0.5) * m_args->m_cell;
            double y = m_bounds.miny + (r + 0.5) * m_args->m_cell;
            int k = 8;
            PointIdList neighbors(k);
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

            out[c * m_rows + r] = M1;
        }
    }

    return out;
}

// Iteratively open the estimated surface. progressiveFilter can be used to
// identify both low points and object (i.e., non-ground) points, depending on
// the inputs.
std::vector<int> SMRFilter::progressiveFilter(std::vector<double> const& ZImin,
                                              double slope, double max_window)
{
    // "The maximum window radius is supplied as a distance metric (e.g., 21 m),
    // but is internally converted to a pixel equivalent by dividing it by the
    // cell size and rounding the result toward positive infinity (i.e., taking
    // the ceiling value)."
    int max_radius = static_cast<int>(std::ceil(max_window / m_args->m_cell));
    std::vector<double> prevSurface = ZImin;
    std::vector<double> prevErosion = ZImin;

    // "...the radius of the element at each step [is] increased by one pixel
    // from a starting value of one pixel to the pixel equivalent of the maximum
    // value."
    std::vector<int> Obj(m_rows * m_cols, 0);
    for (int radius = 1; radius <= max_radius; ++radius)
    {
        // "On the first iteration, the minimum surface (ZImin) is opened using
        // a disk-shaped structuring element with a radius of one pixel."
        std::vector<double> curErosion =
            erodeDiamond(prevErosion, m_rows, m_cols, 1);
        std::vector<double> curOpening =
            dilateDiamond(curErosion, m_rows, m_cols, radius);
        prevErosion = curErosion;

        // "An elevation threshold is then calculated, where the value is equal
        // to the supplied slope tolerance parameter multiplied by the product
        // of the window radius and the cell size."
        double threshold = slope * m_args->m_cell * radius;

        // "This elevation threshold is applied to the difference of the minimum
        // and the opened surfaces."

        // Need to provide means of diffing two vectors.
        std::vector<double> diff;
        std::transform(prevSurface.begin(), prevSurface.end(),
                       curOpening.begin(), std::back_inserter(diff),
                       [&](double l, double r) { return std::fabs(l - r); });

        // "Any grid cell with a difference value exceeding the calculated
        // elevation threshold for the iteration is then flagged as an OBJ
        // cell."
        std::vector<int> foo;
        std::transform(diff.begin(), diff.end(), std::back_inserter(foo),
                       [threshold](double x) {
                           return (x > threshold) ? int(1) : int(0);
                       });
        std::transform(Obj.begin(), Obj.end(), foo.begin(), Obj.begin(),
                       [](int a, int b) { return (std::max)(a, b); });

        // "The algorithm then proceeds to the next window radius (up to the
        // maximum), and proceeds as above with the last opened surface acting
        // as the minimum surface for the next difference calculation."
        prevSurface = curOpening;

        size_t ng = std::count(Obj.begin(), Obj.end(), 1);
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
