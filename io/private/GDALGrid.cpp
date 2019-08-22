/******************************************************************************
* Copyright (c) 2016, Hobu Inc. (info@hobu.co)
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

#include "GDALGrid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <iostream>
#include <pdal/pdal_types.hpp>

namespace pdal
{

//ABELL - In the beginning this data needed to be contiguous, as it was passed
//  directly to GDAL to write.  Since we started supporting various data types
//  in GDAL output, we end up copying/casting data to a block for output,
//  so there's no reason that we must have contiguous data -- we just need
//  an iterator that allows traversal of the data in row-major order.  So,
//  this should probably be re-implemented in some way that doesn't require
//  moving data around every time the grid is resized.

GDALGrid::GDALGrid(size_t width, size_t height, double edgeLength,
        double radius, int outputTypes, size_t windowSize) :
    m_width(width), m_height(height), m_windowSize(windowSize),
    m_edgeLength(edgeLength), m_radius(radius), m_outputTypes(outputTypes)
{
    if (width > (size_t)(std::numeric_limits<int>::max)() ||
        height > (size_t)(std::numeric_limits<int>::max)())
    {
        std::ostringstream oss;
        oss << "Grid width or height is too large. Width and height are "
            "limited to " << (std::numeric_limits<int>::max)() << " cells."
            "Try setting bounds or increasing resolution.";
        throw error(oss.str());
    }
    size_t size(width * height);

    m_count.reset(new DataVec(size));
    if (m_outputTypes & statMin)
        m_min.reset(new DataVec(size, (std::numeric_limits<double>::max)()));
    if (m_outputTypes & statMax)
        m_max.reset(new DataVec(size, std::numeric_limits<double>::lowest()));
    if (m_outputTypes & statIdw)
    {
        m_idw.reset(new DataVec(size));
        m_idwDist.reset(new DataVec(size));
    }
    if ((m_outputTypes & statMean) || (m_outputTypes & statStdDev))
        m_mean.reset(new DataVec(size));
    if (m_outputTypes & statStdDev)
        m_stdDev.reset(new DataVec(size));
}


/**
  Expand the grid to a new size.

  /param width
*/
void GDALGrid::expand(size_t width, size_t height, size_t xshift, size_t yshift)
{
    if (width < m_width)
        throw error("Expanded grid must have width at least as large "
            "as existing grid.");
    if (height < m_height)
        throw error("Expanded grid must have height at least as large "
            "as existing grid.");
    if (m_width + xshift > width || m_height + yshift > height)
        throw error("Can't shift existing grid outside of new grid "
            "during expansion.");
    if (width == m_width && height == m_height)
        return;

    // Grid (raster) works upside down from standard X/Y.
    yshift = height - (m_height + yshift);
    auto moveVec = [=](DataPtr& src, double initializer)
    {
        // Compute an index in the destination given source index coords.
        auto dstIndex = [width, xshift, yshift](size_t i, size_t j)
        {
            return ((yshift + j) * width) + i + xshift;
        };

        size_t size(width * height);
        DataPtr dst(new DataVec(size, initializer));
        for (size_t j = 0; j < m_height; ++j)
        {
            size_t srcPos = index(0, j);
            size_t dstPos = dstIndex(0, j);
            std::copy(src->begin() + srcPos, src->begin() + srcPos + m_width,
                dst->begin() + dstPos);
        }
        src = std::move(dst);
    };

    moveVec(m_count, 0);
    if (m_outputTypes & statMin)
        moveVec(m_min, (std::numeric_limits<double>::max)());
    if (m_outputTypes & statMax)
        moveVec(m_max, std::numeric_limits<double>::lowest());
    if (m_outputTypes & statIdw)
    {
        moveVec(m_idw, 0);
        moveVec(m_idwDist, 0);
    }
    if ((m_outputTypes & statMean) || (m_outputTypes & statStdDev))
        moveVec(m_mean, 0);
    if (m_outputTypes & statStdDev)
        moveVec(m_stdDev, 0);
    m_width = width;
    m_height = height;
}


int GDALGrid::numBands() const
{
    int num = 0;

    if (m_outputTypes & statCount)
        num++;
    if (m_outputTypes & statMin)
        num++;
    if (m_outputTypes & statMax)
        num++;
    if (m_outputTypes & statMean)
        num++;
    if (m_outputTypes & statIdw)
        num++;
    if (m_outputTypes & statStdDev)
        num++;
    return num;
}


double *GDALGrid::data(const std::string& name)
{
    if (name == "count" && (m_outputTypes & statCount))
        return m_count->data();
    if (name == "min" && (m_outputTypes & statMin))
        return m_min->data();
    if (name == "max" && (m_outputTypes & statMax))
        return m_max->data();
    if (name == "mean" && (m_outputTypes & statMean))
        return m_mean->data();
    if (name == "idw" && (m_outputTypes & statIdw))
         return m_idw->data();
    if (name == "stdev" && (m_outputTypes & statStdDev))
        return m_stdDev->data();
    return nullptr;
}


void GDALGrid::addPoint(double x, double y, double z)
{
    // Here's the logic... we divide the cells around the subject cell
    // (at iOrigin, jOrigin) into four quadrants.  We move outward from the
    // subject cell, checking distance until we find that we're farther than
    // permitted by the radius, then we move up (down, over, whatever) a row
    // or column and do it again until we find a case where there's not a
    // single cell in the row that meets our criteria.
    // There are easier ways to figure out which cells will be close enough,
    // be we need the precise distance for all those cells that we already
    // know will qualify, so I don't think there's much overhead here that
    // we can avoid.

    // The four quadrant cases could certainly be merged into one, but I
    // think it would be harder to follow and there's really not that
    // much code here.

    // The quadrants are the standard mathematical ones.  Here's a picture
    // of how things kind of work.  At the end we update the central cell.

    //            ^ ->
    //          ^ | -->
    //        ^ | | --->
    //      ^ | | | ---->
    //   <------- X ------>
    //    <------ | | | v
    //     <----- | | v
    //       <--- | v
    //         <- v

    updateFirstQuadrant(x, y, z);
    updateSecondQuadrant(x, y, z);
    updateThirdQuadrant(x, y, z);
    updateFourthQuadrant(x, y, z);

    int iOrigin = horizontalIndex(x);
    int jOrigin = verticalIndex(y);

    // This is a questionable case.  If a point is in a cell, shouldn't
    // it just be counted?
    double d = distance(iOrigin, jOrigin, x, y);
    if (d < m_radius &&
        iOrigin >= 0 && jOrigin >= 0 &&
        iOrigin < (int)m_width && jOrigin < (int)m_height)
        update(iOrigin, jOrigin, z, d);
}


void GDALGrid::updateFirstQuadrant(double x, double y, double z)
{
    int i, j;
    int iStart, jStart;
    int iOrigin = horizontalIndex(x);
    int jOrigin = verticalIndex(y);

    i = iStart = (std::max)(0, iOrigin + 1);
    j = (std::min)(jOrigin, int(m_height - 1));

    if (iStart >= (int)m_width)
        return;

    while (j >= 0)
    {
        double d = distance(i, j, x, y);
        if (d < m_radius)
        {
            update(i, j, z, d);
            i++;
            if (i < (int)m_width)
                continue;
        }

        // Either d >= m_radius or we've hit the end of a row (i == m_width),
        // so move to the next row.
        if (i == iStart)
            break;
        i = iStart;
        j--;
    }
}


void GDALGrid::updateSecondQuadrant(double x, double y, double z)
{
    int i, j;
    int iStart, jStart;
    int iOrigin = horizontalIndex(x);
    int jOrigin = verticalIndex(y);

    i = (std::min)(iOrigin, int(m_width - 1));
    j = jStart = (std::min)(jOrigin - 1, int(m_height - 1));

    if (jStart < 0)
        return;

    while (i >= 0)
    {
        double d = distance(i, j, x, y);
        if (d < m_radius)
        {
            update(i, j, z, d);
            j--;
            if (j >= 0)
                continue;
        }

        // Either d >= m_radius or we've hit the end of a column (j < 0),
        // so move to the next column.
        if (j == jStart)
            break;
        j = jStart;
        i--;
    }
}


void GDALGrid::updateThirdQuadrant(double x, double y, double z)
{
    int i, j;
    int iStart, jStart;
    int iOrigin = horizontalIndex(x);
    int jOrigin = verticalIndex(y);

    i = iStart = (std::min)(iOrigin - 1, int(m_width - 1));
    j = (std::max)(jOrigin, 0);

    if (iStart < 0)
        return;

    while (j < (int)m_height)
    {
        double d = distance(i, j, x, y);
        if (d < m_radius)
        {
            update(i, j, z, d);
            i--;
            if (i >= 0)
                continue;
        }

        // Either d >= m_radius or we've hit the end of a row (i < 0),
        // so move to the next row.
        if (i == iStart)
            break;
        i = iStart;
        j++;
    }
}


void GDALGrid::updateFourthQuadrant(double x, double y, double z)
{

    int i, j;
    int iStart, jStart;
    int iOrigin = horizontalIndex(x);
    int jOrigin = verticalIndex(y);

    i = (std::max)(iOrigin, 0);
    j = jStart = (std::max)(jOrigin + 1, 0);

    if (jStart >= (int)m_height)
        return;

    while (i < (int)m_width)
    {
        double d = distance(i, j, x, y);
        if (d < m_radius)
        {
            update(i, j, z, d);
            j++;
            if (j < (int)m_height)
                continue;
        }


        // Either d >= m_radius or we've hit the end of a column (j == m_height)
        // so move to the next row.
        if (j == jStart)
            break;
        j = jStart;
        i++;
    }
}


void GDALGrid::update(size_t i, size_t j, double val, double dist)
{
    // Once we determine that a point is close enough to a cell to count it,
    // this function does the actual math.  We use the value of the
    // point (val) and its distance from the cell center (dist).  There's
    // a little math that needs to be done once all points are added.  See
    // finalize() for that.

    // See
    // https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
    // https://en.wikipedia.org/wiki/Inverse_distance_weighting

    size_t offset = index(i, j);

    double& count = (*m_count)[offset];
    count++;

    if (m_min)
    {
        double& min = (*m_min)[offset];
        min = (std::min)(val, min);
    }

    if (m_max)
    {
        double& max = (*m_max)[offset];
        max = (std::max)(val, max);
    }

    if (m_mean)
    {
        double& mean = (*m_mean)[offset];
        double delta = val - mean;

        mean += delta / count;
        if (m_stdDev)
        {
            double& stdDev = (*m_stdDev)[offset];
            stdDev += delta * (val - mean);
        }
    }

    if (m_idw)
    {
        double& idw = (*m_idw)[offset];
        double& idwDist = (*m_idwDist)[offset];

        // If the distance is 0, we set the idwDist to nan to signal that
        // we should ignore the distance and take the value as is.
        if (!std::isnan(idwDist))
        {
            if (dist == 0)
            {
                idw = val;
                idwDist = std::numeric_limits<double>::quiet_NaN();
            }
            else
            {
                idw += val / dist;
                idwDist += 1 / dist;
            }
        }
    }
}

void GDALGrid::finalize()
{
    // See
    // https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
    // https://en.wikipedia.org/wiki/Inverse_distance_weighting
    if (m_stdDev)
        for (size_t i = 0; i < m_count->size(); ++i)
            if (!empty(i))
                (*m_stdDev)[i] = sqrt((*m_stdDev)[i] / (*m_count)[i]);

    if (m_idw)
        for (size_t i = 0; i < m_count->size(); ++i)
            if (!empty(i))
            {
                double& distSum = (*m_idwDist)[i];
                if (!std::isnan(distSum))
                    (*m_idw)[i] /= distSum;
            }

    if (m_windowSize > 0)
        windowFill();
    else
        for (size_t i = 0; i < m_count->size(); ++i)
            if (empty(i))
                fillNodata(i);
}


void GDALGrid::fillNodata(size_t i)
{
    if (m_min)
        (*m_min)[i] = std::numeric_limits<double>::quiet_NaN();
    if (m_max)
        (*m_max)[i] = std::numeric_limits<double>::quiet_NaN();
    if (m_mean)
        (*m_mean)[i] = std::numeric_limits<double>::quiet_NaN();
    if (m_idw)
        (*m_idw)[i] = std::numeric_limits<double>::quiet_NaN();
    if (m_stdDev)
        (*m_stdDev)[i] = std::numeric_limits<double>::quiet_NaN();
}


void GDALGrid::windowFill(size_t dstI, size_t dstJ)
{
    size_t istart = dstI > m_windowSize ? dstI - m_windowSize : (size_t)0;
    size_t iend = (std::min)(width(), dstI + m_windowSize + 1);
    size_t jstart = dstJ > m_windowSize ? dstJ - m_windowSize : (size_t)0;
    size_t jend = (std::min)(height(), dstJ + m_windowSize + 1);

    double distSum = 0;
    size_t dstIdx = index(dstI, dstJ);

    // Initialize to 0 (rather than numeric_limits::max/lowest) since we're
    // going to accumulate and average.
    if (m_min)
        (*m_min)[dstIdx] = 0;
    if (m_max)
        (*m_max)[dstIdx] = 0;

    for (size_t i = istart; i < iend; ++i)
        for (size_t j = jstart; j < jend; ++j)
        {
            size_t srcIdx = index(i, j);
            if ((srcIdx == dstIdx) || empty(srcIdx))
                continue;
            // The ternaries just avoid underflow UB.  We're just trying to
            // find the distance from j to dstJ or i to dstI.
            double distance = (double)(std::max)(j > dstJ ? j - dstJ : dstJ - j,
                i > dstI ? i - dstI : dstI - i);
            windowFillCell(srcIdx, dstIdx, distance);
            distSum += (1 / distance);
        }

    // Divide summed values by the (inverse) distance sum.
    if (distSum > 0)
    {
        if (m_min)
            (*m_min)[dstIdx] /= distSum;
        if (m_max)
            (*m_max)[dstIdx] /= distSum;
        if (m_mean)
            (*m_mean)[dstIdx] /= distSum;
        if (m_idw)
            (*m_idw)[dstIdx] /= distSum;
        if (m_stdDev)
            (*m_stdDev)[dstIdx] /= distSum;
    }
    else
        fillNodata(dstIdx);
}


void GDALGrid::windowFillCell(size_t srcIdx, size_t dstIdx, double distance)
{
    if (m_min)
        (*m_min)[dstIdx] += (*m_min)[srcIdx] / distance;
    if (m_max)
        (*m_max)[dstIdx] += (*m_max)[srcIdx] / distance;
    if (m_mean)
        (*m_mean)[dstIdx] += (*m_mean)[srcIdx] / distance;
    if (m_idw)
        (*m_idw)[dstIdx] += (*m_idw)[srcIdx] / distance;
    if (m_stdDev)
        (*m_stdDev)[dstIdx] += (*m_stdDev)[srcIdx] / distance;
}

} //namespace pdal
