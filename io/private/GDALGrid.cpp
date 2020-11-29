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

GDALGrid::GDALGrid(double xOrigin, double yOrigin, size_t width, size_t height, double edgeLength,
        double radius, int outputTypes, size_t windowSize, double power) :
    m_windowSize(windowSize), m_edgeLength(edgeLength), m_radius(radius), m_power(power),
    m_outputTypes(outputTypes)
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
    RasterLimits limits(xOrigin, yOrigin, width, height, edgeLength);

    m_count.reset(new Rasterd(limits));
    if (m_outputTypes & statMin)
        m_min.reset(new Rasterd(limits, (std::numeric_limits<double>::max)()));
    if (m_outputTypes & statMax)
        m_max.reset(new Rasterd(limits, std::numeric_limits<double>::lowest()));
    if (m_outputTypes & statIdw)
    {
        m_idw.reset(new Rasterd(limits));
        m_idwDist.reset(new Rasterd(limits));
    }
    if ((m_outputTypes & statMean) || (m_outputTypes & statStdDev))
        m_mean.reset(new Rasterd(limits));
    if (m_outputTypes & statStdDev)
        m_stdDev.reset(new Rasterd(limits));
}

int GDALGrid::width() const
{
    return m_count->width();
}

int GDALGrid::height() const
{
    return m_count->height();
}

double GDALGrid::xOrigin() const
{
    return m_count->xOrigin();
}

double GDALGrid::yOrigin() const
{
    return m_count->yOrigin();
}

double GDALGrid::distance(int i, int j, double x, double y) const
{
    double x1 = m_count->xCellPos(i);
    double y1 = m_count->yCellPos(j);
    return std::sqrt(std::pow(x1 - x, 2) + std::pow(y1 - y, 2));
}

void GDALGrid::windowFill()
{
    for (int i = 0; i < width(); ++i)
        for (int j = 0; j < height(); ++j)
            if (empty(i, j))
                windowFill(i, j);
}

/**
  Expand the grid to a new size.

  /param width
*/
void GDALGrid::expandToInclude(double x, double y)
{
    m_count->expandToInclude(x, y);
    if (m_outputTypes & statMin)
        m_min->expandToInclude(x, y);
    if (m_outputTypes & statMax)
        m_max->expandToInclude(x, y);
    if (m_outputTypes & statIdw)
    {
        m_idw->expandToInclude(x, y);
        m_idwDist->expandToInclude(x, y);
    }
    if ((m_outputTypes & statMean) || (m_outputTypes & statStdDev))
        m_mean->expandToInclude(x, y);
    if (m_outputTypes & statStdDev)
        m_stdDev->expandToInclude(x, y);
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

    int iOrigin = m_count->xCell(x);
    int jOrigin = m_count->yCell(y);

    // This is a questionable case.  If a point is in a cell, shouldn't
    // it just be counted?
    double d = distance(iOrigin, jOrigin, x, y);
    if (d < m_radius &&
        iOrigin >= 0 && jOrigin >= 0 &&
        iOrigin < width() && jOrigin < height())
        update(iOrigin, jOrigin, z, d);
}


void GDALGrid::updateFirstQuadrant(double x, double y, double z)
{
    int i, j;
    int iStart;
    int iOrigin = m_count->xCell(x);
    int jOrigin = m_count->yCell(y);

    i = iStart = (std::max)(0, iOrigin + 1);
    j = (std::min)(jOrigin, (height() - 1));

    if (iStart >= width())
        return;

    while (j >= 0)
    {
        double d = distance(i, j, x, y);
        if (d < m_radius)
        {
            update(i, j, z, d);
            i++;
            if (i < width())
                continue;
        }

        // Either d >= m_radius or we've hit the end of a row (i == width),
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
    int jStart;
    int iOrigin = m_count->xCell(x);
    int jOrigin = m_count->yCell(y);

    i = (std::min)(iOrigin, (width() - 1));
    j = jStart = (std::min)(jOrigin - 1, (height() - 1));

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
    int iStart;
    int iOrigin = m_count->xCell(x);
    int jOrigin = m_count->yCell(y);

    i = iStart = (std::min)(iOrigin - 1, (width() - 1));
    j = (std::max)(jOrigin, 0);

    if (iStart < 0)
        return;

    while (j < height())
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
    int jStart;
    int iOrigin = m_count->xCell(x);
    int jOrigin = m_count->yCell(y);

    i = (std::max)(iOrigin, 0);
    j = jStart = (std::max)(jOrigin + 1, 0);

    if (jStart >= height())
        return;

    while (i < width())
    {
        double d = distance(i, j, x, y);
        if (d < m_radius)
        {
            update(i, j, z, d);
            j++;
            if (j < height())
                continue;
        }


        // Either d >= m_radius or we've hit the end of a column (j == height())
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

    double& count = m_count->at(i, j);
    count++;

    if (m_min)
    {
        double& min = m_min->at(i, j);
        min = (std::min)(val, min);
    }

    if (m_max)
    {
        double& max = m_max->at(i, j);
        max = (std::max)(val, max);
    }

    if (m_mean)
    {
        double& mean = m_mean->at(i, j);
        double delta = val - mean;

        mean += delta / count;
        if (m_stdDev)
        {
            double& stdDev = m_stdDev->at(i, j);
            stdDev += delta * (val - mean);
        }
    }

    if (m_idw)
    {
        double& idw = m_idw->at(i, j);
        double& idwDist = m_idwDist->at(i, j);

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
                idw += val / std::pow(dist, m_power);
                idwDist += 1 / std::pow(dist, m_power);
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
    {
        for (int i = 0; i < m_count->width(); ++i)
            for (int j = 0; j < m_count->height(); ++j)
                if (empty(i, j))
                    fillNodata(i, j);
    }
}


void GDALGrid::fillNodata(int i, int j)
{
    if (m_min)
        m_min->at(i, j) = std::numeric_limits<double>::quiet_NaN();
    if (m_max)
        m_max->at(i, j) = std::numeric_limits<double>::quiet_NaN();
    if (m_mean)
        m_mean->at(i, j) = std::numeric_limits<double>::quiet_NaN();
    if (m_idw)
        m_idw->at(i, j) = std::numeric_limits<double>::quiet_NaN();
    if (m_stdDev)
        m_stdDev->at(i, j) = std::numeric_limits<double>::quiet_NaN();
}


void GDALGrid::windowFill(int dstI, int dstJ)
{
    int istart = dstI > m_windowSize ? dstI - m_windowSize : (size_t)0;
    int iend = (std::min)(width(), dstI + m_windowSize + 1);
    int jstart = dstJ > m_windowSize ? dstJ - m_windowSize : (size_t)0;
    int jend = (std::min)(height(), dstJ + m_windowSize + 1);

    double distSum = 0;

    // Initialize to 0 (rather than numeric_limits::max/lowest) since we're
    // going to accumulate and average.
    if (m_min)
        m_min->at(dstI, dstJ) = 0;
    if (m_max)
        m_max->at(dstI, dstJ) = 0;

    for (int i = istart; i < iend; ++i)
        for (int j = jstart; j < jend; ++j)
        {
            if ((i == dstI && j == dstJ) || empty(i, j))
                continue;
            // The ternaries just avoid underflow UB.  We're just trying to
            // find the distance from j to dstJ or i to dstI.
            double distance = (double)(std::max)(j > dstJ ? j - dstJ : dstJ - j,
                i > dstI ? i - dstI : dstI - i);
            windowFillCell(i, j, dstI, dstJ, distance);
            distSum += (1 / distance);
        }

    // Divide summed values by the (inverse) distance sum.
    if (distSum > 0)
    {
        if (m_min)
            m_min->at(dstI, dstJ) /= distSum;
        if (m_max)
            m_max->at(dstI, dstJ) /= distSum;
        if (m_mean)
            m_mean->at(dstI, dstJ) /= distSum;
        if (m_idw)
            m_idw->at(dstI, dstJ) /= distSum;
        if (m_stdDev)
            m_stdDev->at(dstI, dstJ) /= distSum;
    }
    else
        fillNodata(dstI, dstJ);
}


void GDALGrid::windowFillCell(int srcI, int srcJ, int dstI, int dstJ, double distance)
{
    if (m_min)
        m_min->at(dstI, dstJ) += m_min->at(srcI, srcJ) / distance;
    if (m_max)
        m_max->at(dstI, dstJ) += m_max->at(srcI, srcJ) / distance;
    if (m_mean)
        m_mean->at(dstI, dstJ) += m_mean->at(srcI, srcJ) / distance;
    if (m_idw)
        m_idw->at(dstI, dstJ) += m_idw->at(srcI, srcJ) / distance;
    if (m_stdDev)
        m_stdDev->at(dstI, dstJ) += m_stdDev->at(srcI, srcJ) / distance;
}

} //namespace pdal
