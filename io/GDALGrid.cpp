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

namespace pdal
{

GDALGrid::GDALGrid(size_t width, size_t height, double edgeLength,
        double radius, double noData, int outputTypes, size_t windowSize) :
    m_width(width), m_height(height), m_windowSize(windowSize),
    m_edgeLength(edgeLength), m_radius(radius), m_noData(noData),
    m_outputTypes(outputTypes)
{
    size_t size(width * height);

    m_count.reset(new DataVec(size));
    if (m_outputTypes & statMin)
        m_min.reset(new DataVec(size, std::numeric_limits<double>::max()));
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


uint8_t *GDALGrid::data(const std::string& name)
{
    if (name == "count")
        return (m_outputTypes & statCount ?
            (uint8_t *)m_count->data() : nullptr);
    if (name == "min")
        return (m_outputTypes & statMin ?
            (uint8_t *)m_min->data() : nullptr);
    if (name == "max")
        return (m_outputTypes & statMax ?
            (uint8_t *)m_max->data() : nullptr);
    if (name == "mean")
        return (m_outputTypes & statMean ?
            (uint8_t *)m_mean->data() : nullptr);
    if (name == "idw")
        return (m_outputTypes & statIdw ?
            (uint8_t *)m_idw->data() : nullptr);
    if (name == "stdev")
        return (m_outputTypes & statStdDev ?
            (uint8_t *)m_stdDev->data() : nullptr);
    return nullptr;
}


void GDALGrid::addPoint(double x, double y, double z)
{
    int iOrigin = horizontalIndex(x);
    int jOrigin = verticalIndex(y);

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


    // First quadrant;
    int i = iOrigin + 1;
    int j = jOrigin;
    while (i < (int)m_width && j >= 0)
    {
        double d = distance(i, j, x, y);
        if (d < m_radius)
        {
            update(i, j, z, d);
            i++;
        }
        else
        {
            if (i == iOrigin + 1)
                break;
            i = iOrigin + 1;
            j--;
        }
    }

    // Second quadrant;
    i = iOrigin;
    j = jOrigin - 1;
    while (i >= 0 && j >= 0)
    {
        double d = distance(i, j, x, y);
        if (d < m_radius)
        {
            update(i, j, z, d);
            j--;
        }
        else
        {
            if (j == jOrigin - 1)
                break;
            j = jOrigin - 1;
            i--;
        }
    }

    // Third quadrant;
    i = iOrigin - 1;
    j = jOrigin;
    while (i >= 0 && j < (int)m_height)
    {
        double d = distance(i, j, x, y);
        if (d < m_radius)
        {
            update(i, j, z, d);
            i--;
        }
        else
        {
            if (i == iOrigin - 1)
                break;
            i = iOrigin - 1;
            j++;
        }
    }
    // Fourth quadrant;
    i = iOrigin;
    j = jOrigin + 1;
    while (i < (int)m_width && j < (int)m_height)
    {
        double d = distance(i, j, x, y);
        if (d < m_radius)
        {
            update(i, j, z, d);
            j++;
        }
        else
        {
            if (j == jOrigin + 1)
                break;
            j = jOrigin + 1;
            i++;
        }
    }

    // This is a questionable case.  If a point is in a cell, shouldn't
    // it just be counted?
    double d = distance(iOrigin, jOrigin, x, y);
    if (d < m_radius)
        update(iOrigin, jOrigin, z, d);
}

void GDALGrid::update(int i, int j, double val, double dist)
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
        min = std::min(val, min);
    }

    if (m_max)
    {
        double& max = (*m_max)[offset];
        max = std::max(val, max);
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
        (*m_min)[i] = m_noData;
    if (m_max)
        (*m_max)[i] = m_noData;
    if (m_mean)
        (*m_mean)[i] = m_noData;
    if (m_idw)
        (*m_idw)[i] = m_noData;
    if (m_stdDev)
        (*m_stdDev)[i] = m_noData;
}


void GDALGrid::windowFill(size_t dstI, size_t dstJ)
{
    size_t istart = dstI > m_windowSize ? dstI - m_windowSize : (size_t)0;
    size_t iend = std::min(width(), dstI + m_windowSize + 1);
    size_t jstart = dstJ > m_windowSize ? dstJ - m_windowSize : (size_t)0;
    size_t jend = std::min(height(), dstJ + m_windowSize + 1);

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
            double distance = std::max(j > dstJ ? j - dstJ : dstJ - j,
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
