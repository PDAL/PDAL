/******************************************************************************
* Copyright (c) 2020, Hobu Inc.
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

#include "Raster.hpp"

namespace pdal
{

// This should probably call expand().
template <typename T>
void Raster<T>::setLimits(const RasterLimits& limits)
{
    m_limits = limits;
    DataVec dataVec(width() * height(), m_initializer);
    m_data.swap(dataVec);
}


template <typename T>
void Raster<T>::expandToInclude(double x, double y)
{
    int xi = xCell(x);
    int yi = yCell(y);

    if (xi >= 0 && yi >= 0 && xi < width() && yi < height())
        return;

    int w = (std::max)(width(), xi + 1);
    int h = (std::max)(height(), yi + 1);
    int xshift = (std::max)(-xi, 0);
    int yshift = (std::max)(-yi, 0);

    if (xshift)
        w += xshift;
    if (yshift)
        h += yshift;
    expand(w, h, xshift, yshift);
}


template <typename T>
Utils::StatusWithReason Raster<T>::expand(int newWidth, int newHeight, int xshift, int yshift)
{
    if (newWidth < width())
        return { -1, "Expanded grid must have width at least as large as existing grid." };
    if (newHeight < height())
        return { -1, "Expanded grid must have height at least as large as existing grid." };
    if (width() + xshift > newWidth || height() + yshift > newHeight)
        return { -1, "Can't shift existing grid outside of new grid during expansion." };
    if (newWidth == width() && newHeight == height())
        return true;

    m_limits.xOrigin -= xshift * edgeLength();
    m_limits.yOrigin -= yshift * edgeLength();

    // Raster works upside down from cartesian X/Y
    yshift = newHeight - (height() + yshift);

    auto dstIndex = [newWidth, xshift, yshift](size_t i, size_t j)
    {
        return ((yshift + j) * newWidth) + i + xshift;
    };

    // Note: that i, j are internal to the raster and start at the top left and
    //   move across and down.
    DataVec& src = m_data;
    DataVec dst(newWidth * newHeight, m_initializer);
    for (int j = 0; j < height(); ++j)
    {
        size_t srcPos = index(0, j);
        size_t dstPos = dstIndex(0, j);
        std::copy(src.begin() + srcPos, src.begin() + srcPos + width(),
            dst.begin() + dstPos);
    }
    m_data = std::move(dst);
    m_limits.width = newWidth;
    m_limits.height = newHeight;
    return true;
}

// Instantiate Raster<double>
template class Raster<double>;

} // namespace pdal
