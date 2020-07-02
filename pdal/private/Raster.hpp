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

#pragma once

#include <vector>

#include <pdal/util/ProgramArgs.hpp>

namespace pdal
{

struct RasterLimits
{
public:
    void addArgs(ProgramArgs& args)
    {
         args.add("resolution", "Cell edge size in units of X/Y", edgeLength).setPositional();
         m_xOriginArg = &args.add("origin_x", "X origin for grid", xOrigin);
         m_yOriginArg = &args.add("origin_y", "Y origin for grid", yOrigin);
         m_widthArg = &args.add("width", "Number of cells in the X direction", width);
         m_heightArg = &args.add("height", "Number of cells in the Y direction", height);
    }

    int checkArgs()
    {
        int args = 0;
        if (m_xOriginArg->set())
            args++;
        if (m_yOriginArg->set())
            args++;
        if (m_widthArg->set())
            args++;
        if (m_heightArg->set())
            args++;
        return args;
    }

public:
    double xOrigin;
    double yOrigin;
    size_t width;
    size_t height;
    double edgeLength;

private:
    Arg *m_xOriginArg;
    Arg *m_yOriginArg;
    Arg *m_widthArg;
    Arg *m_heightArg;
};

// This is a raw raster suitable for sticking into GDAL. X goes right to left, Y goes from
// top to bottom.
template <typename T>
class Raster
{
public:
    using iterator = typename std::vector<T>::iterator;
    using const_iterator = typename std::vector<T>::const_iterator;

    Raster(const T& initializer = T()) : m_initializer(initializer)
    {}

    Raster(const RasterLimits& limits, const T& initializer = T()) :
        m_limits(limits), m_data(width() * height(), initializer), m_initializer(initializer)
    {}

    size_t width() const
        { return m_limits.width; }

    size_t height() const
        { return m_limits.height; }

    double edgeLength() const
        { return m_limits.edgeLength; }

    double xOrigin() const
        { return m_limits.xOrigin; }

    double yOrigin() const
        { return m_limits.yOrigin; }

    size_t index(size_t i, size_t j) const
        { return (j * width()) + i; }

    int horizontalIndex(double x) const
        { return (int)(x / edgeLength()); }

    int verticalIndex(double y) const
        { return height() - (int)(y / edgeLength()) - 1; }

    double horizontalPos(size_t i) const
        { return (i + .5) * edgeLength(); }

    double verticalPos(size_t j) const
        { return (height() - (j + .5)) * edgeLength(); }

    const T *data() const
        { return m_data.data(); }

    T *data()
        { return m_data.data(); }

    const T& at(size_t x, size_t y) const
        { return m_data[index(x, y)]; }

    T& at(size_t x, size_t y)
        { return m_data[index(x, y)]; }

    iterator begin()
        { return m_data.begin(); }

    const_iterator begin() const
        { return m_data.begin(); }

    iterator end()
        { return m_data.end(); }

    const_iterator end() const
        { return m_data.end(); }

    void setLimits(const RasterLimits& limits)
    {
        m_limits = limits;
        m_data.swap(std::vector<T>(width() * height(), m_initializer));
    }

    const RasterLimits& limits() const
        { return m_limits; }

private:
    RasterLimits m_limits;
    std::vector<T> m_data;
    T m_initializer;
};

using Rasterd = Raster<double>;

} // namespace pdal

