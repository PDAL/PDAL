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
#include <pdal/util/Utils.hpp>

namespace pdal
{

struct RasterLimits
{
public:
    RasterLimits(double xOrigin, double yOrigin, int width, int height, double edgeLength) :
        xOrigin(xOrigin), yOrigin(yOrigin), width(width), height(height), edgeLength(edgeLength)
    {}

    RasterLimits() : xOrigin(0), yOrigin(0), width(-1), height(-1), edgeLength(0)
    {}

    void addArgs(ProgramArgs& args)
    {
         args.add("resolution", "Cell edge size in units of X/Y", edgeLength).setPositional();
         m_xOriginArg = &args.add("origin_x", "X origin for grid", xOrigin);
         m_yOriginArg = &args.add("origin_y", "Y origin for grid", yOrigin);
         m_widthArg = &args.add("width", "Number of cells in the X direction", width);
         m_heightArg = &args.add("height", "Number of cells in the Y direction", height);
    }

    bool valid() const
        { return width > 0; }

    int checkArgs() const
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
    int width;
    int height;
    double edgeLength;

private:
    Arg *m_xOriginArg;
    Arg *m_yOriginArg;
    Arg *m_widthArg;
    Arg *m_heightArg;
};

inline bool operator==(const RasterLimits& l, const RasterLimits& r)
{
    return l.xOrigin == r.xOrigin && l.yOrigin == r.yOrigin &&
        l.width == r.width && l.height == r.height &&
        l.edgeLength == r.edgeLength;
}

inline bool operator!=(const RasterLimits& l, const RasterLimits& r)
{ return !(l == r); }


// This is a raw raster suitable for sticking into GDAL. X goes right to left, Y goes from
// top to bottom.
template <typename T>
class Raster
{
public:
    using DataVec = std::vector<T>;
    using iterator = typename DataVec::iterator;
    using const_iterator = typename DataVec::const_iterator;

    Raster(const T& initializer = T()) : Raster("", initializer)
    {}

    Raster(const std::string& name, const T& initializer = T()) :
        m_name(name), m_initializer(initializer)
    {}

    Raster(const RasterLimits& limits, const T& initializer = T()) : Raster(limits, "", initializer)
    {}

    Raster(const RasterLimits& limits, const std::string& name, const T& initializer = T()) :
        m_name(name), m_limits(limits), m_data(width() * height(), initializer),
        m_initializer(initializer)
    {}

    std::string name() const
        { return m_name; }

    int width() const
        { return m_limits.width; }

    int height() const
        { return m_limits.height; }

    double edgeLength() const
        { return m_limits.edgeLength; }

    double xOrigin() const
        { return m_limits.xOrigin; }

    double yOrigin() const
        { return m_limits.yOrigin; }

    T initializer() const
        { return m_initializer; }

    int xCell(double x) const
        { return (int)std::floor((x - m_limits.xOrigin) / m_limits.edgeLength); }

    int yCell(double y) const
        { return (int)std::floor((y - m_limits.yOrigin) / m_limits.edgeLength); }

    double xCellPos(size_t i) const
        { return m_limits.xOrigin + (i + .5) * edgeLength(); }

    double yCellPos(size_t j) const
        { return m_limits.yOrigin + (j + .5) * edgeLength(); }

    const T *data() const
        { return m_data.data(); }

    T *data()
        { return m_data.data(); }

    const T& at(int x, int y) const
        { return m_data[index(x, height() - y - 1)]; }

    T& at(int x, int y)
        { return m_data[index(x, height() - y - 1)]; }

    const T& at(size_t idx) const
        { return m_data[idx]; }

    T& at(size_t idx)
        { return m_data[idx]; }

    const T& operator[](size_t idx) const
        { return m_data[idx]; }

    T& operator[](size_t idx)
        { return m_data[idx]; }

    iterator begin()
        { return m_data.begin(); }

    const_iterator begin() const
        { return m_data.begin(); }

    iterator end()
        { return m_data.end(); }

    const_iterator end() const
        { return m_data.end(); }

    void expandToInclude(double x, double y);

    //ABELL - This should probably call expand().
    void setLimits(const RasterLimits& limits);

    const RasterLimits& limits() const
        { return m_limits; }

    size_t size() const
        { return m_data.size(); }

private:
    std::string m_name;
    RasterLimits m_limits;
    DataVec m_data;
    T m_initializer;

    Utils::StatusWithReason expand(int w, int h, int xshift, int yshift);

    // You need to pass in internal (Y down) indices.
    size_t index(int i, int j) const
        { return (j * width()) + i; }

};

using Rasterd = Raster<double>;
extern template class Raster<double>;

} // namespace pdal

