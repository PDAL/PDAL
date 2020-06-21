/******************************************************************************
* Copyright (c) 2011, Michael P. Gerlek (mpg@flaxen.com)
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

#include <random>

#include <pdal/Reader.hpp>
#include <pdal/Streamable.hpp>

namespace pdal
{

enum class Mode
{
    Constant,
    Ramp,
    Uniform,
    Normal,
    Grid
};

inline std::istream& operator>>(std::istream& in, Mode& m)
{
    std::string s;

    in >> s;
    s = Utils::tolower(s);
    if (s == "constant")
        m = Mode::Constant;
    else if (s == "random" || s == "uniform")
        m = Mode::Uniform;
    else if (s == "ramp")
        m = Mode::Ramp;
    else if (s == "normal")
        m = Mode::Normal;
    else if (s == "grid")
        m = Mode::Grid;
    else
        in.setstate(std::ios::failbit);
    return in;
}

inline std::ostream& operator<<(std::ostream& out, const Mode& m)
{
    switch (m)
    {
    case Mode::Constant:
        out << "Constant";
    case Mode::Ramp:
        out << "Ramp";
    case Mode::Uniform:
        out << "Uniform";
    case Mode::Normal:
        out << "Normal";
    case Mode::Grid:
        out << "Grid";
    }
    return out;
}

// The FauxReader doesn't read from disk, but instead just makes up data for its
// points.  The reader is constructed with a given bounding box and a given
// number of points.
//
// This reader knows about these fields (Dimensions):
//    X,Y,Z - floats
//    Time  - uint64
//    ReturnNumber (optional) - uint8
//    NumberOfReturns (optional) - uint8
//
// It supports a few modes:
//   - "random" generates points that are randomly distributed within the
//     given bounding box
//   - "constant" generates its points to always be at the minimum of the
//      bounding box
//   - "ramp" generates its points as a linear ramp from the minimum of the
//     bbox to the maximum
//   - "uniform" generates points that are uniformly distributed within the
//     given bounding box
//   - "normal" generates points that are normally distributed with a given
//     mean and standard deviation in each of the XYZ dimensions
// In all these modes, however, the Time field is always set to the point
// number.
//
// ReturnNumber and NumberOfReturns are not included by default, but can be
// activated by passing a numeric value as "number_of_returns" to the
// reader constructor.
//
class PDAL_DLL FauxReader : public Reader, public Streamable
{
public:
    FauxReader()
    {}

    std::string getName() const;

private:
    using nd = std::normal_distribution<double>;
    using urd = std::uniform_real_distribution<double>;

    Mode m_mode;
    BOX3D m_bounds;
    double m_mean_x;
    double m_mean_y;
    double m_mean_z;
    double m_stdev_x;
    double m_stdev_y;
    double m_stdev_z;
    double m_delX;
    double m_delY;
    double m_delZ;
    uint64_t m_time;
    int m_numReturns;
    int m_returnNum;
    point_count_t m_index;
    Arg *m_seedArg;
    uint32_t m_seed;
    std::mt19937 m_generator;
    std::unique_ptr<nd> m_normalX;
    std::unique_ptr<nd> m_normalY;
    std::unique_ptr<nd> m_normalZ;
    std::unique_ptr<urd> m_uniformX;
    std::unique_ptr<urd> m_uniformY;
    std::unique_ptr<urd> m_uniformZ;

    virtual void addArgs(ProgramArgs& args);
    virtual void prepared(PointTableRef table);
    virtual void initialize();
    virtual void addDimensions(PointLayoutPtr layout);
    virtual void ready(PointTableRef table);
    virtual bool processOne(PointRef& point);
    virtual point_count_t read(PointViewPtr view, point_count_t count);
    virtual bool eof()
        { return false; }

    FauxReader& operator=(const FauxReader&); // not implemented
    FauxReader(const FauxReader&); // not implemented
};

} // namespace pdal
