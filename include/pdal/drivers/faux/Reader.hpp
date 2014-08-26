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

#include <pdal/Reader.hpp>
#include <pdal/ReaderIterator.hpp>

namespace pdal
{
namespace drivers
{
namespace faux
{

enum Mode
{
    Constant,
    Random,
    Ramp
};


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
// In all these modes, however, the Time field is always set to the point
// number.
//
// ReturnNumber and NumberOfReturns are not included by default, but can be
// activated by passing a numeric value as "number_of_returns" to the
// reader constructor.
//
class PDAL_DLL Reader : public pdal::Reader
{

public:
    SET_STAGE_NAME("drivers.faux.reader", "Faux Reader")
    SET_STAGE_ENABLED(true)

    Reader(const Options& options);

    static Options getDefaultOptions();
    static Dimension::IdList getDefaultDimensions();

    pdal::StageSequentialIterator *createSequentialIterator() const;

private:
    uint64_t m_numPoints;
    Mode m_mode;
    uint8_t m_numberOfReturns;

    Bounds<double> m_bounds;

    virtual void processOptions(const Options& options);
    virtual void addDimensions(PointContext ctx);

    Reader& operator=(const Reader&); // not implemented
    Reader(const Reader&); // not implemented
};

} // namespace faux
} // namespace drivers

class PDAL_DLL FauxSeqIterator : public pdal::ReaderSequentialIterator
{
public:
    FauxSeqIterator(const Bounds<double>& bounds, drivers::faux::Mode mode,
                    point_count_t numPoints, uint8_t numberOfReturns, LogPtr log);

private:
    double m_minX;
    double m_maxX;
    double m_minY;
    double m_maxY;
    double m_minZ;
    double m_maxZ;
    uint64_t m_time;
    drivers::faux::Mode m_mode;
    point_count_t m_numPoints;
    uint8_t m_returnNumber;
    uint8_t m_numberOfReturns;
    LogPtr m_log;

    point_count_t readBufferImpl(PointBuffer& buf)
        { return readImpl(buf, m_numPoints); }

    uint64_t skipImpl(uint64_t numPts)
    {
        m_time += numPts;
        return numPts;
    }

    point_count_t readImpl(PointBuffer& buf, point_count_t count);
    bool atEndImpl() const
        { return false; }
};

} // namespace pdal

