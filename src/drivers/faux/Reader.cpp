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

#include <pdal/drivers/faux/Reader.hpp>

#include <pdal/PointBuffer.hpp>

#include <boost/algorithm/string.hpp>


namespace pdal
{
namespace drivers
{
namespace faux
{

static Mode string2mode(const std::string& str)
{
    if (boost::iequals(str, "constant")) return Constant;
    if (boost::iequals(str, "random")) return Random;
    if (boost::iequals(str, "ramp")) return Ramp;
    throw pdal_error("invalid Mode option: " + str);
}


Reader::Reader(const Options& options)
    : pdal::Reader(options)
{}


void Reader::processOptions(const Options& options)
{
    m_bounds = options.getValueOrThrow<Bounds<double>>("bounds");
    m_numPoints = options.getValueOrThrow<uint64_t>("num_points");
    m_mode = string2mode(options.getValueOrThrow<std::string>("mode"));
    // boost::lexical_cast, which is used by option.getValue<>, doesn't deal
    // nicely with uint8_t.
    // http://www.boost.org/doc/libs/1_56_0/doc/html/boost_lexical_cast/frequently_asked_questions.html
    m_numberOfReturns = boost::numeric_cast<uint8_t>(
            options.getValueOrDefault<int>("number_of_returns", 0));
}


void Reader::addDimensions(PointContext ctx)
{
    ctx.registerDims(getDefaultDimensions());
    if (m_numberOfReturns > 0)
    {
        ctx.registerDim(Dimension::Id::ReturnNumber);
        ctx.registerDim(Dimension::Id::NumberOfReturns);
    }
}


Dimension::IdList Reader::getDefaultDimensions()
{
    Dimension::IdList ids;

    ids.push_back(Dimension::Id::X);
    ids.push_back(Dimension::Id::Y);
    ids.push_back(Dimension::Id::Z);
    ids.push_back(Dimension::Id::OffsetTime);
    return ids;
}


Options Reader::getDefaultOptions()
{
    Options options;
    return options;
}


pdal::StageSequentialIterator* Reader::createSequentialIterator() const
{
    return new FauxSeqIterator(m_bounds, m_mode, m_numPoints,
                               m_numberOfReturns, log());
}

} // namespace faux
} // namespace drivers


FauxSeqIterator::FauxSeqIterator(const Bounds<double>& bounds,
        drivers::faux::Mode mode, point_count_t numPoints,
        uint8_t numberOfReturns, LogPtr log) :
    m_time(0), m_mode(mode), m_numPoints(numPoints), 
    m_returnNumber(1), m_numberOfReturns(numberOfReturns), m_log(log)
{
    const std::vector<Range<double>>& ranges = bounds.dimensions();
    m_minX = ranges[0].getMinimum();
    m_maxX = ranges[0].getMaximum();
    m_minY = ranges[1].getMinimum();
    m_maxY = ranges[1].getMaximum();
    m_minZ = ranges[2].getMinimum();
    m_maxZ = ranges[2].getMaximum();
}


point_count_t FauxSeqIterator::readImpl(PointBuffer& buf, point_count_t count)
{
    using namespace pdal::drivers::faux;

    const double numDeltas = (double)count - 1.0;
    const double delX = (m_maxX - m_minX) / numDeltas;
    const double delY = (m_maxY - m_minY) / numDeltas;
    const double delZ = (m_maxZ - m_minZ) / numDeltas;

    m_log->get(LogLevel::Debug5) << "Reading a point buffer of " <<
        count << " points." << std::endl;

    for (PointId idx = 0; idx < count; ++idx)
    {
        double x;
        double y;
        double z;
        switch (m_mode)
        {
            case Random:
                x = Utils::random(m_minX, m_maxX);
                y = Utils::random(m_minY, m_maxY);
                z = Utils::random(m_minZ, m_maxZ);
                break;
            case Constant:
                x = m_minX;
                y = m_minY;
                z = m_minZ;
                break;
            case Ramp:
                x = m_minX + delX * idx;
                y = m_minY + delY * idx;
                z = m_minZ + delZ * idx;
                break;
        }

        buf.setField(Dimension::Id::X, idx, x);
        buf.setField(Dimension::Id::Y, idx, y);
        buf.setField(Dimension::Id::Z, idx, z);
        buf.setField(Dimension::Id::OffsetTime, idx, m_time++);
        if (m_numberOfReturns > 0)
        {
            buf.setField(Dimension::Id::ReturnNumber, idx, m_returnNumber);
            buf.setField(Dimension::Id::NumberOfReturns, idx, m_numberOfReturns);
            m_returnNumber = (m_returnNumber % 10) + 1;
        }
    }
    return count;
}

} // namespace pdal

