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

#include <pdal/PointBuffer.hpp>
#include <pdal/filters/Decimation.hpp>

#include <boost/scoped_ptr.hpp>

namespace pdal
{
namespace filters
{


Decimation::Decimation(const Options& options)
    : pdal::Filter(options),
    m_step(options.getValueOrDefault<boost::uint32_t>("step", 1))
{}


void Decimation::initialize(PointContext)
{
    if (m_step < 1)
        throw pdal_error("Decimation step value cannot be less than 1!");
    setNumPoints(this->getNumPoints() / m_step);
//    log()->get(logDEBUG) << "decimation step: " << m_step << std::endl;
}


boost::uint32_t Decimation::getStep() const
{
    return m_step;
}


boost::uint32_t Decimation::processBuffer(PointBuffer& dstData, const PointBuffer& srcData, boost::uint64_t srcStartIndex) const
{
    const boost::uint32_t numSrcPoints = srcData.getNumPoints();
    boost::uint32_t dstIndex = dstData.getNumPoints();

    boost::uint32_t numPointsAdded = 0;

    boost::uint32_t offset = getOptions().getValueOrDefault<boost::uint32_t>("offset", 0);
    boost::uint32_t srcIndex = 0;

    if ((srcStartIndex+srcIndex) % (m_step) != 0)
    {
        srcIndex += m_step - ((srcStartIndex+srcIndex) % (m_step));
    }

    srcIndex = srcIndex + offset;

    while (srcIndex < numSrcPoints)
    {

        dstData.copyPointFast(dstIndex, srcIndex, srcData);
        dstData.setNumPoints(dstIndex+1);
        ++dstIndex;
        ++numPointsAdded;

        srcIndex += m_step;
    }

    return numPointsAdded;
}


pdal::StageSequentialIterator* Decimation::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::filters::iterators::sequential::Decimation(*this, buffer);
}

pdal::StageRandomIterator* Decimation::createRandomIterator(PointBuffer& buffer) const
{
    boost::uint32_t offset =
        getOptions().getValueOrDefault<boost::uint32_t>("offset", 0);
    return new pdal::filters::iterators::random::Decimation(*this, buffer,
        offset);
}


namespace iterators
{

namespace decimation
{
IteratorBase::IteratorBase(pdal::filters::Decimation const& filter, PointBuffer& buffer)
    : m_decimationFilter(filter)
    , m_startingIndex(0)
{

}


boost::uint32_t IteratorBase::decimateData(PointBuffer& data, StageSequentialIterator& iterator)
{
    // The client has asked us for dstData.getCapacity() points.
    // We will read from our previous stage until we get that amount (or
    // until the previous stage runs out of points).

    boost::uint32_t originalCapacity = data.getCapacity();
    boost::int64_t numPointsNeeded = static_cast<boost::int64_t>(data.getCapacity());

    if (numPointsNeeded <=0)
        throw pdal_error("numPointsNeeded is <=0!");

    // we've established numPointsNeeded is > 0
    PointBuffer outputData(data.getSchema(), static_cast<boost::uint32_t>(numPointsNeeded));
    PointBuffer tmpData(data.getSchema(), static_cast<boost::uint32_t>(numPointsNeeded));

    m_decimationFilter.log()->get(logDEBUG2) << "Fetching for block of size: " << numPointsNeeded << std::endl;

    m_startingIndex =  iterator.getIndex();
    bool logOutput = m_decimationFilter.log()->getLevel() > logDEBUG3;

    while (numPointsNeeded > 0)
    {
        if (iterator.atEnd())
        {
            if (logOutput)
                m_decimationFilter.log()->get(logDEBUG4)  << "previous iterator is .atEnd, stopping"
                        << std::endl;
            break;
        }

        if (data.getCapacity() < numPointsNeeded)
        {
            data.resize(static_cast<boost::uint32_t>(numPointsNeeded));
            if (logOutput)
                m_decimationFilter.log()->get(logDEBUG4) << "Resizing original buffer to "
                        << numPointsNeeded
                        << std::endl;

        }

        boost::uint32_t numRead = iterator.read(data);
        if (logOutput)
            m_decimationFilter.log()->get(logDEBUG4) << "Fetched "
                    << numRead << " from previous iterator. "
                    << std::endl;

        if (tmpData.getCapacity() < numRead)
            tmpData.resize(numRead);

        boost::uint32_t numKept = m_decimationFilter.processBuffer(tmpData, data, m_startingIndex);
        if (logOutput)
            m_decimationFilter.log()->get(logDEBUG4)  << "Kept " << numKept
                    << " in decimation filter" << std::endl;

        data.resize(originalCapacity);
        m_startingIndex = iterator.getIndex();

        if (logOutput)
        {
            m_decimationFilter.log()->get(logDEBUG4)  << tmpData.getNumPoints()
                    << " in temp buffer from filter"
                    << std::endl;
            m_decimationFilter.log()->get(logDEBUG4)  << "Starting index is now "<< m_startingIndex
                    << std::endl;

        }

        if (tmpData.getNumPoints() > 0)
        {
            outputData.copyPointsFast(outputData.getNumPoints(), 0, tmpData, tmpData.getNumPoints());
            outputData.setNumPoints(outputData.getNumPoints() + tmpData.getNumPoints());
            tmpData.setNumPoints(0);
            tmpData.resize(originalCapacity);
        }

        numPointsNeeded -= numKept;
        if (logOutput)
            m_decimationFilter.log()->get(logDEBUG4)  << numPointsNeeded
                    << " left to fill this block"
                    << std::endl;
        if (numPointsNeeded <= 0)
        {
            if (logOutput)
                m_decimationFilter.log()->get(logDEBUG4) << "numPointsNeeded <=0, stopping"
                        << std::endl;
            break;
        }

    }

    const boost::uint32_t numPointsAchieved = outputData.getNumPoints();

    if (numPointsAchieved)
    {
        data.resize(originalCapacity);
        data.setNumPoints(0);
        data.copyPointsFast(0, 0, outputData, outputData.getNumPoints());
        data.setNumPoints(outputData.getNumPoints());
        if (logOutput)
            m_decimationFilter.log()->get(logDEBUG2)  << "Copying " << outputData.getNumPoints()
                    << " at end of readBufferImpl"
                    << std::endl;
    }

    return numPointsAchieved;

}


} //decimation
namespace sequential
{


Decimation::Decimation(const pdal::filters::Decimation& filter,
        PointBuffer& buffer) :
    pdal::FilterSequentialIterator(filter, buffer),
    decimation::IteratorBase(filter, buffer)
{}


boost::uint64_t Decimation::skipImpl(boost::uint64_t count)
{
    // BUG: this is not exactly correct
    return getPrevIterator().skip(count * m_decimationFilter.getStep());
}

bool Decimation::atEndImpl() const
{
    const StageSequentialIterator& iter = getPrevIterator();
    return iter.atEnd();
}

boost::uint32_t Decimation::readBufferImpl(PointBuffer& data)
{
    return decimateData(data, getPrevIterator());
}


} // sequential


namespace random
{
Decimation::Decimation(const pdal::filters::Decimation& filter,
        PointBuffer& buffer, boost::uint32_t offset)
    : pdal::FilterRandomIterator(filter, buffer)
    , decimation::IteratorBase(filter, buffer), m_offset(offset)
{
}

boost::uint64_t Decimation::seekImpl(boost::uint64_t count)
{
    m_startingIndex = count + m_offset;
    return getPrevIterator().seek(m_startingIndex);
}

boost::uint32_t Decimation::readBufferImpl(PointBuffer& buffer)
{
    PointBuffer sequential(buffer.getSchema(), 65535u);

    boost::scoped_ptr<StageSequentialIterator> iter(m_decimationFilter.getPrevStage().createSequentialIterator(buffer));
    iter->skip(m_startingIndex);
    return decimateData(buffer, *iter);

}

} // random

} // iterators
} // filters
} // pdal
