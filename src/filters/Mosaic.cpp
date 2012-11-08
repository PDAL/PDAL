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

#include <pdal/filters/Mosaic.hpp>

#include <pdal/Bounds.hpp>
#include <pdal/PointBuffer.hpp>

namespace pdal
{
namespace filters
{



Mosaic::Mosaic(const std::vector<Stage*>& prevStages, const Options& options)
    : MultiFilter(prevStages, options)
{
    return;
}


void Mosaic::initialize()
{
    MultiFilter::initialize();

    const std::vector<Stage*>& stages = getPrevStages();

    const Stage& stage0 = *stages[0];
    const SpatialReference& srs0 = stage0.getSpatialReference();
    const Schema& schema0 = stage0.getSchema();
    PointCountType pointCountType0 = stage0.getPointCountType();
    boost::uint64_t totalPoints = stage0.getNumPoints();
    Bounds<double> bigbox(stage0.getBounds());

    // we will only mosaic if all the stages have the same core properties: SRS, schema, etc
    for (boost::uint32_t i=1; i<stages.size(); i++)
    {
        Stage& stage = *(stages[i]);
        if (stage.getSpatialReference() != srs0)
            throw impedance_invalid("mosaicked stages must have same srs");
        if (stage.getSchema() != schema0)
            throw impedance_invalid("mosaicked stages must have same schema");
        if (stage.getPointCountType() == PointCount_Unknown)
            pointCountType0 = PointCount_Unknown;

        totalPoints += stage.getNumPoints();

        bigbox.grow(stage.getBounds());
    }

    if (pointCountType0 == PointCount_Unknown)
        totalPoints = 0;

    setCoreProperties(stage0);
    setPointCountType(pointCountType0);
    setNumPoints(totalPoints);

    return;
}


Options Mosaic::getDefaultOptions()
{
    Options options;
    return options;
}


pdal::StageSequentialIterator* Mosaic::createSequentialIterator(PointBuffer& buffer) const
{
    return new pdal::filters::iterators::sequential::Mosaic(*this, buffer);
}


namespace iterators
{
namespace sequential
{


Mosaic::Mosaic(const pdal::filters::Mosaic& filter, PointBuffer& buffer)
    : MultiFilterSequentialIterator(filter, buffer)
{
    return;
}


Mosaic::~Mosaic()
{
    return;
}


boost::uint64_t Mosaic::skipImpl(boost::uint64_t targetCount)
{
    boost::uint64_t count = 0;

    while (count < targetCount)
    {
        m_prevIterator = m_prevIterators[m_iteratorIndex];

        // skip as much as we can in the current stage
        boost::uint64_t thisCount = m_prevIterator->skip(count);
        count += thisCount;

        if (m_prevIterators[m_iteratorIndex]->atEnd())
        {
            ++m_iteratorIndex;
        }
        if (m_iteratorIndex == m_prevIterators.size())
        {
            // no more points
            break;
        }
    }

    return count;
}


bool Mosaic::atEndImpl() const
{
    if (m_iteratorIndex == m_prevIterators.size())
        return true;
    if (m_prevIterators[m_iteratorIndex]->atEnd())
        return true;
    return false;
}


boost::uint32_t Mosaic::readBufferImpl(PointBuffer& destData)
{
    boost::uint32_t totalNumPointsToRead = destData.getCapacity();
    boost::uint32_t totalNumPointsRead = 0;
    boost::uint32_t destPointIndex = 0;

    // for each stage, we read as many points as we can
    while (totalNumPointsRead < totalNumPointsToRead)
    {
        assert(m_iteratorIndex < m_prevIterators.size());
        m_prevIterator = m_prevIterators[m_iteratorIndex];

        // read as much as we can into temp buffer
        PointBuffer tmp(destData.getSchema(), totalNumPointsToRead-totalNumPointsRead);
        boost::uint32_t numRead = m_prevIterator->read(tmp);
        totalNumPointsRead += numRead;

        // concat the temp buffer on to end of real dest buffer
        destData.copyPointsFast(destPointIndex, 0, tmp, numRead);
        destPointIndex += numRead;
        destData.setNumPoints(destData.getNumPoints() + numRead);

        if (m_prevIterator->atEnd())
        {
            ++m_iteratorIndex;
        }
        if (m_iteratorIndex == m_prevIterators.size())
        {
            break;
        }
    }

    return totalNumPointsRead;
}


}
} // iterators::sequential

}
} // namespaces
