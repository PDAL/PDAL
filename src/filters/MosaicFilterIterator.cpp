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

#include <pdal/filters/MosaicFilterIterator.hpp>

#include <pdal/PointBuffer.hpp>
#include <pdal/filters/MosaicFilter.hpp>

namespace pdal { namespace filters {


MosaicFilterSequentialIterator::MosaicFilterSequentialIterator(const MosaicFilter& filter)
    : MultiFilterSequentialIterator(filter)
{
    return;
}


MosaicFilterSequentialIterator::~MosaicFilterSequentialIterator()
{
    return;
}




boost::uint64_t MosaicFilterSequentialIterator::skipImpl(boost::uint64_t count)
{
    // BUG: this is clearly not correct, we need to keep track of which tile we're on
    m_prevIterators[0]->skip(count);
    return count;
}


bool MosaicFilterSequentialIterator::atEndImpl() const
{
    // BUG: this is clearly not correct, we need to keep track of which tile we're on
    return m_prevIterators[0]->atEnd();
}


boost::uint32_t MosaicFilterSequentialIterator::readImpl(PointBuffer& destData)
{
    // BUG: We know that the two prev stage schemas are compatible, 
    // but we can't be sure the have the same bitfield layouts as 
    // the buffer we've been given.  We could handle it manually if 
    // they differ, but that would be a pain for now.  (This affects
    // all filters, I guess.)

    // BUG: this doesn't account for isValid()

    boost::uint32_t totalNumPointsToRead = destData.getCapacity();
    boost::uint32_t totalNumPointsRead = 0;

    boost::uint64_t currentPointIndex = getIndex();

    int destPointIndex = 0;
    boost::uint64_t stageStartIndex = 0;

    // for each stage, we read as many points as we can
    for (size_t i=0; i<getPrevIterators().size(); i++)
    {
        StageSequentialIteratorPtr iterator = getPrevIterators()[i];
        const DataStage& stage = iterator->getStage();

        const boost::uint64_t stageStopIndex = stageStartIndex + stage.getNumPoints();

        if (currentPointIndex < stageStopIndex)
        {
            // we need to read some points from this stage

            boost::uint32_t pointsAvail = (boost::uint32_t)(stageStopIndex - currentPointIndex);
            boost::uint32_t pointsToGet = std::min(pointsAvail, totalNumPointsToRead);

            PointBuffer srcData(destData.getSchemaLayout(), pointsToGet);
            boost::uint32_t pointsGotten = iterator->read(srcData);

            for (boost::uint32_t idx=0; idx<pointsGotten; idx++)
            {
                destData.copyPointFast(destPointIndex, idx, srcData);
                destPointIndex++;
                destData.setNumPoints(idx+1);
            }

            totalNumPointsRead += pointsGotten;
            currentPointIndex += pointsGotten;
        }

        stageStartIndex += stage.getNumPoints();

        if (totalNumPointsRead == totalNumPointsToRead) break;
    }

    return totalNumPointsRead;
}


} } // namespaces
