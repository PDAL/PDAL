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

#include <libpc/filters/DecimationFilterIterator.hpp>
#include <libpc/filters/DecimationFilter.hpp>
#include <libpc/exceptions.hpp>

namespace libpc { namespace filters {


DecimationFilterIterator::DecimationFilterIterator(const DecimationFilter& filter)
    : libpc::FilterIterator(filter)
    , m_stageAsDerived(filter)
{
    return;
}


void DecimationFilterIterator::skip(boost::uint64_t count)
{
    while (count > 0)
    {
        const boost::uint32_t thisCount = (boost::uint32_t)std::min<boost::uint64_t>(getChunkSize(), count);

        PointBuffer junk(getStage().getHeader().getSchema(), thisCount);
        
        const boost::uint32_t numRead = read(junk);
        if (numRead == 0) break; // end of file

        count -= numRead;

        incrementIndex(numRead);
    }

    return;
}


bool DecimationFilterIterator::atEnd() const
{
    const Iterator& iter = getPrevIterator();
    return iter.atEnd();
}


boost::uint32_t DecimationFilterIterator::read(PointBuffer& dstData)
{
    // The client has asked us for dstData.getCapacity() points.
    // We will read from our previous stage until we get that amount (or
    // until the previous stage runs out of points).

    boost::uint32_t numPointsNeeded = dstData.getCapacity();
    assert(dstData.getNumPoints() == 0);

    while (numPointsNeeded > 0)
    {
        // set up buffer to be filled by prev stage
        PointBuffer srcData(dstData.getSchemaLayout(), numPointsNeeded);

        // read from prev stage
        const boost::uint64_t srcStartIndex = getPrevIterator().getIndex();
        const boost::uint32_t numSrcPointsRead = getPrevIterator().read(srcData);
        assert(numSrcPointsRead == srcData.getNumPoints());
        assert(numSrcPointsRead <= numPointsNeeded);

        // we got no data, and there is no more to get -- exit the loop
        if (numSrcPointsRead == 0) break;

        // copy points from src (prev stage) into dst (our stage), 
        // based on the CropFilter's rules (i.e. its bounds)
        const boost::uint32_t numPointsAdded = m_stageAsDerived.processBuffer(dstData, srcData, srcStartIndex);
        incrementIndex(numPointsAdded);

        numPointsNeeded -= numPointsAdded;
    }

    const boost::uint32_t numPointsAchieved = dstData.getNumPoints();
    return numPointsAchieved;
}



} } // namespaces
