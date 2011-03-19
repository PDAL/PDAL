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


DecimationFilterIterator::DecimationFilterIterator(DecimationFilter& filter)
    : libpc::FilterIterator(filter)
    , m_stageAsDerived(filter)
{
    return;
}


void DecimationFilterIterator::seekToPoint(boost::uint64_t pointNum)
{
    getPrevIterator().seekToPoint(pointNum);
}


boost::uint32_t DecimationFilterIterator::readBuffer(PointData& dstData)
{
    DecimationFilter& filter = m_stageAsDerived;

    const boost::uint32_t step = filter.getStep();

    // naive implementation: read a buffer N times larger, then pull out what we need
    PointData srcData(dstData.getSchemaLayout(), dstData.getCapacity() * step);
    boost::uint32_t numSrcPointsRead = getPrevIterator().read(srcData);

    boost::uint32_t numPoints = dstData.getCapacity();
    
    boost::uint32_t srcIndex = 0;
    boost::uint32_t dstIndex = 0;
    for (dstIndex=0; dstIndex<numPoints; dstIndex++)
    {
        dstData.copyPointFast(dstIndex, srcIndex, srcData);
        dstData.setNumPoints(dstIndex+1);
        srcIndex += step;
        if (srcIndex > numSrcPointsRead) break;
    }

    return dstIndex;
}



} } // namespaces
