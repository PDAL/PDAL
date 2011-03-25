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

#include <libpc/filters/DecimationFilter.hpp>

#include <libpc/filters/DecimationFilterIterator.hpp>
#include <libpc/Header.hpp>
#include <libpc/PointBuffer.hpp>

namespace libpc { namespace filters {


DecimationFilter::DecimationFilter(const Stage& prevStage, boost::uint32_t step)
    : Filter(prevStage)
    , m_step(step)
{
    Header& header = getHeader();
    header.setNumPoints( header.getNumPoints() / step );

    return;
}


const std::string& DecimationFilter::getName() const
{
    static std::string name("Decimation Filter");
    return name;
}


boost::uint32_t DecimationFilter::getStep() const
{
    return m_step;
}


boost::uint32_t DecimationFilter::processBuffer(PointBuffer& dstData, const PointBuffer& srcData, boost::uint64_t srcStartIndex) const
{
    const boost::uint32_t numSrcPoints = srcData.getNumPoints();
    boost::uint32_t dstIndex = dstData.getNumPoints();

    boost::uint32_t numPointsAdded = 0;

    for (boost::uint32_t srcIndex=0; srcIndex<numSrcPoints; srcIndex++)
    {
        if ((srcStartIndex+srcIndex) % m_step == 0)
        {
            dstData.copyPointFast(dstIndex, srcIndex, srcData);
            dstData.setNumPoints(dstIndex+1);
            ++dstIndex;
            ++numPointsAdded;
        }
    }
    
    assert(dstIndex <= dstData.getCapacity());

    return numPointsAdded;
}


libpc::SequentialIterator* DecimationFilter::createSequentialIterator() const
{
    return new DecimationFilterSequentialIterator(*this);
}

libpc::Iterator* DecimationFilter::createIterator(StageIteratorType t) const
{
    if (t == StageIterator_Sequential)
        return new DecimationFilterSequentialIterator(*this);
    
    return 0;
}



} } // namespaces
