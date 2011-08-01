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

#include <pdal/filters/DecimationFilter.hpp>

#include <pdal/filters/DecimationFilterIterator.hpp>
#include <pdal/PointBuffer.hpp>

namespace pdal { namespace filters {


IMPLEMENT_STATICS(DecimationFilter, "filters.decimation", "Decimation Filter")


DecimationFilter::DecimationFilter(Stage& prevStage, const Options& options)
    : pdal::Filter(prevStage, options)
    , m_step(options.getValueOrThrow<boost::uint32_t>("step"))
{
    return;
}


DecimationFilter::DecimationFilter(Stage& prevStage, boost::uint32_t step)
    : Filter(prevStage, Options::none())
    , m_step(step)
{
    return;
}


void DecimationFilter::initialize()
{
    Filter::initialize();

    this->setNumPoints( this->getNumPoints() / m_step );

    return;
}


const Options& DecimationFilter::s_getDefaultOptions()
{
    static Options options;
    return options;
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

    boost::uint32_t srcIndex = 0;

    // find start point
    if ((srcStartIndex+srcIndex) % m_step != 0)
    {
        srcIndex += m_step - ( (srcStartIndex+srcIndex) % m_step ) ;
        assert((srcStartIndex+srcIndex) % m_step == 0);
    }

    while (srcIndex < numSrcPoints)
    {
        assert((srcStartIndex+srcIndex) % m_step == 0);
        
        dstData.copyPointFast(dstIndex, srcIndex, srcData);
        dstData.setNumPoints(dstIndex+1);
        ++dstIndex;
        ++numPointsAdded;
        
        srcIndex += m_step;
    }
    
    assert(dstIndex <= dstData.getCapacity());

    return numPointsAdded;
}


pdal::StageSequentialIterator* DecimationFilter::createSequentialIterator() const
{
    return new DecimationFilterSequentialIterator(*this);
}

} } // namespaces
