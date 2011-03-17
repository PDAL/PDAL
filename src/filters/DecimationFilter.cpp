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
#include <libpc/exceptions.hpp>

namespace libpc { namespace filters {


DecimationFilter::DecimationFilter(Stage& prevStage, int step)
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


void DecimationFilter::seekToPoint(boost::uint64_t pointNum)
{
    m_prevStage.seekToPoint(pointNum);
}


boost::uint32_t DecimationFilter::readBuffer(PointData& dstData, const Bounds<double>& bounds)
{
    // naive implementation: read a buffer N times larger, then pull out what we need
    PointData srcData(dstData.getSchemaLayout(), dstData.getCapacity() * m_step);
    boost::uint32_t numSrcPointsRead = m_prevStage.read(srcData, bounds);

    boost::uint32_t numPoints = dstData.getCapacity();
    
    boost::uint32_t srcIndex = 0;
    boost::uint32_t dstIndex = 0;
    for (dstIndex=0; dstIndex<numPoints; dstIndex++)
    {
        dstData.copyPointFast(dstIndex, srcIndex, srcData);
        dstData.setNumPoints(dstIndex+1);
        srcIndex += m_step;
        if (srcIndex > numSrcPointsRead) break;
    }

    return dstIndex;
}


Iterator* DecimationFilter::createIterator(const Bounds<double>& bounds)
{
    throw not_yet_implemented("iterator");
}


} } // namespaces
