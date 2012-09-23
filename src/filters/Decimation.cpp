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

#include <pdal/filters/Decimation.hpp>

#include <pdal/PointBuffer.hpp>

namespace pdal
{
namespace filters
{


Decimation::Decimation(Stage& prevStage, const Options& options)
    : pdal::Filter(prevStage, options)
    , m_step(options.getValueOrDefault<boost::uint32_t>("step", 1))
{
    return;
}


Decimation::Decimation(Stage& prevStage, boost::uint32_t step)
    : Filter(prevStage, Options::none())
    , m_step(step)
{
    return;
}


void Decimation::initialize()
{
    Filter::initialize();

    if (m_step < 1)
    {
        throw pdal_error("Decimation step value cannot be less than 1!");
    }
    this->setNumPoints(this->getNumPoints() / m_step);

    log()->get(logDEBUG) << "decimation step: " << m_step << std::endl;

    return;
}


const Options Decimation::getDefaultOptions() const
{
    Options options;
    return options;
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
        srcIndex += m_step - ((srcStartIndex+srcIndex) % (m_step)) + offset;
    } else 
    {
        srcIndex = srcIndex + offset;
    }

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



namespace iterators
{
namespace sequential
{


Decimation::Decimation(const pdal::filters::Decimation& filter, PointBuffer& buffer)
    : pdal::FilterSequentialIterator(filter, buffer)
    , m_filter(filter)
    , m_startingIndex(0)
    , bDidOffset(false)
{
    return;
}


boost::uint64_t Decimation::skipImpl(boost::uint64_t count)
{
    //return naiveSkipImpl(count);

    // BUG: this is not exactly correct
    m_filter.log()->get(logDEBUG) << "Skipping " << count<< " points " << std::endl;
    return getPrevIterator().skip(count );
}


bool Decimation::atEndImpl() const
{
    const StageSequentialIterator& iter = getPrevIterator();
    return iter.atEnd();
}

boost::uint32_t Decimation::readBufferImpl(PointBuffer& data)
{
    // The client has asked us for dstData.getCapacity() points.
    // We will read from our previous stage until we get that amount (or
    // until the previous stage runs out of points).

    boost::uint32_t originalCapacity = data.getCapacity();
    boost::int64_t numPointsNeeded = static_cast<boost::int64_t>(data.getCapacity());


    if (numPointsNeeded <=0 )
        throw pdal_error("numPointsNeeded is <=0!");
        
    // we've established numPointsNeeded is > 0
    PointBuffer outputData(data.getSchema(), static_cast<boost::uint32_t>(numPointsNeeded));
    PointBuffer tmpData(data.getSchema(), static_cast<boost::uint32_t>(numPointsNeeded));

    m_filter.log()->get(logDEBUG2) << "Fetching for block of size: " << numPointsNeeded << std::endl;
    
    m_startingIndex =  getPrevIterator().getIndex();

    while (numPointsNeeded > 0)
    {
        if (getPrevIterator().atEnd()) 
        {
            m_filter.log()->get(logDEBUG4) << "previous iterator is .atEnd, stopping"  
                                               << std::endl;
            break;
        }

        // Only read a maximum of the number of points we need 
        // to fill this buffer
        
        if (data.getCapacity() < numPointsNeeded)
            data.resize(static_cast<boost::uint32_t>(numPointsNeeded));

        boost::uint32_t numRead = getPrevIterator().read(data);
        m_filter.log()->get(logDEBUG4) << "Fetched " 
                                           << numRead << " from previous iterator. "  
                                           << std::endl;

        if (tmpData.getCapacity() < numRead)
            tmpData.resize(numRead);
        
        // assert(numRead <= numPointsNeeded);


        boost::uint32_t numKept = m_filter.processBuffer(tmpData, data, m_startingIndex);
        m_filter.log()->get(logDEBUG4) << "Kept " << numKept << " in decimation filter" << std::endl;
        
        data.resize(originalCapacity);
        m_startingIndex = getPrevIterator().getIndex();      
        
        m_filter.log()->get(logDEBUG4) << tmpData.getNumPoints() << " in temp buffer from filter" << std::endl;
        m_filter.log()->get(logDEBUG4) << "Starting index is now "<< m_startingIndex << std::endl;
                
        if (tmpData.getNumPoints() > 0)
        {
            outputData.copyPointsFast(outputData.getNumPoints(), 0, tmpData, tmpData.getNumPoints());
            outputData.setNumPoints(outputData.getNumPoints() + tmpData.getNumPoints());
            tmpData.setNumPoints(0);
            tmpData.resize(originalCapacity);
        }

        numPointsNeeded -= numKept;
        m_filter.log()->get(logDEBUG4) << numPointsNeeded << " left to fill this block" << std::endl;
        if ( numPointsNeeded <= 0)
        {
            m_filter.log()->get(logDEBUG4) << "numPointsNeeded <=0, stopping"  
                                               << std::endl;
            break;
        }

    }

    const boost::uint32_t numPointsAchieved = outputData.getNumPoints();
    
    data.resize(originalCapacity);
    data.setNumPoints(0);
    data.copyPointsFast(0, 0, outputData, outputData.getNumPoints());
    data.setNumPoints(outputData.getNumPoints());
    m_filter.log()->get(logDEBUG2) << "Copying " << outputData.getNumPoints() << " at end of readBufferImpl" << std::endl;

    return numPointsAchieved;

}
// 
// boost::uint32_t Decimation::readBufferImpl(PointBuffer& dstData)
// {
//     // The client has asked us for dstData.getCapacity() points.
//     // We will read from our previous stage until we get that amount (or
//     // until the previous stage runs out of points).
// 
//     boost::uint32_t numPointsNeeded = dstData.getCapacity();
//     assert(dstData.getNumPoints() == 0);
// 
//     // set up buffer to be filled by prev stage
//     PointBuffer srcData(dstData.getSchema(), numPointsNeeded);
// 
//     while (numPointsNeeded > 0)
//     {
//         // read from prev stage
//         const boost::uint64_t srcStartIndex = getPrevIterator().getIndex();
//         const boost::uint32_t numSrcPointsRead = getPrevIterator().read(srcData);
//         assert(numSrcPointsRead <= srcData.getNumPoints());
//         //assert(numSrcPointsRead <= numPointsNeeded);
// 
//         // we got no data, and there is no more to get -- exit the loop
//         if (numSrcPointsRead == 0) break;
// 
//         // copy points from src (prev stage) into dst (our stage),
//         // based on the CropFilter's rules (i.e. its bounds)
//         const boost::uint32_t numPointsAdded = m_filter.processBuffer(dstData, srcData, srcStartIndex);
// 
//         numPointsNeeded -= numPointsAdded;
//         //printf(".");fflush(stdout);
//     }
// 
//     const boost::uint32_t numPointsAchieved = dstData.getNumPoints();
//     return numPointsAchieved;
// }
// 


}
} // iterators::sequential
}
} // namespaces
