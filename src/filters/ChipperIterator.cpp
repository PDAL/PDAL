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

#include <libpc/filters/ChipperIterator.hpp>


namespace libpc { namespace filters {


ChipperSequentialIterator::ChipperSequentialIterator(Chipper const& filter)
    : libpc::FilterSequentialIterator(filter)
    , m_chipper(filter)
    , m_currentBlockId(0)
    , m_currentPointCount(0)
{
    const_cast<Chipper&>(m_chipper).Chip();
    return;
}

boost::uint64_t ChipperSequentialIterator::skipImpl(boost::uint64_t count)
{
    return naiveSkipImpl(count);
}


boost::uint32_t ChipperSequentialIterator::readImpl(PointBuffer& buffer)
{

    if (m_currentBlockId == m_chipper.GetBlockCount())
        return 0; // we're done.


    buffer.setNumPoints(0);

    filters::chipper::Block const& block = m_chipper.GetBlock(m_currentBlockId);
    std::size_t numPointsThisBlock = block.GetIDs().size();
    m_currentPointCount = m_currentPointCount + numPointsThisBlock;
    
    if (buffer.getCapacity() < numPointsThisBlock)
    {
        // FIXME: Expand the buffer?
        throw libpc_error("Buffer not large enough to hold block!");
    }
    block.GetBuffer(m_chipper.getPrevStage(), buffer, m_currentBlockId);

    buffer.setSpatialBounds(block.GetBounds());
    m_currentBlockId++;
    return numPointsThisBlock;

}

bool ChipperSequentialIterator::atEndImpl() const
{
    // we don't have a fixed point point --
    // we are at the end only when our source is at the end
    const SequentialIterator& iter = getPrevIterator();
    return iter.atEnd();
}

} } // namespaces
