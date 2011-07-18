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

#include <pdal/StageIterator.hpp>

#include <algorithm> // for std::min/max

#include <pdal/Stage.hpp>
#include <pdal/PointBuffer.hpp>

namespace pdal
{

static boost::uint32_t s_defaultChunkSize = 1024;


//---------------------------------------------------------------------------
//
// StageIterator
//
//---------------------------------------------------------------------------

StageIterator::StageIterator(const DataStage& stage)
    : m_index(0)
    , m_stage(stage)
    , m_chunkSize(s_defaultChunkSize)
{
    return;
}


StageIterator::~StageIterator()
{
    return;
}


const DataStage& StageIterator::getStage() const
{
    return m_stage;
}


boost::uint64_t StageIterator::getIndex() const
{
    return m_index;
}


void StageIterator::setChunkSize(boost::uint32_t size)
{
    m_chunkSize = size;
}


boost::uint32_t StageIterator::getChunkSize() const
{
    return m_chunkSize;
}


boost::uint32_t StageIterator::read(PointBuffer& data)
{
    const boost::uint32_t numRead = readImpl(data);

    m_index += numRead;

    return numRead;
}


boost::uint64_t StageIterator::naiveSkipImpl(boost::uint64_t count)
{
    boost::uint64_t totalNumRead = 0;

    // read (and discard) all the next 'count' points
    // in case count is really big, we do this in blocks of size 'chunk'
    while (count > 0)
    {
        const boost::uint64_t thisCount64 = std::min<boost::uint64_t>(getChunkSize(), count);
        // getChunkSize is a uint32, so this cast is safe
        const boost::uint32_t thisCount = static_cast<boost::uint32_t>(thisCount64);

        PointBuffer junk(getStage().getSchema(), thisCount);
        
        const boost::uint32_t numRead = read(junk);
        if (numRead == 0) break; // end of file or something

        count -= numRead;
        totalNumRead += numRead;
    }

    return totalNumRead;
}


//---------------------------------------------------------------------------
//
// StageSequentialIterator
//
//---------------------------------------------------------------------------

StageSequentialIterator::StageSequentialIterator(const DataStage& stage)
    : StageIterator(stage)
{
    return;
}


StageSequentialIterator::~StageSequentialIterator()
{
    return;
}


boost::uint64_t StageSequentialIterator::skip(boost::uint64_t count)
{
    const boost::uint64_t numSkipped = skipImpl(count);

    m_index += numSkipped;

    return numSkipped;
}


bool StageSequentialIterator::atEnd() const
{
    return atEndImpl();
}


//---------------------------------------------------------------------------
//
// StageRandomIterator
//
//---------------------------------------------------------------------------

StageRandomIterator::StageRandomIterator(const DataStage& stage)
    : StageIterator(stage)
{
    return;
}


StageRandomIterator::~StageRandomIterator()
{
    return;
}


boost::uint64_t StageRandomIterator::seek(boost::uint64_t position)
{
    const boost::uint64_t newPos = seekImpl(position);

    m_index = newPos;

    return newPos;
}


//---------------------------------------------------------------------------
//
// StageBlockIterator
//
//---------------------------------------------------------------------------

StageBlockIterator::StageBlockIterator(const DataStage& stage)
    : StageIterator(stage)
{
    return;
}


StageBlockIterator::~StageBlockIterator()
{
    return;
}



} // namespace pdal
