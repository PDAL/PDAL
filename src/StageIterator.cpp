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

static boost::uint32_t s_defaultChunkSize = 65536;


//---------------------------------------------------------------------------
//
// StageIterator
//
//---------------------------------------------------------------------------

StageIterator::StageIterator(const Stage& stage, PointBuffer& buffer)
    : m_index(0)
    , m_stage(stage)
    , m_buffer(buffer)
    , m_chunkSize(s_defaultChunkSize)
    , m_readBeginPerformed(false)
    , m_readBufferBeginPerformed(false)
{
    return;
}


StageIterator::~StageIterator()
{
    // sanity checks (these are not exceptions, because dtors should not throw)
    //assert(!m_readBufferBeginPerformed);
    //assert(!m_readBeginPerformed);

    return;
}


const Stage& StageIterator::getStage() const
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


boost::uint32_t StageIterator::read(PointBuffer& buffer)
{
    readBegin();
    readBufferBegin(buffer);
    const boost::uint32_t numRead = readBuffer(buffer);
    readBufferEnd(buffer);
    readEnd();

    return numRead;
}


void StageIterator::readBegin()
{
    if (!m_stage.isInitialized())
    {
        throw pdal_error("stage not initialized: " + m_stage.getName());
    }

    if (m_readBeginPerformed)
    {
        throw pdal_error("readBegin called without corresponding readEnd");
    }
    m_readBeginPerformed = false;

    readBeginImpl();

    m_readBeginPerformed = true;

    return;
}


void StageIterator::readBufferBegin(PointBuffer& buffer)
{
    if (!m_readBeginPerformed)
    {
        throw pdal_error("readBufferBegin called without corresponding readBegin");
    }
    if (m_readBufferBeginPerformed)
    {
        throw pdal_error("readBufferBegin called without corresponding readBufferEnd");
    }
    m_readBufferBeginPerformed = false;

    readBufferBeginImpl(buffer);

    m_readBufferBeginPerformed = true;

    return;
}


boost::uint32_t StageIterator::readBuffer(PointBuffer& buffer)
{
    if (!m_readBufferBeginPerformed)
    {
        throw pdal_error("readBuffer called without corresponding readBufferBegin");
    }

    boost::uint32_t numRead = 0;

    
    numRead = readBufferImpl(buffer);

    m_index += numRead;

    return numRead;
}


void StageIterator::readBufferEnd(PointBuffer& buffer)
{
    if (!m_readBufferBeginPerformed)
    {
        throw pdal_error("readBufferEnd called without corresponding readBufferBegin");
    }

    readBufferEndImpl(buffer);
    
    m_readBufferBeginPerformed = false;

    return;
}


void StageIterator::readEnd()
{
    if (m_readBufferBeginPerformed)
    {
        throw pdal_error("readEnd called without corresponding readBufferEnd");
    }

    if (!m_readBeginPerformed)
    {
        throw pdal_error("readEnd called without corresponding readBegin");
    }

    readEndImpl();

    m_readBeginPerformed = false;

    return;
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

StageSequentialIterator::StageSequentialIterator(const Stage& stage, PointBuffer& buffer)
    : StageIterator(stage, buffer)
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

StageRandomIterator::StageRandomIterator(const Stage& stage, PointBuffer& buffer)
    : StageIterator(stage, buffer)
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




} // namespace pdal
